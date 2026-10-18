//! Analytical and bounded-iteration S-curve solver for `#![no_std]` environments.

use crate::error::TrajectoryError;
use crate::profile::StepProfile;
use crate::state::{KinematicLimits, KinematicState};

#[inline]
fn sqrt_f64(x: f64) -> f64 {
    #[cfg(feature = "libm")]
    {
        libm::sqrt(x)
    }
    #[cfg(all(not(feature = "libm"), feature = "std"))]
    {
        x.sqrt()
    }
}

#[inline]
fn abs_f64(x: f64) -> f64 {
    #[cfg(feature = "libm")]
    {
        libm::fabs(x)
    }
    #[cfg(all(not(feature = "libm"), feature = "std"))]
    {
        x.abs()
    }
}

/// Online deterministic S-curve solver and step generator.
#[derive(Debug, Clone, Copy)]
pub struct SCurveSolver {
    limits: KinematicLimits,
}

impl SCurveSolver {
    /// Creates a new solver with validated physical actuator limits.
    pub fn new(limits: KinematicLimits) -> Result<Self, TrajectoryError> {
        limits.validate()?;
        Ok(Self { limits })
    }

    /// Computes a time-optimal 7-segment rest-to-rest S-curve profile for a displacement $\Delta p$,
    /// optionally stretched to match a minimum duration `min_duration` (used for multi-axis sync).
    pub fn plan_rest_to_rest(
        &self,
        start_pos: f64,
        target_pos: f64,
        min_duration: Option<f64>,
    ) -> Result<StepProfile, TrajectoryError> {
        if !start_pos.is_finite() || !target_pos.is_finite() {
            return Err(TrajectoryError::NonFiniteState);
        }

        let delta = target_pos - start_pos;
        let sign = if delta >= 0.0 { 1.0 } else { -1.0 };
        let dist = abs_f64(delta);

        if dist < 1e-12 {
            return Ok(StepProfile {
                durations: [0.0; 7],
                jerks: [0.0; 7],
                total_time: 0.0,
            });
        }

        // If a synchronization duration is requested, scale effective velocity/acceleration
        // deterministically via fixed-step bisection (24 iterations = 2^-24 precision, zero heap).
        let mut v_max = self.limits.max_velocity;
        let a_max = self.limits.max_acceleration;
        let j_max = self.limits.max_jerk;

        let base_profile = Self::compute_symmetric_7seg(dist, sign, v_max, a_max, j_max);

        if let Some(target_T) = min_duration {
            if target_T > base_profile.total_time {
                let mut low = 1e-6 * v_max;
                let mut high = v_max;
                for _ in 0..28 {
                    let mid = 0.5 * (low + high);
                    let candidate = Self::compute_symmetric_7seg(dist, sign, mid, a_max, j_max);
                    if candidate.total_time < target_T {
                        high = mid;
                    } else {
                        low = mid;
                    }
                }
                v_max = 0.5 * (low + high);
                return Ok(Self::compute_symmetric_7seg(dist, sign, v_max, a_max, j_max));
            }
        }

        Ok(base_profile)
    }

    /// Online closed-loop step update (`update_step`) suitable for a 1 kHz MCU ISR.
    /// Handles arbitrary non-zero initial velocity and acceleration (e.g., mid-motion target changes).
    pub fn update_step(
        &self,
        current: KinematicState,
        target_pos: f64,
        target_vel: f64,
        dt: f64,
    ) -> Result<KinematicState, TrajectoryError> {
        current.validate()?;
        if !target_pos.is_finite() || !target_vel.is_finite() {
            return Err(TrajectoryError::NonFiniteState);
        }
        if dt <= 0.0 || !dt.is_finite() {
            return Err(TrajectoryError::InvalidTimeStep);
        }

        let j_max = self.limits.max_jerk;
        let a_max = self.limits.max_acceleration;
        let v_max = self.limits.max_velocity;

        // Compute minimum distance required to bring current acceleration to 0:
        let t_acc_zero = abs_f64(current.acceleration) / j_max;
        let v_at_acc_zero = current.velocity
            + 0.5 * current.acceleration * t_acc_zero;

        // Stopping distance estimation from current velocity to target_vel
        let vel_err = target_vel - v_at_acc_zero;
        let pos_err = target_pos - current.position;

        // Calculate time-optimal braking distance
        let braking_dist = (current.velocity * abs_f64(current.velocity)) / (2.0 * a_max)
            + abs_f64(current.velocity) * (a_max / (2.0 * j_max));

        let dir = if pos_err >= braking_dist {
            1.0
        } else if pos_err <= -braking_dist {
            -1.0
        } else if vel_err >= 0.0 {
            1.0
        } else {
            -1.0
        };

        // Determine desired velocity bounded by [ -v_max, +v_max ]
        let raw_desired_vel = dir * sqrt_f64(2.0 * a_max * abs_f64(pos_err)).min(v_max);
        let vel_delta = raw_desired_vel - current.velocity;

        // Minimum time to ramp acceleration to 0
        let stopping_vel_change =
            (current.acceleration * abs_f64(current.acceleration)) / (2.0 * j_max);

        let applied_jerk = if vel_delta > stopping_vel_change && current.acceleration < a_max {
            j_max
        } else if vel_delta < -stopping_vel_change && current.acceleration > -a_max {
            -j_max
        } else if current.acceleration > 0.0 {
            -j_max
        } else if current.acceleration < 0.0 {
            j_max
        } else {
            0.0
        };

        let mut next = StepProfile::integrate_segment(current, applied_jerk, dt);

        // Enforce hard physical saturation limits
        next.acceleration = next.acceleration.clamp(-a_max, a_max);
        next.velocity = next.velocity.clamp(-v_max, v_max);

        // Snap cleanly at target within numerical tolerance
        if abs_f64(target_pos - next.position) < 1e-7
            && abs_f64(target_vel - next.velocity) < 1e-6
            && abs_f64(next.acceleration) < 1e-5
        {
            next.position = target_pos;
            next.velocity = target_vel;
            next.acceleration = 0.0;
            next.jerk = 0.0;
        }

        Ok(next)
    }

    #[inline]
    fn compute_symmetric_7seg(
        dist: f64,
        sign: f64,
        v_max: f64,
        a_max: f64,
        j_max: f64,
    ) -> StepProfile {
        let mut tj = a_max / j_max;
        let mut ta = (v_max / a_max) - tj;

        // Case 1: Velocity limit is reached before acceleration limit (short triangular jerk pulse)
        if ta < 0.0 {
            tj = sqrt_f64(v_max / j_max);
            ta = 0.0;
        }

        let a_peak = j_max * tj;
        let v_peak = a_peak * (tj + ta);

        // Distance covered during acceleration and deceleration phases alone (tv = 0)
        let d_acc_dec = v_peak * (2.0 * tj + ta);

        let tv = if dist >= d_acc_dec {
            (dist - d_acc_dec) / v_peak
        } else {
            // Distance is too short to reach v_max; solve for reduced ta or reduced tj
            if dist >= 2.0 * j_max * tj * tj * tj {
                // a_max is still reached, solve quadratic for ta:
                // a_max * ta^2 + 3 * a_max * tj * ta + 2 * a_max * tj^2 - dist = 0
                let disc = (9.0 * tj * tj) - 4.0 * (2.0 * tj * tj - dist / a_max);
                ta = 0.5 * (-3.0 * tj + sqrt_f64(disc.max(0.0)));
            } else {
                // Neither v_max nor a_max is reached; pure 4-segment cubic jerk profile:
                // dist = 2 * j_max * tj^3  =>  tj = (dist / (2 * j_max))^(1/3)
                ta = 0.0;
                tj = Self::cbrt_positive(dist / (2.0 * j_max));
            }
            0.0
        };

        let durations = [tj, ta, tj, tv, tj, ta, tj];
        let j = sign * j_max;
        let jerks = [j, 0.0, -j, 0.0, -j, 0.0, j];
        let total_time = 4.0 * tj + 2.0 * ta + tv;

        StepProfile {
            durations,
            jerks,
            total_time,
        }
    }

    /// Fast, deterministic Halley's method cube-root for `x >= 0` without heap or `std`.
    #[inline]
    fn cbrt_positive(x: f64) -> f64 {
        if x <= 0.0 {
            return 0.0;
        }
        #[cfg(feature = "libm")]
        {
            libm::cbrt(x)
        }
        #[cfg(all(not(feature = "libm"), feature = "std"))]
        {
            x.cbrt()
        }
    }
}
