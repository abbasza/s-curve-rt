//! 7-segment S-curve phase representation and exact analytical integration.

use crate::state::KinematicState;

/// The 7 canonical phases of a jerk-limited S-curve trajectory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub enum Phase {
    /// Phase 1: Jerk is $+j_{\max}$ or $-j_{\max}$ building acceleration.
    JerkRampUp = 0,
    /// Phase 2: Constant maximum acceleration ($j = 0$).
    ConstantAcceleration = 1,
    /// Phase 3: Jerk reverses to ramp acceleration back to zero.
    JerkRampDown = 2,
    /// Phase 4: Constant cruise velocity ($a = 0, j = 0$).
    CruiseVelocity = 3,
    /// Phase 5: Jerk ramps deceleration up.
    DecelJerkUp = 4,
    /// Phase 6: Constant maximum deceleration ($j = 0$).
    ConstantDeceleration = 5,
    /// Phase 7: Jerk ramps deceleration to zero at target.
    DecelJerkDown = 6,
    /// Target reached; actuator is holding position.
    Completed = 7,
}

/// Analytical 7-segment time and jerk profile for a 1D point-to-point motion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StepProfile {
    /// Duration of each of the 7 segments $t_i \ge 0$.
    pub durations: [f64; 7],
    /// Constant jerk applied during each of the 7 segments.
    pub jerks: [f64; 7],
    /// Total duration $T = \sum_{i=0}^6 t_i$.
    pub total_time: f64,
}

impl StepProfile {
    /// Exact third-order Taylor integration over time interval $\Delta t$ with constant jerk $j$:
    /// $$p(t + \Delta t) = p(t) + v(t)\Delta t + \frac{1}{2}a(t)\Delta t^2 + \frac{1}{6}j\Delta t^3$$
    #[inline]
    #[must_use]
    pub fn integrate_segment(state: KinematicState, jerk: f64, dt: f64) -> KinematicState {
        let dt2 = dt * dt;
        let dt3 = dt2 * dt;

        let next_pos = state.position
            + state.velocity * dt
            + 0.5 * state.acceleration * dt2
            + (1.0 / 6.0) * jerk * dt3;
        let next_vel = state.velocity + state.acceleration * dt + 0.5 * jerk * dt2;
        let next_acc = state.acceleration + jerk * dt;

        KinematicState {
            position: next_pos,
            velocity: next_vel,
            acceleration: next_acc,
            jerk,
        }
    }

    /// Evaluates the exact kinematic state at arbitrary elapsed time $t \in [0, T]$.
    #[must_use]
    pub fn sample_at(&self, initial: KinematicState, elapsed: f64) -> (KinematicState, Phase) {
        if elapsed <= 0.0 {
            return (initial, Phase::JerkRampUp);
        }

        let mut remaining = elapsed;
        let mut current = initial;

        let phases = [
            Phase::JerkRampUp,
            Phase::ConstantAcceleration,
            Phase::JerkRampDown,
            Phase::CruiseVelocity,
            Phase::DecelJerkUp,
            Phase::ConstantDeceleration,
            Phase::DecelJerkDown,
        ];

        for i in 0..7 {
            let seg_dt = self.durations[i];
            let seg_jerk = self.jerks[i];

            if remaining < seg_dt {
                let interpolated = Self::integrate_segment(current, seg_jerk, remaining);
                return (interpolated, phases[i]);
            }
            if seg_dt > 0.0 {
                current = Self::integrate_segment(current, seg_jerk, seg_dt);
                remaining -= seg_dt;
            }
        }

        current.jerk = 0.0;
        (current, Phase::Completed)
    }
}
