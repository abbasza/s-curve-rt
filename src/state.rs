//! Kinematic state and physical actuator constraint structures.

use crate::error::TrajectoryError;

/// Instantaneous 1-DOF kinematic state of an actuator.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct KinematicState {
    /// Position $p(t)$ in physical units (e.g., rad or m).
    pub position: f64,
    /// Velocity $v(t) = \dot{p}(t)$ (e.g., rad/s or m/s).
    pub velocity: f64,
    /// Acceleration $a(t) = \ddot{p}(t)$ (e.g., rad/s² or m/s²).
    pub acceleration: f64,
    /// Applied jerk $j(t) = \dddot{p}(t)$ (e.g., rad/s³ or m/s³).
    pub jerk: f64,
}

impl KinematicState {
    /// Creates a new kinematic state.
    #[must_use]
    pub const fn new(position: f64, velocity: f64, acceleration: f64) -> Self {
        Self {
            position,
            velocity,
            acceleration,
            jerk: 0.0,
        }
    }

    /// Validates that all state variables are finite numbers.
    pub fn validate(&self) -> Result<(), TrajectoryError> {
        if self.position.is_finite()
            && self.velocity.is_finite()
            && self.acceleration.is_finite()
            && self.jerk.is_finite()
        {
            Ok(())
        } else {
            Err(TrajectoryError::NonFiniteState)
        }
    }
}

/// Symmetric physical limits for a 1-DOF actuator.
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct KinematicLimits {
    /// Maximum allowed velocity $v_{\max} > 0$.
    pub max_velocity: f64,
    /// Maximum allowed acceleration $a_{\max} > 0$.
    pub max_acceleration: f64,
    /// Maximum allowed jerk $j_{\max} > 0$.
    pub max_jerk: f64,
}

impl KinematicLimits {
    /// Creates and validates physical actuator limits.
    pub fn new(
        max_velocity: f64,
        max_acceleration: f64,
        max_jerk: f64,
    ) -> Result<Self, TrajectoryError> {
        let limits = Self {
            max_velocity,
            max_acceleration,
            max_jerk,
        };
        limits.validate()?;
        Ok(limits)
    }

    /// Validates that all limits are strictly positive and finite.
    pub fn validate(&self) -> Result<(), TrajectoryError> {
        if self.max_velocity > 0.0
            && self.max_velocity.is_finite()
            && self.max_acceleration > 0.0
            && self.max_acceleration.is_finite()
            && self.max_jerk > 0.0
            && self.max_jerk.is_finite()
        {
            Ok(())
        } else {
            Err(TrajectoryError::InvalidKinematicLimits)
        }
    }
}
