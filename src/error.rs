//! Error definitions for kinematic constraints and numerical solver verification.

/// Errors returned during trajectory configuration or step evaluation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub enum TrajectoryError {
    /// Velocity, acceleration, or jerk limit is non-positive or NaN.
    InvalidKinematicLimits = 1,
    /// Input state contains NaN or infinite values.
    NonFiniteState = 2,
    /// Sample period `dt` is zero, negative, or non-finite.
    InvalidTimeStep = 3,
    /// Numerical convergence failed within the deterministic iteration budget.
    ConvergenceFailure = 4,
}
