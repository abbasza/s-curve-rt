//! Multi-axis phase-synchronized trajectory planner using stack-allocated const generics.

use crate::error::TrajectoryError;
use crate::profile::StepProfile;
use crate::solver::SCurveSolver;
use crate::state::KinematicLimits;

/// Synchronizes `N` robotic degrees of freedom on the stack so all axes arrive simultaneously.
#[derive(Debug, Clone, Copy)]
pub struct MultiDofSynchronizer<const N: usize> {
    solvers: [SCurveSolver; N],
}

impl<const N: usize> MultiDofSynchronizer<N> {
    /// Initializes an `N`-DOF synchronizer from an array of per-axis kinematic limits.
    pub fn new(limits: [KinematicLimits; N]) -> Result<Self, TrajectoryError> {
        let dummy = KinematicLimits {
            max_velocity: 1.0,
            max_acceleration: 1.0,
            max_jerk: 1.0,
        };
        let mut solvers = [SCurveSolver::new(dummy)?; N];
        for i in 0..N {
            solvers[i] = SCurveSolver::new(limits[i])?;
        }
        Ok(Self { solvers })
    }

    /// Plans a phase-synchronized rest-to-rest trajectory across all `N` axes.
    /// First finds the slowest limiting axis $T_{\text{sync}} = \max_i T_i$, then stretches
    /// all faster axes to complete their motion at exactly $T_{\text{sync}}$.
    pub fn plan_synchronized(
        &self,
        start_positions: &[f64; N],
        target_positions: &[f64; N],
    ) -> Result<[StepProfile; N], TrajectoryError> {
        let zero_profile = StepProfile {
            durations: [0.0; 7],
            jerks: [0.0; 7],
            total_time: 0.0,
        };
        let mut profiles = [zero_profile; N];
        let mut max_duration = 0.0_f64;

        // Pass 1: Compute independent time-optimal profiles and find limiting duration
        for i in 0..N {
            profiles[i] =
                self.solvers[i].plan_rest_to_rest(start_positions[i], target_positions[i], None)?;
            if profiles[i].total_time > max_duration {
                max_duration = profiles[i].total_time;
            }
        }

        // Pass 2: Stretch non-limiting axes to match `max_duration`
        for i in 0..N {
            if profiles[i].total_time > 0.0 && profiles[i].total_time < max_duration {
                profiles[i] = self.solvers[i].plan_rest_to_rest(
                    start_positions[i],
                    target_positions[i],
                    Some(max_duration),
                )?;
            }
        }

        Ok(profiles)
    }
}
