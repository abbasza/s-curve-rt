//! C-ABI (`extern "C"`) and WebAssembly (`wasm-bindgen`) interoperability layer.

use crate::solver::SCurveSolver;
use crate::state::{KinematicLimits, KinematicState};

/// C-ABI entry point: advances a 1-DOF S-curve controller by `dt` seconds.
/// Returns `0` on success, or a non-zero `TrajectoryError` code on failure.
///
/// # Safety
/// `limits`, `current_state`, and `out_next_state` must be valid, non-null aligned pointers.
#[no_mangle]
pub unsafe extern "C" fn s_curve_step_c(
    limits: *const KinematicLimits,
    current_state: *const KinematicState,
    target_pos: f64,
    target_vel: f64,
    dt: f64,
    out_next_state: *mut KinematicState,
) -> i32 {
    if limits.is_null() || current_state.is_null() || out_next_state.is_null() {
        return -1;
    }

    // SAFETY: Caller guarantees non-null aligned pointers.
    let lim = unsafe { *limits };
    let curr = unsafe { *current_state };

    let solver = match SCurveSolver::new(lim) {
        Ok(s) => s,
        Err(e) => return e as i32,
    };

    match solver.update_step(curr, target_pos, target_vel, dt) {
        Ok(next) => {
            // SAFETY: Caller guarantees out_next_state is valid and writable.
            unsafe { *out_next_state = next };
            0
        }
        Err(e) => e as i32,
    }
}

#[cfg(feature = "wasm")]
pub mod wasm_api {
    use super::*;
    use wasm_bindgen::prelude::*;

    /// WebAssembly wrapper for browser/Wasmtime trajectory evaluation.
    #[wasm_bindgen]
    pub struct WasmSCurvePlanner {
        solver: SCurveSolver,
    }

    #[wasm_bindgen]
    impl WasmSCurvePlanner {
        #[wasm_bindgen(constructor)]
        pub fn new(v_max: f64, a_max: f64, j_max: f64) -> Result<WasmSCurvePlanner, JsValue> {
            let limits = KinematicLimits::new(v_max, a_max, j_max)
                .map_err(|e| JsValue::from_str(&format!("Invalid limits: {e:?}")))?;
            let solver = SCurveSolver::new(limits)
                .map_err(|e| JsValue::from_str(&format!("Solver init error: {e:?}")))?;
            Ok(Self { solver })
        }

        /// Computes total duration of a rest-to-rest trajectory.
        pub fn compute_duration(&self, start_pos: f64, target_pos: f64) -> Result<f64, JsValue> {
            let profile = self
                .solver
                .plan_rest_to_rest(start_pos, target_pos, None)
                .map_err(|e| JsValue::from_str(&format!("Plan error: {e:?}")))?;
            Ok(profile.total_time)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::multi_dof::MultiDofSynchronizer;
    use approx::assert_relative_eq;

    #[test]
    fn test_exact_rest_to_rest_boundary_conditions() {
        let limits = KinematicLimits::new(2.0, 4.0, 20.0).unwrap();
        let solver = SCurveSolver::new(limits).unwrap();

        let profile = solver.plan_rest_to_rest(0.0, 5.0, None).unwrap();
        let initial = KinematicState::new(0.0, 0.0, 0.0);

        let (final_state, _) = profile.sample_at(initial, profile.total_time);
        assert_relative_eq!(final_state.position, 5.0, epsilon = 1e-6);
        assert_relative_eq!(final_state.velocity, 0.0, epsilon = 1e-6);
        assert_relative_eq!(final_state.acceleration, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_3dof_synchronization_matches_duration() {
        let lim1 = KinematicLimits::new(2.0, 4.0, 20.0).unwrap();
        let lim2 = KinematicLimits::new(3.0, 5.0, 30.0).unwrap();
        let lim3 = KinematicLimits::new(1.5, 2.0, 10.0).unwrap();

        let sync = MultiDofSynchronizer::new([lim1, lim2, lim3]).unwrap();
        let profiles = sync
            .plan_synchronized(&[0.0, 0.0, 0.0], &[1.0, 4.0, 2.5])
            .unwrap();

        assert_relative_eq!(profiles[0].total_time, profiles[1].total_time, epsilon = 1e-3);
        assert_relative_eq!(profiles[1].total_time, profiles[2].total_time, epsilon = 1e-3);
    }
}
