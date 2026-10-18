use s_curve_rt::{KinematicLimits, KinematicState, SCurveSolver};

fn main() {
    let limits = KinematicLimits::new(1.5, 3.0, 15.0).expect("valid limits");
    let solver = SCurveSolver::new(limits).expect("solver init");

    let dt = 0.001; // 1 kHz hard real-time period
    let mut state = KinematicState::new(0.0, 0.0, 0.0);
    let mut target_pos = 1.0;

    println!("time_s,pos,vel,acc,jerk,target_pos");
    for step in 0..2500 {
        let t = f64::from(step) * dt;
        // Simulate online setpoint override at t = 0.6s (e.g. obstacle detected by ToF sensor)
        if step == 600 {
            target_pos = 0.4;
        }
        state = solver
            .update_step(state, target_pos, 0.0, dt)
            .expect("step valid");
        if step % 25 == 0 {
            println!(
                "{t:.3},{:.5},{:.5},{:.5},{:.2},{target_pos:.2}",
                state.position, state.velocity, state.acceleration, state.jerk
            );
        }
    }
}
