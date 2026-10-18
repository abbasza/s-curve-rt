use criterion::{black_box, criterion_group, criterion_main, Criterion};
use s_curve_rt::{KinematicLimits, KinematicState, MultiDofSynchronizer, SCurveSolver};

fn bench_scurve(c: &mut Criterion) {
    let limits = KinematicLimits::new(2.0, 5.0, 25.0).unwrap();
    let solver = SCurveSolver::new(limits).unwrap();
    let state = KinematicState::new(0.2, 0.8, 1.2);

    c.bench_function("1dof_online_step_1khz", |b| {
        b.iter(|| {
            solver
                .update_step(black_box(state), black_box(2.5), black_box(0.0), black_box(0.001))
                .unwrap()
        })
    });

    let sync = MultiDofSynchronizer::new([limits; 3]).unwrap();
    c.bench_function("3dof_synchronized_plan", |b| {
        b.iter(|| {
            sync.plan_synchronized(black_box(&[0.0, 0.0, 0.0]), black_box(&[1.2, -3.4, 0.8]))
                .unwrap()
        })
    });
}

criterion_group!(benches, bench_scurve);
criterion_main!(benches);
