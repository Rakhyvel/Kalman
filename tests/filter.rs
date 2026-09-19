use kalman::filter::Filter;
use kalman::sensor::Sensor;
use kalman::state::State;

#[test]
fn basic() {
    let mut state = kalman::state::ConstantVelocity {
        velocity: 1.0,
        position: 0.0,
    };
    let mut sensor = kalman::sensor::GaussianSensor::new(0.1, 67).unwrap();
    let mut filter = kalman::filter::MovingAverage::new(0.5, 0.0).unwrap();

    const DT: f64 = 0.1;
    const ABS_TOL: f64 = 1.0;
    for i in 0..100 {
        let t = i as f64 * DT;

        let msr = sensor.measure(&state);
        assert!(msr.is_finite());

        let est = filter.update(msr);
        assert!(est.is_finite());

        let err = (est - state.state()).abs();
        assert!(
            err < ABS_TOL,
            "t={t:.2}, measurement={msr:.4}, estimate={est:.4}, \
             truth={:.4}, error={err:.4}",
            state.position
        );

        state.step(DT);
    }
}
