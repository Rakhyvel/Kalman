use kalman::filter::{Filter, Kalman};
use kalman::sensor::{GaussianSensor, Sensor};
use kalman::state::{Oscillating, State};

use kalman::plot::{Sample, SampleLog};
use nalgebra::{SMatrix, SVector};

const STD_DEV: f64 = 0.2;
const SEED: u64 = 67;

const DT: f64 = 0.1;

const Q: f64 = 0.5;
const R: f64 = STD_DEV * STD_DEV;

fn main() {
    let mut state = Oscillating {
        amplitude: 1.0,
        angular_frequency: 1.0,
        phase: 0.0,
        t: 0.0,
    };

    let mut sensor = GaussianSensor::new(STD_DEV, SEED).unwrap();

    let mut filter = Kalman::<2, 1, 1>::new(
        SVector::<f64, 2>::new(0.0, 0.0),
        SMatrix::<f64, 2, 2>::new(1.0, 0.0, 0.0, 1.0),
        SMatrix::<f64, 2, 2>::new(1.0, DT, 0.0, 1.0),
        SMatrix::<f64, 2, 1>::new(0.5 * DT * DT, DT),
        SMatrix::<f64, 1, 1>::new(Q),
        SMatrix::<f64, 1, 2>::new(1.0, 0.0),
        SMatrix::<f64, 1, 1>::new(R),
    );

    let mut samples = SampleLog::new();

    const MAX_ITERS: usize = 100;
    for i in 0..MAX_ITERS {
        let t = i as f64 * DT;

        let truth = state.state();
        let measurement = SVector::<f64, 1>::new(sensor.measure(&state));
        let estimate: SVector<f64, 2> = filter.update(measurement);
        let covariance = filter.p();

        samples.record(Sample {
            t,
            truth,
            measurement: measurement.x,
            estimate: estimate.x,
            covariance: covariance[(0, 0)],
        });

        state.step(DT);
    }

    println!("rms: {}", samples.rms());
    samples.save();
}
