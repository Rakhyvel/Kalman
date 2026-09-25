use kalman::filter::{Filter, Kalman};
use kalman::sensor::{GaussianSensor, Sensor};
use kalman::state::{Oscillating, State};

use kalman::plot::{Sample, SampleLog};

const STD_DEV: f64 = 0.4;
const SEED: u64 = 69;

const INITIAL_ESTIMATE: f64 = 1.0;
const INITIAL_COVARIANCE: f64 = 1.0;

const DT: f64 = 0.1;

const A: f64 = 1.0;
const GAMMA: f64 = 1.0;
const Q: f64 = 0.025529;
const C: f64 = 1.0;
const R: f64 = STD_DEV * STD_DEV;

fn main() {
    let mut state = Oscillating {
        amplitude: 1.0,
        angular_frequency: 1.0,
        phase: 0.0,
        t: 0.0,
    };

    let mut sensor = GaussianSensor::new(STD_DEV, SEED).unwrap();

    let mut filter = Kalman::new(INITIAL_ESTIMATE, INITIAL_COVARIANCE, A, GAMMA, Q, C, R);

    let mut samples = SampleLog::new();

    const MAX_ITERS: usize = 100;
    for i in 0..MAX_ITERS {
        let t = i as f64 * DT;

        let truth = state.state();
        let measurement = sensor.measure(&state);
        let estimate = filter.update(measurement);
        let covariance = filter.p();

        samples.record(Sample {
            t,
            truth,
            measurement,
            estimate,
            covariance,
        });

        state.step(DT);
    }

    println!("rms: {}", samples.rms());
    samples.save();
}
