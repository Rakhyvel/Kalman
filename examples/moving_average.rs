use kalman::filter::{Filter, MovingAverage};
use kalman::sensor::{GaussianSensor, Sensor};
use kalman::state::{ConstantVelocity, State};

use kalman::plot::{Sample, SampleLog};

const STD_DEV: f64 = 0.9;
const SEED: u64 = 67;

const ALPHA: f64 = 0.29;
const INITIAL_ESTIMATE: f64 = 0.0;

const DT: f64 = 0.1;

fn main() {
    let mut state = ConstantVelocity {
        velocity: 1.0,
        position: 0.0,
    };

    let mut sensor = GaussianSensor::new(STD_DEV, SEED).unwrap();
    let mut filter = MovingAverage::new(ALPHA, INITIAL_ESTIMATE).unwrap();

    let mut samples = SampleLog::new();

    for i in 0..100 {
        let t = i as f64 * DT;

        let truth = state.state();

        let measurement = sensor.measure(&state);

        let estimate = filter.update(measurement);

        samples.record(Sample {
            t,
            truth,
            measurement,
            estimate,
        });

        state.step(DT);
    }

    samples.save();
}
