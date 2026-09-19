use kalman::filter::{Filter, RollingAverage};
use kalman::sensor::{GaussianSensor, Sensor};
use kalman::state::{Oscillating, State};

use kalman::plot::{Sample, SampleLog};

const STD_DEV: f64 = 0.1;
const SEED: u64 = 67;

const DT: f64 = 0.1;

fn main() {
    let mut state = Oscillating {
        amplitude: 1.0,
        angular_frequency: 1.0,
        phase: 0.0,
        t: 0.0,
    };

    let mut sensor = GaussianSensor::new(STD_DEV, SEED).unwrap();
    let mut filter = RollingAverage::<4>::new();

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

    println!("rms: {}", samples.rms());
    samples.save();
}
