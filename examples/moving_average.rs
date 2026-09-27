use kalman::filter::{Filter, MovingAverage};
use kalman::sensor::{GaussianSensor, Sensor};
use kalman::truth::{Oscillating, Truth};

use kalman::plot::{Sample, SampleLog};
use nalgebra::{SMatrix, SVector};

const STD_DEV: f64 = 0.1;
const SEED: u64 = 67;

const ALPHA: f64 = 0.435;
const INITIAL_ESTIMATE: f64 = 0.0;

const DT: f64 = 0.1;

fn main() {
    let mut truth = Oscillating {
        amplitude: 1.0,
        angular_frequency: 0.01,
        phase: 0.0,
        t: 0.0,
    };

    let mut sensor =
        GaussianSensor::new(SMatrix::<f64, 1, 2>::new(1.0, 0.0), STD_DEV, SEED).unwrap();
    let mut filter = MovingAverage::new(ALPHA, INITIAL_ESTIMATE).unwrap();

    let mut samples = SampleLog::new();

    for i in 0..100 {
        let t = i as f64 * DT;

        truth.step(DT);
        let truth_state = truth.state()[0];

        let measurement = sensor.measure(&truth.state());

        let estimate: SVector<f64, 1> = filter.update(measurement);

        samples.record(Sample {
            t,
            truth: truth_state,
            measurement: measurement.x,
            estimate: estimate.x,
            covariance: 0.0,
        });
    }

    println!("rms: {}", samples.rms());
    samples.save();
}
