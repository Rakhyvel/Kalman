use kalman::dynamics::HarmonicOscillator;
use kalman::filter::Kalman;
use kalman::sensor::{GaussianSensor, Sensor};
use kalman::truth::{Oscillating, Truth};

use kalman::plot::{Sample, SampleLog};
use nalgebra::{SMatrix, SVector};

const POS_STD_DEV: f64 = 0.44;
const VEL_STD_DEV: f64 = 0.1;
const SEED: u64 = 67;

const DT: f64 = 0.1;

fn main() {
    let mut truth = Oscillating {
        amplitude: 1.0,
        angular_frequency: 1.0,
        phase: 0.0,
        t: 0.0,
    };

    let dynamics = HarmonicOscillator {
        angular_frequency: 1.1,
        q: 0.1,
    };

    let mut pos_sensor =
        GaussianSensor::new(SMatrix::<f64, 1, 2>::new(1.0, 0.0), POS_STD_DEV, SEED).unwrap();
    let pos_msr_model = pos_sensor.nominal_model();

    let mut vel_sensor =
        GaussianSensor::new(SMatrix::<f64, 1, 2>::new(0.0, 1.0), VEL_STD_DEV, SEED + 1).unwrap();
    let vel_msr_model = vel_sensor.nominal_model();

    let mut filter = Kalman::<2>::new(SVector::zeros(), SMatrix::identity() * 1000.0);

    let mut samples = SampleLog::new();

    const MAX_ITERS: usize = 1000;
    for i in 0..MAX_ITERS {
        let t = i as f64 * DT;

        truth.step(DT);
        let truth_state = truth.state()[0];

        filter.predict(&dynamics, DT);

        // Get position msrs every 10 measurements
        if i % 10 == 0 {
            let measurement = pos_sensor.measure(&truth.state());
            filter.correct(&pos_msr_model, measurement);
        } else {
            let measurement = vel_sensor.measure(&truth.state());
            filter.correct(&vel_msr_model, measurement);
        };

        let estimate: SVector<f64, 2> = filter.x();
        let covariance = filter.p();

        samples.record(Sample {
            t,
            truth: truth_state,
            measurement: 0.0,
            estimate: estimate.x,
            covariance: covariance[(0, 0)],
        });
    }

    println!("rms: {}", samples.rms());
    samples.save();
}
