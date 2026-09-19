use rand::{SeedableRng, rngs::StdRng};
use rand_distr::{Distribution, Normal};

use crate::state::State;

pub trait Sensor<S: State> {
    fn measure(&mut self, current_state: &S) -> f64;
}

#[derive(Debug)]
pub enum SensorError {
    BadStdDev,
}

pub struct GaussianSensor {
    normal: Normal<f64>,
    rng: StdRng,
}

impl GaussianSensor {
    pub fn new(std_dev: f64, seed: u64) -> Result<GaussianSensor, SensorError> {
        if !std_dev.is_finite() || std_dev < 0.0 {
            return Err(SensorError::BadStdDev);
        }

        let normal = Normal::new(0.0, std_dev).map_err(|_| SensorError::BadStdDev)?;

        Ok(GaussianSensor {
            normal,
            rng: StdRng::seed_from_u64(seed),
        })
    }
}

impl<S: State> Sensor<S> for GaussianSensor {
    fn measure(&mut self, current_state: &S) -> f64 {
        let noise: f64 = self.normal.sample(&mut self.rng);
        current_state.state() + noise
    }
}
