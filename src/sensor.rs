use nalgebra::{SMatrix, SVector};
use rand::{SeedableRng, rngs::StdRng};
use rand_distr::{Distribution, Normal};

use crate::measurement::LinearMeasurementModel;

pub trait Sensor<const N: usize, const M: usize> {
    fn measure(&mut self, truth_state: &SVector<f64, N>) -> SVector<f64, M>;
}

#[derive(Debug)]
pub enum SensorError {
    BadStdDev,
}

pub struct GaussianSensor<const N: usize, const M: usize> {
    h: SMatrix<f64, M, N>,
    std_dev: f64,
    normal: Normal<f64>,
    rng: StdRng,
}

impl<const N: usize, const M: usize> GaussianSensor<N, M> {
    pub fn new(
        h: SMatrix<f64, M, N>,
        std_dev: f64,
        seed: u64,
    ) -> Result<GaussianSensor<N, M>, SensorError> {
        if !std_dev.is_finite() || std_dev < 0.0 {
            return Err(SensorError::BadStdDev);
        }

        let normal = Normal::new(0.0, std_dev).map_err(|_| SensorError::BadStdDev)?;

        Ok(GaussianSensor::<N, M> {
            h,
            std_dev,
            normal,
            rng: StdRng::seed_from_u64(seed),
        })
    }

    pub fn nominal_model(&self) -> LinearMeasurementModel<N, M> {
        LinearMeasurementModel::<N, M> {
            c: self.h,
            r: SMatrix::identity() * self.std_dev,
        }
    }
}

impl<const N: usize, const M: usize> Sensor<N, M> for GaussianSensor<N, M> {
    fn measure(&mut self, truth_state: &SVector<f64, N>) -> SVector<f64, M> {
        let mut retval = self.h * *truth_state;

        for i in 0..M {
            retval[i] += self.normal.sample(&mut self.rng);
        }

        retval
    }
}
