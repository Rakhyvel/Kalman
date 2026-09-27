use nalgebra::{SMatrix, SVector};

use crate::{dynamics::Dynamics, measurement::MeasurementModel};

pub trait Filter<const STATE_DIM: usize, const MSR_DIM: usize> {
    /// Take in a new measurement and update the estimate
    fn update(&mut self, measurement: SVector<f64, MSR_DIM>) -> SVector<f64, STATE_DIM>;
}

#[derive(Debug)]
pub enum FilterError {
    InvalidGain,
    InvalidInitialEstimate,
}

pub struct MovingAverage {
    alpha: f64,
    estimate: f64,
}

impl MovingAverage {
    pub fn new(alpha: f64, initial_estimate: f64) -> Result<MovingAverage, FilterError> {
        if !alpha.is_finite() || !(0.0..=1.0).contains(&alpha) {
            return Err(FilterError::InvalidGain);
        }

        if !initial_estimate.is_finite() {
            return Err(FilterError::InvalidInitialEstimate);
        }

        Ok(MovingAverage {
            alpha,
            estimate: initial_estimate,
        })
    }
}

impl Filter<1, 1> for MovingAverage {
    fn update(&mut self, measurement: SVector<f64, 1>) -> SVector<f64, 1> {
        self.estimate = self.alpha * self.estimate + (1.0 - self.alpha) * measurement.x;
        SVector::<f64, 1>::new(self.estimate)
    }
}

pub struct RollingAverage<const N: usize> {
    estimates: [f64; N],
    count: usize,
    idx: usize,
    sum: f64,
}

impl<const N: usize> RollingAverage<N> {
    pub fn new() -> Self {
        Self {
            estimates: [0.0; N],
            count: 0,
            idx: 0,
            sum: 0.0,
        }
    }
}

impl<const N: usize> Filter<1, 1> for RollingAverage<N> {
    fn update(&mut self, measurement: SVector<f64, 1>) -> SVector<f64, 1> {
        self.sum -= self.estimates[self.idx];
        self.estimates[self.idx] = measurement.x;
        self.sum += measurement.x;
        self.idx = (self.idx + 1) % N;
        self.count = (self.count + 1).min(N);

        SVector::<f64, 1>::new(self.sum / self.count as f64)
    }
}

pub struct Kalman<const N: usize> {
    x: SVector<f64, N>,
    p: SMatrix<f64, N, N>,
}

impl<const N: usize> Kalman<N> {
    pub fn new(initial_estimate: SVector<f64, N>, initial_covariance: SMatrix<f64, N, N>) -> Self {
        Self {
            x: initial_estimate,
            p: initial_covariance,
        }
    }

    pub fn x(&self) -> SVector<f64, N> {
        self.x
    }

    pub fn p(&self) -> SMatrix<f64, N, N> {
        self.p
    }

    pub fn predict(&mut self, dynamics: &impl Dynamics<N>, dt: f64) {
        let (stm, q) = dynamics.discretize(dt);

        // Inflate covariance
        self.p = stm * self.p * stm.transpose() + q;

        // Propagate state
        self.x = stm * self.x;
    }

    pub fn correct<const M: usize>(
        &mut self,
        meas: &impl MeasurementModel<N, M>,
        z: SVector<f64, M>,
    ) {
        let c = meas.measurement_matrix();
        let r = meas.noise_covariance();

        // Update kalman gain
        let gain = self.p
            * c.transpose()
            * (c * self.p * c.transpose() + r)
                .try_inverse() // TODO: cholesky decomp (ch. 7)
                .unwrap();

        // Reduce covariance in dim of measurement
        self.p = (SMatrix::identity() - gain * c) * self.p; // TODO: replace with Joseph normal form (ch. 6)

        // Update estimate
        let innovation = z - c * self.x;
        self.x = self.x + gain * innovation;
    }
}
