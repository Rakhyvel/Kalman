use nalgebra::{SMatrix, SVector};

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

pub struct Kalman<const STATE_DIM: usize, const MSR_DIM: usize, const PROCESS_DIM: usize> {
    x: SVector<f64, STATE_DIM>,

    p: SMatrix<f64, STATE_DIM, STATE_DIM>,

    a: SMatrix<f64, STATE_DIM, STATE_DIM>,
    gamma: SMatrix<f64, STATE_DIM, PROCESS_DIM>,
    q: SMatrix<f64, PROCESS_DIM, PROCESS_DIM>,

    c: SMatrix<f64, MSR_DIM, STATE_DIM>,
    r: SMatrix<f64, MSR_DIM, MSR_DIM>,
}

impl<const STATE_DIM: usize, const MSR_DIM: usize, const PROCESS_DIM: usize>
    Kalman<STATE_DIM, MSR_DIM, PROCESS_DIM>
{
    pub fn new(
        initial_estimate: SVector<f64, STATE_DIM>,
        initial_covariance: SMatrix<f64, STATE_DIM, STATE_DIM>,
        a: SMatrix<f64, STATE_DIM, STATE_DIM>,
        gamma: SMatrix<f64, STATE_DIM, PROCESS_DIM>,
        q: SMatrix<f64, PROCESS_DIM, PROCESS_DIM>,
        c: SMatrix<f64, MSR_DIM, STATE_DIM>,
        r: SMatrix<f64, MSR_DIM, MSR_DIM>,
    ) -> Self {
        Self {
            x: initial_estimate,
            p: initial_covariance,
            a,
            gamma,
            q,
            c,
            r,
        }
    }

    pub fn p(&self) -> SMatrix<f64, STATE_DIM, STATE_DIM> {
        self.p
    }

    fn predict(&mut self) {
        // Inflate covariance
        self.p =
            self.a * self.p * self.a.transpose() + self.gamma * self.q * self.gamma.transpose();

        // Propagate state
        self.x = self.a * self.x;
    }

    fn correct(&mut self, z: SVector<f64, MSR_DIM>) {
        // Update kalman gain
        let gain = self.p
            * self.c.transpose()
            * (self.c * self.p * self.c.transpose() + self.r)
                .try_inverse() // TODO: cholesky decomp (ch. 7)
                .unwrap();

        // Reduce covariance in dim of measurement
        self.p = (SMatrix::identity() - gain * self.c) * self.p; // TODO: replace with Joseph normal form (ch. 6)

        // Update estimate
        let innovation = z - self.c * self.x;
        self.x = self.x + gain * innovation;
    }
}

impl<const STATE_DIM: usize, const MSR_DIM: usize, const PROCESS_DIM: usize>
    Filter<STATE_DIM, MSR_DIM> for Kalman<STATE_DIM, MSR_DIM, PROCESS_DIM>
{
    fn update(&mut self, z: SVector<f64, MSR_DIM>) -> SVector<f64, STATE_DIM> {
        self.predict();
        self.correct(z);

        self.x
    }
}
