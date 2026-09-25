pub trait Filter {
    /// Take in a new measurement and update the estimate
    fn update(&mut self, measurement: f64) -> f64;
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

impl Filter for MovingAverage {
    fn update(&mut self, measurement: f64) -> f64 {
        self.estimate = self.alpha * self.estimate + (1.0 - self.alpha) * measurement;
        self.estimate
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

impl<const N: usize> Filter for RollingAverage<N> {
    fn update(&mut self, measurement: f64) -> f64 {
        self.sum -= self.estimates[self.idx];
        self.estimates[self.idx] = measurement;
        self.sum += measurement;
        self.idx = (self.idx + 1) % N;
        self.count = (self.count + 1).min(N);

        self.sum / self.count as f64
    }
}

pub struct Kalman {
    x: f64,

    p: f64,
    g: f64,

    a: f64,
    gamma: f64,
    q: f64,

    c: f64,
    r: f64,
}

impl Kalman {
    pub fn new(
        initial_estimate: f64,
        initial_covariance: f64,
        a: f64,
        gamma: f64,
        q: f64,
        c: f64,
        r: f64,
    ) -> Self {
        Self {
            x: initial_estimate,
            p: initial_covariance,
            g: 1.0,
            a,
            gamma,
            q,
            c,
            r,
        }
    }

    pub fn p(&self) -> f64 {
        self.p
    }

    fn predict(&mut self) {
        // Inflate covariance
        self.p = self.a * self.p * self.a + self.gamma * self.q * self.gamma;

        // Update kalman gain
        self.g = self.p * self.c * (1.0 / (self.c * self.p * self.c + self.r));
    }

    fn correct(&mut self, measurement: f64) {
        // Reduce covariance in dim of measurement
        self.p = (1.0 - self.g * self.c) * self.p;

        // Propagate state
        self.x = self.a * self.x;

        // Update estimate
        let innovation = measurement - self.c * self.x;
        self.x = self.x + self.g * innovation;
    }
}

impl Filter for Kalman {
    fn update(&mut self, measurement: f64) -> f64 {
        self.predict();
        self.correct(measurement);

        self.x
    }
}
