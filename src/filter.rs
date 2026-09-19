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
