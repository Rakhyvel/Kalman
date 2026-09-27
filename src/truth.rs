///! Truth models used to simulate scenarios
use nalgebra::SVector;

pub trait Truth<const N: usize> {
    fn step(&mut self, dt: f64);

    fn state(&self) -> SVector<f64, N>;
}

/// A basic oscillator
pub struct Oscillating {
    pub amplitude: f64,
    pub angular_frequency: f64,
    pub phase: f64,
    pub t: f64,
}

impl Truth<2> for Oscillating {
    fn step(&mut self, dt: f64) {
        self.t += dt
    }

    fn state(&self) -> SVector<f64, 2> {
        let pos = self.amplitude * (self.angular_frequency * self.t + self.phase).sin();
        let vel = self.amplitude
            * self.angular_frequency
            * (self.angular_frequency * self.t + self.phase).cos();
        SVector::<f64, 2>::new(pos, vel)
    }
}
