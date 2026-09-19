pub trait State {
    fn step(&mut self, dt: f64);

    fn state(&self) -> f64;
}

pub struct ConstantVelocity {
    pub velocity: f64,
    pub position: f64,
}

impl State for ConstantVelocity {
    fn step(&mut self, dt: f64) {
        self.position += self.velocity * dt;
    }

    fn state(&self) -> f64 {
        self.position
    }
}

pub struct Oscillating {
    pub amplitude: f64,
    pub angular_frequency: f64,
    pub phase: f64,
    pub t: f64,
}

impl State for Oscillating {
    fn step(&mut self, dt: f64) {
        self.t += dt
    }

    fn state(&self) -> f64 {
        self.amplitude + (self.angular_frequency * self.t + self.phase).sin()
    }
}
