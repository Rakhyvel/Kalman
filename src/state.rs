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
