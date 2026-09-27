///! Dynamics models that a filter could use
use nalgebra::{SMatrix, SVector};

use crate::truth::{Oscillating, Truth};

/// Dynamics model used by a filter's predict
pub trait Dynamics<const N: usize> {
    /// Returns the (STM, Q) for a step of dt
    fn discretize(&self, dt: f64) -> (SMatrix<f64, N, N>, SMatrix<f64, N, N>);
}

/// A simple constant velocity dynamics model
pub struct ConstantVelocity {
    pub velocity: f64,
    pub q: f64,
}

impl Dynamics<1> for ConstantVelocity {
    fn discretize(&self, dt: f64) -> (SMatrix<f64, 1, 1>, SMatrix<f64, 1, 1>) {
        let stm = SMatrix::<f64, 1, 1>::new(self.velocity * dt);
        let q = SMatrix::<f64, 1, 1>::new(self.q);

        (stm, q)
    }
}

/// A harmonic, oscillating dynamics model
pub struct HarmonicOscillator {
    pub angular_frequency: f64,
    pub q: f64,
}

impl Dynamics<2> for HarmonicOscillator {
    fn discretize(&self, dt: f64) -> (SMatrix<f64, 2, 2>, SMatrix<f64, 2, 2>) {
        let (s, c) = (self.angular_frequency * dt).sin_cos();

        let stm = SMatrix::<f64, 2, 2>::new(
            c,
            s / self.angular_frequency,
            -self.angular_frequency * s,
            c,
        );

        let gamma = SVector::<f64, 2>::new(dt.powi(2) / 2.0, dt);
        let q = gamma * SVector::<f64, 1>::new(self.q) * gamma.transpose();

        (stm, q)
    }
}

#[test]
fn oscillating_stm_small_omega() {
    let osc = HarmonicOscillator {
        angular_frequency: 1e-9, // small omega
        q: 1.0,
    };

    const DT: f64 = 0.5;

    let (stm, _q) = osc.discretize(DT);

    // A small angular velocity is basically just constant velocity
    const ABS_TOL: f64 = 1e-6;
    assert!((stm[(0, 0)] - 1.0).abs() < ABS_TOL);
    assert!((stm[(0, 1)] - DT).abs() < ABS_TOL);
    assert!((stm[(1, 0)] - 0.0).abs() < ABS_TOL);
    assert!((stm[(1, 1)] - 1.0).abs() < ABS_TOL);
}

#[test]
fn oscillating_stm_det() {
    let osc = HarmonicOscillator {
        angular_frequency: 0.6,
        q: 1.0,
    };

    const DT: f64 = 0.2;

    let (stm, _q) = osc.discretize(DT);

    // The oscillator conserves phase-space area (symplectic?!)
    assert_eq!(stm.determinant(), 1.0);
}

#[test]
fn oscillating_stm_chain() {
    let osc = HarmonicOscillator {
        angular_frequency: 0.6,
        q: 1.0,
    };

    const DT_1: f64 = 0.5;
    let (stm_1, _q) = osc.discretize(DT_1);

    const DT_2: f64 = 0.7;
    let (stm_2, _q) = osc.discretize(DT_2);

    let (stm_2_1, _q) = osc.discretize(DT_1 + DT_2);

    const ABS_TOL: f64 = 1e-6;
    assert!((stm_1 * stm_2 - stm_2_1).norm() < ABS_TOL);
}

#[test]
fn oscillating_stm_against_truth() {
    let mut truth = Oscillating {
        amplitude: 1.0,
        phase: 1.0,
        angular_frequency: 0.6,
        t: 0.0,
    };

    let osc = HarmonicOscillator {
        angular_frequency: 0.6,
        q: 1.0,
    };

    let x_1 = truth.state();

    const DT: f64 = 0.5;
    let (stm, _q) = osc.discretize(DT);

    truth.step(DT);

    let x_2 = truth.state();

    const ABS_TOL: f64 = 1e-6;
    assert!((stm * x_1 - x_2).norm() < ABS_TOL);
}

#[test]
fn oscillating_q_symmetric() {
    let osc = HarmonicOscillator {
        angular_frequency: 0.6,
        q: 1.0,
    };

    const DT: f64 = 0.5;
    let (_stm, q) = osc.discretize(DT);

    const ABS_TOL: f64 = 1e-6;
    assert!((q[(0, 1)] - q[(1, 0)]).abs() < ABS_TOL);
}

#[test]
fn oscillating_q_zero() {
    let osc = HarmonicOscillator {
        angular_frequency: 0.6,
        q: 0.0,
    };

    const DT: f64 = 0.5;
    let (_stm, q) = osc.discretize(DT);

    const ABS_TOL: f64 = 1e-6;
    assert!(q.norm() < ABS_TOL);
}
