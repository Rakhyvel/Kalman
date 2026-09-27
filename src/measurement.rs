use nalgebra::SMatrix;
pub trait MeasurementModel<const N: usize, const M: usize> {
    /// The C/H matrix for this kind of measurement
    fn measurement_matrix(&self) -> SMatrix<f64, M, N>;

    /// The R matrix for this kind of measurement
    fn noise_covariance(&self) -> SMatrix<f64, M, M>;
}

/// Represents measurements that are some linear transformation of the state, plus Gaussian noise
#[derive(Clone, Copy)]
pub struct LinearMeasurementModel<const N: usize, const M: usize> {
    pub c: SMatrix<f64, M, N>,
    pub r: SMatrix<f64, M, M>,
}

impl<const N: usize, const M: usize> MeasurementModel<N, M> for LinearMeasurementModel<N, M> {
    fn measurement_matrix(&self) -> SMatrix<f64, M, N> {
        self.c
    }

    fn noise_covariance(&self) -> SMatrix<f64, M, M> {
        self.r
    }
}
