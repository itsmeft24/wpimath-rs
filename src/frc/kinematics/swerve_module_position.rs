use nalgebra::Vector2;

#[derive(Clone, Copy)]
pub struct SwerveModulePosition {
    pub distance: f64,
    pub angle: Vector2<f64>,
}

impl SwerveModulePosition {
    pub fn new(distance: f64, angle: &Vector2<f64>) -> Self {
        Self {
            distance: distance,
            angle: angle.clone(),
        }
    }
}
