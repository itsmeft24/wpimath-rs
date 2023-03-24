use nalgebra::Vector2;

#[derive(Clone, Copy)]
pub struct SwerveModuleState {
    pub speed: f64,
    pub angle: Vector2<f64>,
}

impl SwerveModuleState {
    pub fn default() -> Self {
        Self {
            speed: 0.0_f64,
            angle: Vector2::zeros(),
        }
    }
    pub fn new(speed: f64, angle: &Vector2<f64>) -> Self {
        Self {
            speed: speed,
            angle: angle.clone(),
        }
    }
}
