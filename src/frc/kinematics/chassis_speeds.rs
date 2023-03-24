use nalgebra::Vector2;

#[derive(Clone, Copy)]
pub struct ChassisSpeeds {
    pub vx_mps: f64,
    pub vy_mps: f64,
    pub omega_rps: f64,
}

impl ChassisSpeeds {
    pub fn new(vx_mps: f64, vy_mps: f64, omega_rps: f64) -> Self {
        Self {
            vx_mps: vx_mps,
            vy_mps: vy_mps,
            omega_rps: omega_rps,
        }
    }
    pub fn from_field_relative_speeds(
        vx_mps: f64,
        vy_mps: f64,
        omega_rps: f64,
        robot_angle: &Vector2<f64>,
    ) -> Self {
        Self {
            vx_mps: vx_mps * robot_angle.x,
            vy_mps: vy_mps * robot_angle.y,
            omega_rps: omega_rps,
        }
    }
}
