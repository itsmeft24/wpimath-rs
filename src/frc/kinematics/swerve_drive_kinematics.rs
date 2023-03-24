use nalgebra::{Dynamic, Vector2, Vector3};

use super::{ChassisSpeeds, SwerveModuleState};

pub struct SwerveDriveKinematics {
    inverse_kinematics: nalgebra::DMatrix<f64>,
    forward_kinematics: nalgebra::SVD<f64, Dynamic, Dynamic>,
    modules: [Vector2<f64>; 4],
    module_states: [SwerveModuleState; 4],
    previous_center_of_rotation: Vector2<f64>,
}

impl SwerveDriveKinematics {
    pub fn new(wheels: [Vector2<f64>; 4]) -> Self {
        let mut inverse_kinematics = nalgebra::DMatrix::zeros(8, 3);
        for i in 0..4 {
            inverse_kinematics
                .fixed_slice_mut::<2, 3>(i * 2, 0)
                .copy_from(&nalgebra::Matrix2x3::new(
                    1.0,
                    0.0,
                    (-wheels[i].y).into(),
                    0.0,
                    1.0,
                    (wheels[i].x).into(),
                ));
        }
        let forward_kinematics = inverse_kinematics.clone().svd(true, true);
        Self {
            inverse_kinematics: inverse_kinematics,
            forward_kinematics: forward_kinematics,
            modules: wheels,
            module_states: [
                SwerveModuleState::default(),
                SwerveModuleState::default(),
                SwerveModuleState::default(),
                SwerveModuleState::default(),
            ],
            previous_center_of_rotation: Vector2::zeros(),
        }
    }

    pub fn to_swerve_module_states(
        &mut self,
        chassis_speeds: &ChassisSpeeds,
        center_of_rotation: &Vector2<f64>,
    ) -> [SwerveModuleState; 4] {
        if chassis_speeds.vx_mps == 0.0
            && chassis_speeds.vy_mps == 0.0
            && chassis_speeds.omega_rps == 0.0
        {
            for i in 0..4 {
                self.module_states[i].speed = 0.0;
            }
            return self.module_states.clone();
        }
        if !self.previous_center_of_rotation.eq(center_of_rotation) {
            for i in 0..4 {
                self.inverse_kinematics
                    .fixed_slice_mut::<2, 3>(i * 2, 0)
                    .copy_from(&nalgebra::Matrix2x3::new(
                        1.0,
                        0.0,
                        (-self.modules[i].y + center_of_rotation.y).into(),
                        0.0,
                        1.0,
                        (self.modules[i].x - center_of_rotation.x).into(),
                    ));
            }
            self.previous_center_of_rotation = *center_of_rotation;
        }
        let chassis_speeds_vector = Vector3::new(
            chassis_speeds.vx_mps,
            chassis_speeds.vy_mps,
            chassis_speeds.omega_rps,
        );
        let module_state_matrix = self.inverse_kinematics.clone() * chassis_speeds_vector;
        for i in 0..4 {
            let x = *module_state_matrix.get((i * 2, 0)).unwrap_or(&0.0);
            let y = *module_state_matrix.get((i * 2 + 1, 0)).unwrap_or(&0.0);
            let speed = x.hypot(y);
            let rotation = Vector2::new(x, y).normalize();
            self.module_states[i] = SwerveModuleState::new(speed, &rotation);
        }
        return self.module_states.clone();
    }
    pub fn desaturate_wheel_speeds(
        module_states: &mut [SwerveModuleState; 4],
        attainable_max_speed: f64,
    ) {
        let max_speed = module_states
            .iter()
            .max_by(|a, b| a.speed.abs().partial_cmp(&b.speed.abs()).unwrap())
            .unwrap()
            .speed;
        if max_speed > attainable_max_speed {
            for state in module_states.iter_mut() {
                state.speed = state.speed / max_speed * attainable_max_speed;
            }
        }
    }
    pub fn to_chassis_speeds(&self, module_states: &[SwerveModuleState; 4]) -> ChassisSpeeds {
        let mut module_state_matrix = nalgebra::DMatrix::<f64>::zeros(8, 1);
        for i in 0..4 {
            *module_state_matrix.get_mut((i * 2, 0)).unwrap() =
                module_states[i].speed * module_states[i].angle.x;
            *module_state_matrix.get_mut((i * 2 + 1, 0)).unwrap() =
                module_states[i].speed * module_states[i].angle.y;
        }
        let chassis_speeds_vector = self
            .forward_kinematics
            .solve(&module_state_matrix, 0.0)
            .unwrap();
        ChassisSpeeds::new(
            *chassis_speeds_vector.get(0).unwrap(),
            *chassis_speeds_vector.get(1).unwrap(),
            *chassis_speeds_vector.get(2).unwrap(),
        )
    }
}
