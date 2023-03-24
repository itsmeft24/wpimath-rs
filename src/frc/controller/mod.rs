use std::ops::{Div, Mul, Sub};

pub struct SimpleMotorFeedforward {
    ks: f64,
    kv: f64,
    ka: f64,
}

impl SimpleMotorFeedforward {
    pub fn new(ks: f64, kv: f64, ka: f64) -> Self {
        Self {
            ks: ks,
            kv: kv,
            ka: ka,
        }
    }

    pub fn calculate(&self, velocity: f64, acceleration: f64) -> f64 {
        self.ks * velocity.signum() + self.kv * velocity + self.ka * acceleration
    }
}

fn input_modulus<T>(input: T, minimum_input: T, maximum_input: T) -> T
where
    T: Sub<Output = T> + Div<Output = T> + Mul<Output = T> + std::cmp::PartialOrd + Copy,
{
    let modulus = maximum_input - minimum_input;

    // Wrap input if it's above the maximum input
    let num_max = (input - minimum_input) / modulus;
    let input = input - num_max * modulus;

    // Wrap input if it's below the minimum input
    let num_min = (input - maximum_input) / modulus;
    let input = input - num_min * modulus;

    return input;
}

pub struct PIDController {
    kp: f64,
    ki: f64,
    kd: f64,
    period: f64,
    maximum_integral: f64,
    minimum_integral: f64,
    maximum_input: f64,
    minimum_input: f64,
    continuous: bool,
    position_error: f64,
    velocity_error: f64,
    prev_error: f64,
    total_error: f64,
    position_tolerance: f64,
    velocity_tolerance: f64,
    setpoint: f64,
    measurement: f64,
    have_measurement: bool,
    have_setpoint: bool,
}
impl PIDController {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self::new_with_period(kp, ki, kd, 0.02)
    }
    pub fn new_with_period(kp: f64, ki: f64, kd: f64, period: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            period,
            maximum_integral: 1.0,
            minimum_integral: -1.0,
            maximum_input: 0.0,
            minimum_input: 0.0,
            continuous: false,
            position_error: 0.0,
            velocity_error: 0.0,
            prev_error: 0.0,
            total_error: 0.0,
            position_tolerance: 0.05,
            velocity_tolerance: std::f64::INFINITY,
            setpoint: 0.0,
            measurement: 0.0,
            have_measurement: false,
            have_setpoint: false,
        }
    }
    pub fn set_pid(&mut self, kp: f64, ki: f64, kd: f64) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }
    pub fn set_p(&mut self, kp: f64) {
        self.kp = kp;
    }
    pub fn set_i(&mut self, ki: f64) {
        self.ki = ki;
    }
    pub fn set_d(&mut self, kd: f64) {
        self.kd = kd;
    }
    pub fn get_p(&self) -> f64 {
        return self.kp;
    }
    pub fn get_i(&self) -> f64 {
        return self.ki;
    }
    pub fn get_d(&self) -> f64 {
        return self.kd;
    }
    pub fn get_period(&self) -> f64 {
        return self.period;
    }
    pub fn get_position_tolerance(&self) -> f64 {
        return self.position_tolerance;
    }
    pub fn get_velocity_tolerance(&self) -> f64 {
        return self.velocity_tolerance;
    }
    pub fn set_setpoint(&mut self, setpoint: f64) {
        self.setpoint = setpoint;
        self.have_setpoint = true;

        if self.continuous {
            let error_bound = (self.maximum_input - self.minimum_input) / 2.0;
            self.position_error =
                input_modulus(self.setpoint - self.measurement, -error_bound, error_bound);
        } else {
            self.position_error = self.setpoint - self.measurement;
        }

        self.velocity_error = (self.position_error - self.prev_error) / self.period;
    }
    pub fn get_setpoint(&self) -> f64 {
        return self.setpoint;
    }
    pub fn at_setpoint(&self) -> bool {
        return self.have_measurement
            && self.have_setpoint
            && self.position_error.abs() < self.position_tolerance
            && self.velocity_error.abs() < self.velocity_tolerance;
    }

    pub fn enable_continuous_input(&mut self, minimum_input: f64, maximum_input: f64) {
        self.continuous = true;
        self.minimum_input = minimum_input;
        self.maximum_input = maximum_input;
    }

    pub fn disable_continuous_input(&mut self) {
        self.continuous = false;
    }

    pub fn is_continuous_input_enabled(&self) -> bool {
        return self.continuous;
    }

    pub fn set_integrator_range(&mut self, minimum_integral: f64, maximum_integral: f64) {
        self.minimum_integral = minimum_integral;
        self.maximum_integral = maximum_integral;
    }

    pub fn set_tolerance(&mut self, position_tolerance: f64) {
        self.set_tolerances(position_tolerance, std::f64::INFINITY);
    }

    pub fn set_tolerances(&mut self, position_tolerance: f64, velocity_tolerance: f64) {
        self.position_tolerance = position_tolerance;
        self.velocity_tolerance = velocity_tolerance;
    }
    pub fn get_position_error(&self) -> f64 {
        return self.position_error;
    }

    pub fn get_velocity_error(&self) -> f64 {
        return self.velocity_error;
    }

    pub fn calculate_with_new_setpoint(&mut self, measurement: f64, setpoint: f64) -> f64 {
        self.setpoint = setpoint;
        self.have_setpoint = true;
        return self.calculate(measurement);
    }

    pub fn calculate(&mut self, measurement: f64) -> f64 {
        self.measurement = measurement;
        self.prev_error = self.position_error;
        self.have_measurement = true;

        if self.continuous {
            let error_bound = (self.maximum_input - self.minimum_input) / 2.0;
            self.position_error =
                input_modulus(self.setpoint - self.measurement, -error_bound, error_bound);
        } else {
            self.position_error = self.setpoint - self.measurement;
        }

        self.velocity_error = (self.position_error - self.prev_error) / self.period;

        if self.ki != 0.0 {
            self.total_error = (self.total_error + self.position_error * self.period).clamp(
                self.minimum_integral / self.ki,
                self.maximum_integral / self.ki,
            );
        }

        return self.kp * self.position_error
            + self.ki * self.total_error
            + self.kd * self.velocity_error;
    }
    pub fn reset(&mut self) {
        self.position_error = 0.0;
        self.prev_error = 0.0;
        self.total_error = 0.0;
        self.velocity_error = 0.0;
        self.have_measurement = false;
    }
}
