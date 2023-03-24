pub trait Subsystem {
    fn periodic(&mut self);
    fn simulation_periodic(&mut self);
}