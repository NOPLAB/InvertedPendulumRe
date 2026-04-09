//! FFI bindings to MATLAB Coder generated C controller functions.
//!
//! Generated function signatures (coder.Constant params are baked in):
//!   float lqr_step(const float state[4]);
//!   float pid_step(const float state[4]);
//!   float mpc_step(const float state[4]);
//!   float mrac_step(const float state[4]);
//!   void  observer_step(float position, float theta, float force, float x_hat[4]);
//!
//! Each function also has an _initialize() variant to reset persistent variables.

extern "C" {
    pub fn lqr_step_initialize();
    pub fn lqr_step(state: *const f32) -> f32;

    pub fn pid_step_initialize();
    pub fn pid_step(state: *const f32) -> f32;

    pub fn mpc_step_initialize();
    pub fn mpc_step(state: *const f32) -> f32;

    pub fn mrac_step_initialize();
    pub fn mrac_step(state: *const f32) -> f32;

    pub fn observer_step_initialize();
    pub fn observer_step(position: f32, theta: f32, force: f32, x_hat: *mut f32);
}
