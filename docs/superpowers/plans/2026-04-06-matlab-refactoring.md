# MATLAB Refactoring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor MATLAB simulation code — delete dead code, split FirmwareExperiment.m into focused modules, consolidate Rust export helpers, and rename mode4-specific files.

**Architecture:** FirmwareExperiment.m (742 lines) is split into a simulation core + 4 controller step files + utility/options modules. Dead files are deleted. Rust export helpers are consolidated into a shared utility. All changes are behavior-preserving.

**Tech Stack:** MATLAB (functions, classdef)

**Spec:** `docs/superpowers/specs/2026-04-06-matlab-refactoring-design.md`

---

### Task 1: Capture baseline output for regression verification

Before any changes, run main.m and save numerical results to verify nothing breaks.

**Files:**
- Create: `MATLAB/test_baseline.m`

- [ ] **Step 1: Create baseline test script**

```matlab
%% test_baseline.m - Capture/verify simulation output for regression testing
clear; close all; clc;

rootDir = fileparts(which(mfilename));
addpath(fullfile(rootDir, 'plant'));
addpath(fullfile(rootDir, 'control'));
addpath(fullfile(rootDir, 'sim'));

p = params();
x0 = [0; 0; 0.1; 0];
t_end = 15;
disturbance = struct('time', 10.0, 'duration', 0.05, 'force', 5.0);
sim_options = struct('disturbance', disturbance);

modes = {'pid', 'lqr', 'mrac', 'mpc'};
baseline_file = fullfile(rootDir, 'test_baseline.mat');

if ~isfile(baseline_file)
    % CAPTURE MODE: save baseline
    results = cell(size(modes));
    for i = 1:numel(modes)
        results{i} = simulate_firmware(p, x0, t_end, modes{i}, [], sim_options);
    end
    save(baseline_file, 'results');
    fprintf('Baseline saved to %s\n', baseline_file);
else
    % VERIFY MODE: compare against baseline
    loaded = load(baseline_file, 'results');
    baseline = loaded.results;
    all_pass = true;
    for i = 1:numel(modes)
        result = simulate_firmware(p, x0, t_end, modes{i}, [], sim_options);
        max_state_err = max(abs(result.x(:) - baseline{i}.x(:)));
        max_force_err = max(abs(result.force(:) - baseline{i}.force(:)));
        if max_state_err < 1e-10 && max_force_err < 1e-10
            fprintf('[PASS] %s: state_err=%.2e, force_err=%.2e\n', modes{i}, max_state_err, max_force_err);
        else
            fprintf('[FAIL] %s: state_err=%.2e, force_err=%.2e\n', modes{i}, max_state_err, max_force_err);
            all_pass = false;
        end
    end
    if all_pass
        fprintf('\nAll modes match baseline.\n');
    else
        fprintf('\nREGRESSION DETECTED!\n');
    end
end
```

- [ ] **Step 2: Run baseline capture in MATLAB**

Run: `test_baseline` in MATLAB (from `MATLAB/` directory)
Expected: "Baseline saved to .../test_baseline.mat"

- [ ] **Step 3: Verify the baseline can be reproduced**

Run: `test_baseline` again in MATLAB
Expected: All 4 modes show `[PASS]` with errors at 0.00e+00

- [ ] **Step 4: Commit**

```bash
git add MATLAB/test_baseline.m
git commit -m "test: add baseline regression test for MATLAB refactoring"
```

---

### Task 2: Delete dead code files

Remove unused files: `simulate_nonlinear.m`, `animate_pendulum.m`, `compare_controllers.m`.

**Files:**
- Delete: `MATLAB/sim/simulate_nonlinear.m`
- Delete: `MATLAB/sim/animate_pendulum.m`
- Delete: `MATLAB/sim/compare_controllers.m`

- [ ] **Step 1: Delete the three unused files**

```bash
git rm MATLAB/sim/simulate_nonlinear.m
git rm MATLAB/sim/animate_pendulum.m
git rm MATLAB/sim/compare_controllers.m
```

- [ ] **Step 2: Run regression test**

Run: `test_baseline` in MATLAB
Expected: All 4 modes `[PASS]`

- [ ] **Step 3: Commit**

```bash
git commit -m "refactor(matlab): remove unused simulation files

Remove simulate_nonlinear.m, animate_pendulum.m, and compare_controllers.m.
These are fully replaced by FirmwareExperiment and main.m comparison logic."
```

---

### Task 3: Rename linearize_mode4_voltage_system.m to linearize_voltage_input.m

**Files:**
- Rename: `MATLAB/plant/linearize_mode4_voltage_system.m` → `MATLAB/plant/linearize_voltage_input.m`
- Modify: `MATLAB/control/design_mrac.m:10`

- [ ] **Step 1: Rename the file and update the function name**

Rename the file:
```bash
git mv MATLAB/plant/linearize_mode4_voltage_system.m MATLAB/plant/linearize_voltage_input.m
```

Edit `MATLAB/plant/linearize_voltage_input.m` — change the function declaration and comment:

```matlab
function [A, B, C, D] = linearize_voltage_input(p)
% LINEARIZE_VOLTAGE_INPUT Linearized plant for direct-voltage control.
%   State: x = [position; velocity; theta; theta_dot; current]
%   Input: u = motor voltage [V]
%
%   The mechanical subsystem is the existing force-input linearization.
%   Force is generated from armature current:
%       F = 2 * G * Kt / r * i
%   Armature current dynamics are:
%       i_dot = (v - Ke * G / r * x_dot - Ra * i) / La
```

- [ ] **Step 2: Update the call site in design_mrac.m**

In `MATLAB/control/design_mrac.m` line 10, change:
```matlab
    [A, B, ~, ~] = linearize_mode4_voltage_system(p);
```
to:
```matlab
    [A, B, ~, ~] = linearize_voltage_input(p);
```

- [ ] **Step 3: Search for any other references to the old name**

```bash
grep -r "linearize_mode4" MATLAB/
```
Expected: No matches (FirmwareExperiment.m does not call this directly — it calls `design_mrac` which calls it)

- [ ] **Step 4: Run regression test**

Run: `test_baseline` in MATLAB
Expected: All 4 modes `[PASS]`

- [ ] **Step 5: Commit**

```bash
git add MATLAB/plant/linearize_voltage_input.m MATLAB/control/design_mrac.m
git commit -m "refactor(matlab): rename linearize_mode4_voltage_system to linearize_voltage_input

The function linearizes the 5-state voltage-input plant model. The name
'mode4' was tied to a controller mode number; 'voltage_input' describes
what the function actually does."
```

---

### Task 4: Extract sim_utils.m (utility functions from FirmwareExperiment)

Extract static utility functions that are general-purpose and not specific to the simulation loop.

**Files:**
- Create: `MATLAB/sim/sim_utils.m`
- Modify: `MATLAB/sim/FirmwareExperiment.m`

- [ ] **Step 1: Create sim_utils.m**

```matlab
function util = sim_utils()
% SIM_UTILS Shared utility functions for the firmware simulator.
%   Usage: util = sim_utils();
%          lpf = util.init_lpf(sample_time, cutoff_freq);
%          lpf = util.lpf_update(lpf, input);
%          out = util.merge_structs(base, override);
%          spec = util.normalize_mode(mode);
%          y = util.clamp(x, lo, hi);
%          alpha = util.lpf_alpha(cutoff_freq, sample_time);

    util.init_lpf = @init_lpf;
    util.lpf_update = @lpf_update;
    util.merge_structs = @merge_structs;
    util.normalize_mode = @normalize_mode;
    util.clamp = @clamp;
    util.lpf_alpha = @lpf_alpha;
end

function lpf = init_lpf(sample_time, cutoff_freq)
    lpf.alpha = lpf_alpha(cutoff_freq, sample_time);
    lpf.output = 0.0;
    lpf.initialized = false;
end

function lpf = lpf_update(lpf, input)
    if ~lpf.initialized
        lpf.output = input;
        lpf.initialized = true;
    else
        lpf.output = lpf.alpha * input + (1 - lpf.alpha) * lpf.output;
    end
end

function out = merge_structs(base, override)
    out = base;
    if isempty(override)
        return;
    end
    fields = fieldnames(override);
    for idx = 1:numel(fields)
        name = fields{idx};
        if isfield(out, name) && isstruct(out.(name)) && isstruct(override.(name))
            out.(name) = merge_structs(out.(name), override.(name));
        else
            out.(name) = override.(name);
        end
    end
end

function spec = normalize_mode(mode)
    if isstring(mode) || ischar(mode)
        key = lower(strtrim(char(mode)));
        switch key
            case {'mode1', 'debug'}
                spec = struct('mode_number', 1, 'firmware_mode', 0, 'name', 'debug');
            case {'mode2', 'pid'}
                spec = struct('mode_number', 2, 'firmware_mode', 1, 'name', 'pid');
            case {'mode3', 'lqr'}
                spec = struct('mode_number', 3, 'firmware_mode', 2, 'name', 'lqr');
            case {'mode4', 'mrac'}
                spec = struct('mode_number', 4, 'firmware_mode', 3, 'name', 'mrac');
            case {'mode5', 'mpc'}
                spec = struct('mode_number', 5, 'firmware_mode', 4, 'name', 'mpc');
            otherwise
                error('sim_utils:InvalidMode', 'Unsupported mode: %s', char(mode));
        end
        return;
    end

    if isnumeric(mode) && isscalar(mode)
        switch double(mode)
            case 1
                spec = struct('mode_number', 1, 'firmware_mode', 0, 'name', 'debug');
            case 2
                spec = struct('mode_number', 2, 'firmware_mode', 1, 'name', 'pid');
            case 3
                spec = struct('mode_number', 3, 'firmware_mode', 2, 'name', 'lqr');
            case 4
                spec = struct('mode_number', 4, 'firmware_mode', 3, 'name', 'mrac');
            case 5
                spec = struct('mode_number', 5, 'firmware_mode', 4, 'name', 'mpc');
            otherwise
                error('sim_utils:InvalidMode', ...
                    'Numeric mode must be one of 1, 2, 3, 4, 5.');
        end
        return;
    end

    error('sim_utils:InvalidMode', 'mode must be a string or scalar number.');
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function alpha = lpf_alpha(cutoff_freq, sample_time)
    tau = 1 / (2 * pi * cutoff_freq);
    alpha = sample_time / (tau + sample_time);
end
```

- [ ] **Step 2: Update FirmwareExperiment.m to use sim_utils**

Replace all internal calls to `FirmwareExperiment.init_lpf(...)` with `util.init_lpf(...)`, `FirmwareExperiment.lpf_update(...)` with `util.lpf_update(...)`, `FirmwareExperiment.merge_structs(...)` with `util.merge_structs(...)`, `FirmwareExperiment.normalize_mode(...)` with `util.normalize_mode(...)`, `FirmwareExperiment.clamp(...)` with `util.clamp(...)`, and `FirmwareExperiment.lpf_alpha(...)` with `util.lpf_alpha(...)`.

Add `util = sim_utils();` as a persistent variable or call at the top of methods that need it.

In the constructor, change:
```matlab
spec = FirmwareExperiment.normalize_mode(mode);
```
to:
```matlab
util = sim_utils();
spec = util.normalize_mode(mode);
```

In `default_options`, change:
```matlab
opts = FirmwareExperiment.merge_structs(opts, options);
```
to:
```matlab
util = sim_utils();
opts = util.merge_structs(opts, options);
```

In `init_runtime`, change all `FirmwareExperiment.init_lpf(...)` to `util.init_lpf(...)` (add `util = sim_utils();` at top of method).

In `step_balance_loop`, change `FirmwareExperiment.lpf_update(...)` to `util.lpf_update(...)`.

In `step_current_loop`, change `FirmwareExperiment.lpf_update(...)` to `util.lpf_update(...)`.

In `mrac_step`, change `FirmwareExperiment.clamp(...)` to `util.clamp(...)`.

In all other static methods that use `clamp`, change similarly.

Remove the following static methods from FirmwareExperiment.m (they now live in sim_utils.m):
- `normalize_mode`
- `merge_structs`
- `init_lpf`
- `lpf_update`
- `clamp`
- `lpf_alpha`

- [ ] **Step 3: Run regression test**

Run: `test_baseline` in MATLAB
Expected: All 4 modes `[PASS]`

- [ ] **Step 4: Commit**

```bash
git add MATLAB/sim/sim_utils.m MATLAB/sim/FirmwareExperiment.m
git commit -m "refactor(matlab): extract utility functions to sim_utils.m

Move init_lpf, lpf_update, merge_structs, normalize_mode, clamp, and
lpf_alpha from FirmwareExperiment static methods to sim_utils module."
```

---

### Task 5: Extract sim_options.m (default options and controller params)

Extract `default_options` and `default_controller_param` from FirmwareExperiment. Replace hardcoded observer matrices with dynamic computation.

**Files:**
- Create: `MATLAB/sim/sim_options.m`
- Modify: `MATLAB/sim/FirmwareExperiment.m`

- [ ] **Step 1: Create sim_options.m**

```matlab
function opt = sim_options()
% SIM_OPTIONS Default options and controller parameter factories.
%   Usage: opt = sim_options();
%          opts = opt.default_options(overrides);
%          ctrl = opt.default_controller_param(p, spec);

    opt.default_options = @default_options;
    opt.default_controller_param = @default_controller_param;
end

function opts = default_options(options)
    util = sim_utils();

    % オブザーバ行列を動的に計算
    p = params();
    [A_lin, B_lin, C_lin, ~] = linearize_system(p);
    obs = design_observer(A_lin, B_lin, C_lin);

    opts = struct( ...
        'vin', 12.0, ...
        'current_loop_frequency', 5000.0, ...
        'balance_loop_frequency', 1000.0, ...
        'enable_encoder_quantization', true, ...
        'enable_theta_quantization', true, ...
        'enable_motor_electrical_dynamics', true, ...
        'enable_force_saturation', false, ...
        'force_limit', 10.0, ...
        'max_force', 10.0, ...
        'max_voltage', 12.0, ...
        'theta_filter_cutoff', 50.0, ...
        'theta_dot_filter_cutoff', 25.0, ...
        'current_filter_cutoff', 500.0, ...
        'current_pid', struct( ...
            'Kp', 0.928, ...
            'Ki', 10178.8, ...
            'Kd', 0.0), ...
        'observer', struct( ...
            'Ad', obs.Ad, ...
            'Bd', obs.Bd, ...
            'Ld', obs.Ld), ...
        'disturbance', []);

    opts = util.merge_structs(opts, options);
end

function ctrl_param = default_controller_param(p, spec)
    switch spec.name
        case 'debug'
            ctrl_param = [];
        case 'pid'
            ctrl_param = design_pid();
        case 'lqr'
            [A, B, ~, ~] = linearize_system(p);
            ctrl_param = design_lqr(A, B);
        case 'mrac'
            ctrl_param = design_mrac(p);
        case 'mpc'
            [A, B, ~, ~] = linearize_system(p);
            ctrl_param = design_mpc(A, B);
        otherwise
            error('sim_options:InvalidMode', 'Unsupported mode: %s', spec.name);
    end
end
```

- [ ] **Step 2: Update FirmwareExperiment.m to use sim_options**

In the constructor, change:
```matlab
obj.options = FirmwareExperiment.default_options(options);
```
to:
```matlab
opt = sim_options();
obj.options = opt.default_options(options);
```

And change:
```matlab
obj.ctrl_param = FirmwareExperiment.default_controller_param(p, spec);
```
to:
```matlab
obj.ctrl_param = opt.default_controller_param(p, spec);
```

Remove the following static methods from FirmwareExperiment.m:
- `default_options`
- `default_controller_param`

- [ ] **Step 3: Run regression test**

Run: `test_baseline` in MATLAB
Expected: All 4 modes `[PASS]`

Note: The observer matrices were previously hardcoded. Now they are computed dynamically using `design_observer`. The numerical values should be identical because `design_observer` with default parameters produces the same matrices. If there is any floating-point difference, it should be well below the 1e-10 threshold.

- [ ] **Step 4: Commit**

```bash
git add MATLAB/sim/sim_options.m MATLAB/sim/FirmwareExperiment.m
git commit -m "refactor(matlab): extract sim_options with dynamic observer computation

Move default_options and default_controller_param from FirmwareExperiment
to sim_options module. Observer matrices are now computed via
design_observer() instead of being hardcoded."
```

---

### Task 6: Extract controller step functions (step_pid, step_lqr, step_mrac, step_mpc)

Extract the mode-specific control logic from `step_balance_loop` into individual files under `control/`.

**Files:**
- Create: `MATLAB/control/step_pid.m`
- Create: `MATLAB/control/step_lqr.m`
- Create: `MATLAB/control/step_mrac.m`
- Create: `MATLAB/control/step_mpc.m`
- Modify: `MATLAB/sim/FirmwareExperiment.m`

- [ ] **Step 1: Create step_pid.m**

```matlab
function [force, balance_pid] = step_pid(balance_pid, state, max_force)
% STEP_PID Cascaded PID balance controller step.
%   state: struct with fields position, velocity, theta, theta_dot
%   Returns force [N] and updated PID state.

    util = sim_utils();
    [u_theta, balance_pid.angle] = pid_update( ...
        balance_pid.angle, state.theta, 0.0);
    [u_x, balance_pid.position] = pid_update( ...
        balance_pid.position, state.position, 0.0);
    force = util.clamp(u_theta + u_x, -max_force, max_force);
end

function [output, pid] = pid_update(pid, setpoint, measurement)
    util = sim_utils();
    error_value = setpoint - measurement;
    p_term = pid.kp * error_value;
    pid.integral = pid.integral + pid.ki * error_value * pid.dt;

    if pid.first_call
        d_term = 0.0;
        pid.first_call = false;
    else
        d_term = pid.kd * (error_value - pid.prev_error) / pid.dt;
    end

    unclamped = p_term + pid.integral + d_term;
    output = util.clamp(unclamped, pid.min_output, pid.max_output);

    if unclamped > pid.max_output && pid.integral > 0.0
        pid.integral = max(pid.max_output - p_term - d_term, 0.0);
    elseif unclamped < pid.min_output && pid.integral < 0.0
        pid.integral = min(pid.min_output - p_term - d_term, 0.0);
    end

    pid.prev_error = error_value;
end
```

- [ ] **Step 2: Create step_lqr.m**

```matlab
function [force, target_current] = step_lqr(ctrl_param, x_ctrl, max_force, p)
% STEP_LQR LQR state feedback controller step.
%   ctrl_param: K gain vector (1x4)
%   x_ctrl: state vector [position; velocity; theta; theta_dot]
%   Returns force [N] and target current [A].

    util = sim_utils();
    gain = reshape(ctrl_param, 1, []);
    force = -(gain * x_ctrl);
    force = util.clamp(force, -max_force, max_force);
    target_current = force * (p.r / (2.0 * p.G * p.Kt));
end
```

- [ ] **Step 3: Create step_mrac.m**

```matlab
function [voltage, controller, current_used, current_state] = step_mrac(controller, state, measured_current, p, dt)
% STEP_MRAC Model Reference Adaptive Control step.
%   state: struct with fields position, velocity, theta, theta_dot
%   measured_current: signed motor current measurement
%   Returns voltage command and updated controller state.

    util = sim_utils();
    c = controller.ctrl;

    motor_speed = p.G * state.velocity / p.r;
    controller.current_state = step_current_estimate( ...
        controller.current_state, controller.prev_voltage, motor_speed, p, dt);

    if isfield(c, 'UseCurrentEstimate') && c.UseCurrentEstimate
        current = controller.current_state;
    else
        current = measured_current;
    end

    x = [ ...
        state.position; ...
        state.velocity; ...
        soft_zone(state.theta, c.ThetaSoftZone, c.InnerAngleGain); ...
        soft_zone(state.theta_dot, c.ThetaDotSoftZone, c.InnerAngularVelocityGain); ...
        soft_zone(current, c.CurrentSoftZone, c.InnerCurrentGain)];

    if ~controller.initialized
        controller.reference_state = x;
        controller.initialized = true;
        controller.startup_timer = 0.0;
    else
        controller.reference_state = controller.reference_state + dt * (c.Am * controller.reference_state);
        controller.startup_timer = controller.startup_timer + dt;
    end

    error_value = x - controller.reference_state;
    s = error_value' * c.PB;
    x_norm = x' * x;

    adaptation_enabled = abs(state.theta) < c.EnableAngleLimit ...
        && abs(state.theta_dot) < c.EnableAngularVelocityLimit ...
        && abs(current) < c.EnableCurrentLimit ...
        && controller.startup_timer >= c.AdaptationDelaySec ...
        && abs(s) > c.ErrorDeadzone;

    if adaptation_enabled
        scale = s / (c.NormalizationEps + x_norm);
        for idx = 1:5
            update = c.GammaDiag(idx, idx) * x(idx) * scale ...
                - c.Sigma * controller.adaptive_gains(idx);
            controller.adaptive_gains(idx) = controller.adaptive_gains(idx) + dt * update;
            controller.adaptive_gains(idx) = util.clamp( ...
                controller.adaptive_gains(idx), -c.MaxAdaptiveGain, c.MaxAdaptiveGain);
        end
    else
        controller.adaptive_gains = controller.adaptive_gains + dt * (-c.Sigma * controller.adaptive_gains);
    end

    raw_voltage = -c.Kx * x - controller.adaptive_gains' * x;
    max_delta = c.MaxVoltageSlewRate * dt;
    slewed_voltage = controller.prev_voltage + util.clamp( ...
        raw_voltage - controller.prev_voltage, -max_delta, max_delta);
    compensated_voltage = apply_low_speed_drive_compensation( ...
        slewed_voltage, state.theta, state.theta_dot, state.velocity, c);
    voltage = util.clamp(compensated_voltage, -c.MaxVoltage, c.MaxVoltage);
    controller.prev_voltage = voltage;
    current_used = current;
    current_state = controller.current_state;
end

function y = soft_zone(x, zone, inner_gain)
    abs_x = abs(x);
    if abs_x <= zone
        y = inner_gain * x;
    else
        y = sign(x) * (inner_gain * zone + (abs_x - zone));
    end
end

function y = apply_low_speed_drive_compensation(voltage, theta, theta_dot, velocity, c)
    abs_voltage = abs(voltage);
    if abs_voltage < c.MinActiveVoltage
        y = 0.0;
    elseif abs(theta) < c.StictionEnableAngleLimit ...
            && abs(theta_dot) < c.StictionEnableAngularVelocityLimit ...
            && abs(velocity) < c.LowSpeedVelocityThreshold ...
            && abs_voltage < c.MinDriveVoltage
        y = sign(voltage) * c.MinDriveVoltage;
    else
        y = voltage;
    end
end

function next = step_current_estimate(current, motor_voltage, motor_speed, p, dt)
    numerator = current + (dt / p.La) * (motor_voltage - p.Ke * motor_speed);
    denominator = 1.0 + (dt * p.Ra / p.La);
    next = numerator / denominator;
end
```

- [ ] **Step 4: Create step_mpc.m**

```matlab
function [force, mpc_state] = step_mpc(ctrl_param, x_ctrl, mpc_state, max_force)
% STEP_MPC MPC controller step (wraps mpc_controller).
%   ctrl_param: design_mpc() output
%   x_ctrl: state vector [position; velocity; theta; theta_dot]
%   Returns clamped force [N] and updated MPC state.

    util = sim_utils();
    [force, mpc_state] = mpc_controller(ctrl_param, x_ctrl, mpc_state);
    force = util.clamp(force, -max_force, max_force);
end
```

- [ ] **Step 5: Update FirmwareExperiment.m step_balance_loop to dispatch to step functions**

Replace the switch block in `step_balance_loop` (the `case 'pid'`, `case 'lqr'`, `case 'mrac'`, `case 'mpc'` sections) with calls to the new step functions. The `case 'debug'` remains inline since it's trivial.

The new switch block in `step_balance_loop` should be:

```matlab
switch obj.mode_name
    case 'debug'
        runtime.target_force = 0.0;
        runtime.target_current = 0.0;
        runtime.target_voltage = 0.0;
        velocity_est = observer_velocity;
        theta_dot_est = observer_theta_dot;
        current_used = NaN;
        current_state = NaN;

    case 'pid'
        processed = struct( ...
            'position', position, ...
            'velocity', observer_velocity, ...
            'theta', theta, ...
            'theta_dot', observer_theta_dot);
        [force, runtime.balance_pid] = step_pid( ...
            runtime.balance_pid, processed, obj.options.max_force);
        runtime.target_force = force;
        runtime.target_current = runtime.target_force * FirmwareExperiment.force_to_current(obj.p);
        runtime.target_voltage = 0.0;
        velocity_est = processed.velocity;
        theta_dot_est = processed.theta_dot;
        current_used = NaN;
        current_state = NaN;

    case 'lqr'
        x_ctrl = [position; observer_velocity; theta; observer_theta_dot];
        [force, runtime.target_current] = step_lqr( ...
            obj.ctrl_param, x_ctrl, obj.options.max_force, obj.p);
        runtime.target_force = force;
        runtime.target_voltage = 0.0;
        velocity_est = observer_velocity;
        theta_dot_est = observer_theta_dot;
        current_used = NaN;
        current_state = NaN;

    case 'mrac'
        processed = struct( ...
            'position', position, ...
            'velocity', velocity, ...
            'theta', theta, ...
            'theta_dot', theta_dot_direct);
        [runtime.target_voltage, runtime.mrac, current_used, current_state] = ...
            step_mrac(runtime.mrac, processed, signed_current, obj.p, balance_dt);
        runtime.target_force = 0.0;
        runtime.target_current = 0.0;
        velocity_est = processed.velocity;
        theta_dot_est = processed.theta_dot;

    case 'mpc'
        x_ctrl = [position; observer_velocity; theta; observer_theta_dot];
        [force, runtime.mpc_state] = step_mpc( ...
            obj.ctrl_param, x_ctrl, runtime.mpc_state, obj.options.max_force);
        runtime.target_force = force;
        runtime.target_current = runtime.target_force * FirmwareExperiment.force_to_current(obj.p);
        runtime.target_voltage = 0.0;
        velocity_est = observer_velocity;
        theta_dot_est = observer_theta_dot;
        current_used = NaN;
        current_state = NaN;

    otherwise
        error('FirmwareExperiment:UnsupportedMode', 'Unsupported mode: %s', obj.mode_name);
end
```

Also remove the following static methods from FirmwareExperiment.m since they are now in the step files:
- `balance_pid_step`
- `init_balance_pid`
- `pid_update`
- `init_pid`
- `mrac_step`
- `init_mrac_controller`
- `soft_zone`
- `apply_low_speed_drive_compensation`
- `step_current_estimate`

Note: `init_balance_pid` and `init_pid` are still needed by `init_runtime`. Move them as local functions inside `step_pid.m`:

Add to `step_pid.m`:

```matlab
function balance_pid = init_balance_pid(gains, dt, max_force)
    balance_pid.angle = init_pid(gains.theta, dt, -max_force, max_force);
    balance_pid.position = init_pid(gains.x, dt, -max_force, max_force);
end

function pid = init_pid(cfg, dt, min_output, max_output)
    pid.kp = cfg.Kp;
    pid.ki = cfg.Ki;
    pid.kd = cfg.Kd;
    pid.dt = dt;
    pid.min_output = min_output;
    pid.max_output = max_output;
    pid.integral = 0.0;
    pid.prev_error = 0.0;
    pid.first_call = true;
end
```

But since MATLAB local functions in a function file are only visible inside that file, `init_balance_pid` needs to be callable from FirmwareExperiment's `init_runtime`. Make `step_pid.m` a function that returns a struct of function handles (like sim_utils):

Replace `step_pid.m` entirely with:

```matlab
function api = step_pid()
% STEP_PID Cascaded PID balance controller.
%   api = step_pid();
%   balance_pid = api.init(gains, dt, max_force);
%   [force, balance_pid] = api.step(balance_pid, state, max_force);

    api.init = @init_balance_pid;
    api.step = @balance_pid_step;
end

function balance_pid = init_balance_pid(gains, dt, max_force)
    balance_pid.angle = init_pid(gains.theta, dt, -max_force, max_force);
    balance_pid.position = init_pid(gains.x, dt, -max_force, max_force);
end

function pid = init_pid(cfg, dt, min_output, max_output)
    pid.kp = cfg.Kp;
    pid.ki = cfg.Ki;
    pid.kd = cfg.Kd;
    pid.dt = dt;
    pid.min_output = min_output;
    pid.max_output = max_output;
    pid.integral = 0.0;
    pid.prev_error = 0.0;
    pid.first_call = true;
end

function [force, balance_pid] = balance_pid_step(balance_pid, state, max_force)
    util = sim_utils();
    [u_theta, balance_pid.angle] = pid_update( ...
        balance_pid.angle, state.theta, 0.0);
    [u_x, balance_pid.position] = pid_update( ...
        balance_pid.position, state.position, 0.0);
    force = util.clamp(u_theta + u_x, -max_force, max_force);
end

function [output, pid] = pid_update(pid, setpoint, measurement)
    util = sim_utils();
    error_value = setpoint - measurement;
    p_term = pid.kp * error_value;
    pid.integral = pid.integral + pid.ki * error_value * pid.dt;

    if pid.first_call
        d_term = 0.0;
        pid.first_call = false;
    else
        d_term = pid.kd * (error_value - pid.prev_error) / pid.dt;
    end

    unclamped = p_term + pid.integral + d_term;
    output = util.clamp(unclamped, pid.min_output, pid.max_output);

    if unclamped > pid.max_output && pid.integral > 0.0
        pid.integral = max(pid.max_output - p_term - d_term, 0.0);
    elseif unclamped < pid.min_output && pid.integral < 0.0
        pid.integral = min(pid.min_output - p_term - d_term, 0.0);
    end

    pid.prev_error = error_value;
end
```

Similarly, make `step_mrac.m` return a struct with `init` and `step`:

```matlab
function api = step_mrac()
% STEP_MRAC Model Reference Adaptive Control.
%   api = step_mrac();
%   controller = api.init(ctrl_param);
%   [voltage, controller, current_used, current_state] = api.step(controller, state, measured_current, p, dt);

    api.init = @init_mrac_controller;
    api.step = @mrac_step;
end

function controller = init_mrac_controller(c)
    controller.reference_state = zeros(5, 1);
    controller.adaptive_gains = zeros(5, 1);
    controller.initialized = false;
    controller.prev_voltage = 0.0;
    controller.current_state = 0.0;
    controller.startup_timer = 0.0;
    controller.ctrl = c;
end

function [voltage, controller, current_used, current_state] = mrac_step(controller, state, measured_current, p, dt)
    % ... (full mrac_step code as shown in Step 3 above)
```
(Include the full `mrac_step`, `soft_zone`, `apply_low_speed_drive_compensation`, and `step_current_estimate` functions as shown in Step 3.)

- [ ] **Step 6: Update FirmwareExperiment.m init_runtime to use step_pid and step_mrac**

In `init_runtime`, change:
```matlab
case 'pid'
    runtime.balance_pid = FirmwareExperiment.init_balance_pid( ...
        obj.ctrl_param, balance_dt, opts.max_force);
```
to:
```matlab
case 'pid'
    pid_api = step_pid();
    runtime.balance_pid = pid_api.init( ...
        obj.ctrl_param, balance_dt, opts.max_force);
```

Change:
```matlab
case 'mrac'
    runtime.mrac = FirmwareExperiment.init_mrac_controller(obj.ctrl_param);
```
to:
```matlab
case 'mrac'
    mrac_api = step_mrac();
    runtime.mrac = mrac_api.init(obj.ctrl_param);
```

Also update the current loop PID initialization. The `init_pid` function is now inside `step_pid.m` and not directly accessible. Keep `init_pid` as a static method on FirmwareExperiment for the current PID, since it's part of the current loop (not the balance controller). Rename it to `init_current_pid` for clarity:

```matlab
function pid = init_current_pid(cfg, dt, min_output, max_output)
    pid.kp = cfg.Kp;
    pid.ki = cfg.Ki;
    pid.kd = cfg.Kd;
    pid.dt = dt;
    pid.min_output = min_output;
    pid.max_output = max_output;
    pid.integral = 0.0;
    pid.prev_error = 0.0;
    pid.first_call = true;
end
```

And keep `pid_update` as a static method on FirmwareExperiment for the current loop PID. The current loop's `pid_update` in `step_current_loop` stays as `FirmwareExperiment.pid_update(...)`.

Actually, to avoid duplication: keep `init_pid` and `pid_update` as static methods on FirmwareExperiment (they are used by the current loop). The step_pid.m file can call them or duplicate them. Since MATLAB function files can't call another file's local functions, `step_pid.m` must have its own copies of `init_pid` and `pid_update` as local functions. This is acceptable — the PID logic is small (15 lines each) and the two uses (balance PID and current PID) may diverge.

Keep `init_pid` and `pid_update` as static methods on FirmwareExperiment for the current loop.

- [ ] **Step 7: Run regression test**

Run: `test_baseline` in MATLAB
Expected: All 4 modes `[PASS]`

- [ ] **Step 8: Commit**

```bash
git add MATLAB/control/step_pid.m MATLAB/control/step_lqr.m MATLAB/control/step_mrac.m MATLAB/control/step_mpc.m MATLAB/sim/FirmwareExperiment.m
git commit -m "refactor(matlab): extract controller step functions from FirmwareExperiment

Move PID, LQR, MRAC, and MPC step logic to individual files under
control/. Each file provides init() and step() via function handle struct.
FirmwareExperiment now dispatches to these modules."
```

---

### Task 7: Create export_utils.m and consolidate Rust export helpers

**Files:**
- Create: `MATLAB/control/export_utils.m`
- Modify: `MATLAB/control/export_lqr_rust.m`
- Modify: `MATLAB/control/export_pid_rust.m`
- Modify: `MATLAB/control/export_mrac_rust.m`
- Modify: `MATLAB/control/export_observer_rust.m`

- [ ] **Step 1: Create export_utils.m**

```matlab
function util = export_utils()
% EXPORT_UTILS Shared Rust code generation helpers.
%   util = export_utils();
%   util.print_scalar(name, value)
%   util.print_vector(name, vector)
%   util.print_matrix(name, matrix)

    util.print_scalar = @print_rust_scalar;
    util.print_vector = @print_rust_vector;
    util.print_matrix = @print_rust_matrix;
end

function print_rust_scalar(name, value)
    fprintf('const %s: f32 = %.9g;\n\n', name, value);
end

function print_rust_vector(name, vector)
    vector = vector(:);
    fprintf('const %s: [f32; %d] = [', name, numel(vector));
    for idx = 1:numel(vector)
        fprintf('%.9g', vector(idx));
        if idx < numel(vector)
            fprintf(', ');
        end
    end
    fprintf('];\n\n');
end

function print_rust_matrix(name, matrix)
    [rows, cols] = size(matrix);
    fprintf('const %s: [[f32; %d]; %d] = [\n', name, cols, rows);
    for r = 1:rows
        fprintf('    [');
        for c = 1:cols
            fprintf('%.9g', matrix(r, c));
            if c < cols
                fprintf(', ');
            end
        end
        fprintf('],\n');
    end
    fprintf('];\n\n');
end
```

- [ ] **Step 2: Update export_lqr_rust.m**

Replace entire file with:

```matlab
function export_lqr_rust()
% EXPORT_LQR_RUST Print Rust constants for the firmware LQR controller.

    eu = export_utils();
    p = params();
    [A, B, ~, ~] = linearize_system(p);
    K = design_lqr(A, B);

    fprintf('// Designed in MATLAB/control/design_lqr.m\n');
    fprintf('// Q = diag([100, 0, 50, 100]), R = 10\n');
    eu.print_scalar('K_POSITION', -K(1));
    eu.print_scalar('K_VELOCITY', -K(2));
    eu.print_scalar('K_ANGLE', -K(3));
    eu.print_scalar('K_ANGULAR_VELOCITY', -K(4));
end
```

- [ ] **Step 3: Update export_pid_rust.m**

Replace entire file with:

```matlab
function export_pid_rust()
% EXPORT_PID_RUST Print Rust constants for the firmware PID balance controller.

    eu = export_utils();
    pid_gains = design_pid();

    fprintf('// Designed in MATLAB/control/design_pid.m\n');
    fprintf('// 角度制御（内側ループ）\n');
    eu.print_scalar('BALANCE_ANGLE_KP', pid_gains.theta.Kp);
    eu.print_scalar('BALANCE_ANGLE_KI', pid_gains.theta.Ki);
    eu.print_scalar('BALANCE_ANGLE_KD', pid_gains.theta.Kd);
    fprintf('\n');
    fprintf('// 位置制御（外側ループ）\n');
    eu.print_scalar('BALANCE_POS_KP', pid_gains.x.Kp);
    eu.print_scalar('BALANCE_POS_KI', pid_gains.x.Ki);
    eu.print_scalar('BALANCE_POS_KD', pid_gains.x.Kd);
end
```

- [ ] **Step 4: Update export_mrac_rust.m**

Replace entire file with:

```matlab
function export_mrac_rust()
% EXPORT_MRAC_RUST Print Rust constants for the firmware MRAC controller.

    eu = export_utils();
    p = params();
    mrac = design_mrac(p);

    fprintf('// Designed in MATLAB/control/design_mrac.m\n');
    eu.print_vector('MRAC_KX', mrac.Kx);
    eu.print_matrix('MRAC_AM', mrac.Am);
    eu.print_matrix('MRAC_P', mrac.P);
    eu.print_vector('MRAC_PB', mrac.PB);
    eu.print_vector('MRAC_GAMMA_DIAG', diag(mrac.GammaDiag));
    eu.print_scalar('MRAC_SIGMA', mrac.Sigma);
    eu.print_scalar('MRAC_NORMALIZATION_EPS', mrac.NormalizationEps);
    eu.print_scalar('MRAC_MAX_VOLTAGE', mrac.MaxVoltage);
    eu.print_scalar('MRAC_MAX_VOLTAGE_SLEW_RATE', mrac.MaxVoltageSlewRate);
    eu.print_scalar('MRAC_ADAPTATION_DELAY_SEC', mrac.AdaptationDelaySec);
    eu.print_scalar('MRAC_ERROR_DEADZONE', mrac.ErrorDeadzone);
    eu.print_scalar('MRAC_MIN_ACTIVE_VOLTAGE', mrac.MinActiveVoltage);
    eu.print_scalar('MRAC_MIN_DRIVE_VOLTAGE', mrac.MinDriveVoltage);
    eu.print_scalar('MRAC_LOW_SPEED_VELOCITY_THRESHOLD', mrac.LowSpeedVelocityThreshold);
    eu.print_scalar('MRAC_STICTION_ENABLE_ANGLE_LIMIT', mrac.StictionEnableAngleLimit);
    eu.print_scalar('MRAC_STICTION_ENABLE_ANGULAR_VELOCITY_LIMIT', mrac.StictionEnableAngularVelocityLimit);
    eu.print_scalar('MRAC_THETA_SOFT_ZONE', mrac.ThetaSoftZone);
    eu.print_scalar('MRAC_THETA_DOT_SOFT_ZONE', mrac.ThetaDotSoftZone);
    eu.print_scalar('MRAC_INNER_ANGLE_GAIN', mrac.InnerAngleGain);
    eu.print_scalar('MRAC_INNER_ANGULAR_VELOCITY_GAIN', mrac.InnerAngularVelocityGain);
    eu.print_scalar('MRAC_CURRENT_SOFT_ZONE', mrac.CurrentSoftZone);
    eu.print_scalar('MRAC_INNER_CURRENT_GAIN', mrac.InnerCurrentGain);
    eu.print_scalar('MRAC_MAX_ADAPTIVE_GAIN', mrac.MaxAdaptiveGain);
    eu.print_scalar('MRAC_ENABLE_ANGLE_LIMIT', mrac.EnableAngleLimit);
    eu.print_scalar('MRAC_ENABLE_ANGULAR_VELOCITY_LIMIT', mrac.EnableAngularVelocityLimit);
    eu.print_scalar('MRAC_ENABLE_CURRENT_LIMIT', mrac.EnableCurrentLimit);
end
```

- [ ] **Step 5: Update export_observer_rust.m**

Replace entire file with:

```matlab
function export_observer_rust()
% EXPORT_OBSERVER_RUST Print Rust constants for the firmware observer.

    eu = export_utils();
    p = params();
    [A, B, C, ~] = linearize_system(p);
    obs = design_observer(A, B, C);

    fprintf('// Designed in MATLAB/control/design_observer.m\n');
    eu.print_matrix('AD', obs.Ad);
    eu.print_vector('BD', obs.Bd);
    eu.print_matrix('LD', obs.Ld);
end
```

- [ ] **Step 6: Verify exports produce identical output**

Run each export function in MATLAB and verify output matches the previous version:
```matlab
export_lqr_rust
export_pid_rust
export_observer_rust
export_mrac_rust
```
Expected: Output is identical to before refactoring.

- [ ] **Step 7: Commit**

```bash
git add MATLAB/control/export_utils.m MATLAB/control/export_lqr_rust.m MATLAB/control/export_pid_rust.m MATLAB/control/export_mrac_rust.m MATLAB/control/export_observer_rust.m
git commit -m "refactor(matlab): consolidate Rust export helpers into export_utils.m

Extract print_rust_scalar, print_rust_vector, and print_rust_matrix into
a shared export_utils module. Remove duplicated local helper functions
from all four export files."
```

---

### Task 8: Clean up FirmwareExperiment.m and final verification

Remove any remaining dead static methods from FirmwareExperiment.m and do final regression check.

**Files:**
- Modify: `MATLAB/sim/FirmwareExperiment.m`

- [ ] **Step 1: Review FirmwareExperiment.m for remaining dead methods**

After tasks 4-6, FirmwareExperiment.m should contain only:
- **Properties**: p, mode, firmware_mode, mode_name, ctrl_param, options
- **Public methods**: constructor, `run()`, `simulate()` (static convenience)
- **Private methods**: `init_runtime`, `init_history`, `empty_balance_sample`, `step_balance_loop`, `step_current_loop`, `theta_filter_cutoff`, `theta_dot_filter_cutoff`
- **Private static methods**: `measure_theta`, `measure_encoder`, `pulse_to_position`, `force_to_current`, `correct_current_sign`, `step_motor_current`, `rk4_step`, `eval_disturbance`, `init_pid`, `pid_update`, `init_observer`, `update_observer`

Verify no orphaned methods remain. Remove any that are no longer called.

- [ ] **Step 2: Run full regression test**

Run: `test_baseline` in MATLAB
Expected: All 4 modes `[PASS]`

- [ ] **Step 3: Run main.m end-to-end**

Run: `main` in MATLAB
Expected: Simulation runs successfully, produces the same comparison plot with 4 controllers, performance metrics print correctly.

- [ ] **Step 4: Commit**

```bash
git add MATLAB/sim/FirmwareExperiment.m
git commit -m "refactor(matlab): clean up FirmwareExperiment after extraction

Remove orphaned static methods. FirmwareExperiment now contains only
simulation core logic (~350 lines, down from 742)."
```

---

### Task 9: Clean up test artifacts

Remove the baseline test file and .mat data since they were only needed during refactoring.

**Files:**
- Delete: `MATLAB/test_baseline.m`
- Delete: `MATLAB/test_baseline.mat`

- [ ] **Step 1: Remove test artifacts**

```bash
git rm MATLAB/test_baseline.m
rm -f MATLAB/test_baseline.mat
git commit -m "chore(matlab): remove refactoring regression test artifacts"
```
