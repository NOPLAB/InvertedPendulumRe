# MATLAB Coder → Rust FFI Controller Code Generation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace hand-written Rust controller implementations with MATLAB Embedded Coder generated C code, consumed via FFI through an independent Rust crate.

**Architecture:** MATLAB `build.m` designs all controllers and generates C code (via Embedded Coder) and MEX files. A new `inverted-pendulum-controller` crate wraps the generated C in safe Rust APIs. STM32 firmware depends on this crate instead of its own controller implementations. `build.rs` automates MATLAB invocation and C cross-compilation.

**Tech Stack:** MATLAB R2025b + Embedded Coder, Rust (`no_std`, `cc` crate, `glob` crate), `arm-none-eabi-gcc`

---

## File Structure

### New files to create:
- `MATLAB/build.m` — Code generation pipeline entry point
- `MATLAB/control/codegen/codegen_controller.m` — Orchestrates code generation for all controllers
- `MATLAB/control/codegen/codegen_config.m` — Embedded Coder configuration (lib + mex targets)
- `MATLAB/control/codegen/entry_points/lqr_step.m` — LQR Coder entry point
- `MATLAB/control/codegen/entry_points/pid_step.m` — Cascaded PID Coder entry point
- `MATLAB/control/codegen/entry_points/mpc_step.m` — MPC ADMM Coder entry point
- `MATLAB/control/codegen/entry_points/mrac_step.m` — MRAC Coder entry point
- `MATLAB/control/codegen/entry_points/observer_step.m` — Observer Coder entry point
- `inverted-pendulum-controller/Cargo.toml` — New crate manifest
- `inverted-pendulum-controller/build.rs` — MATLAB invocation + cc compilation
- `inverted-pendulum-controller/src/lib.rs` — Safe Rust API wrappers
- `inverted-pendulum-controller/src/ffi.rs` — `extern "C"` declarations
- `inverted-pendulum-controller/.gitignore` — Exclude `codegen/`
- `MATLAB/sim/mex/.gitignore` — Exclude MEX build artifacts

### Files to modify:
- `inverted-pendulum-stm32/Cargo.toml` — Add dependency on controller crate
- `inverted-pendulum-stm32/src/controller/mod.rs` — Replace controller struct fields with FFI calls
- `inverted-pendulum-stm32/src/tasks/mod.rs` — Remove `mpc` module
- `inverted-pendulum-stm32/src/main.rs` — Remove `mpc_task` spawn
- `MATLAB/sim/FirmwareExperiment.m` — Use MEX-generated controllers in simulation

### Files to delete:
- `inverted-pendulum-stm32/src/controller/lqr.rs`
- `inverted-pendulum-stm32/src/controller/lqr_constants.rs`
- `inverted-pendulum-stm32/src/controller/pid_balance.rs`
- `inverted-pendulum-stm32/src/controller/pid_balance_constants.rs`
- `inverted-pendulum-stm32/src/controller/mpc.rs`
- `inverted-pendulum-stm32/src/controller/mpc_constants.rs`
- `inverted-pendulum-stm32/src/controller/mrac.rs`
- `inverted-pendulum-stm32/src/controller/mrac_constants.rs`
- `inverted-pendulum-stm32/src/controller/observer.rs`
- `inverted-pendulum-stm32/src/controller/observer_constants.rs`
- `inverted-pendulum-stm32/src/tasks/mpc.rs`

---

## Task 1: MATLAB Coder Entry Point Functions

Create the `%#codegen`-annotated entry point functions that MATLAB Coder will use to generate C code. Each function takes `single`-precision inputs and uses `persistent` variables for internal state.

**Files:**
- Create: `MATLAB/control/codegen/entry_points/lqr_step.m`
- Create: `MATLAB/control/codegen/entry_points/pid_step.m`
- Create: `MATLAB/control/codegen/entry_points/mpc_step.m`
- Create: `MATLAB/control/codegen/entry_points/mrac_step.m`
- Create: `MATLAB/control/codegen/entry_points/observer_step.m`

- [ ] **Step 1: Create `lqr_step.m`**

LQR is stateless: `u = -K * x`. The gain `K` is injected as a `coder.Constant`.

```matlab
function force = lqr_step(K, state)  %#codegen
% LQR_STEP LQR state feedback controller (codegen entry point).
%   K:     single(1x4) — LQR gain vector
%   state: single(4x1) — [x; x_dot; theta; theta_dot]
%   force: single       — control force [N]

    assert(isa(K, 'single'));
    assert(all(size(K) == [1 4]));
    assert(isa(state, 'single'));
    assert(all(size(state) == [4 1]));

    force = -(K * state);
end
```

- [ ] **Step 2: Create `pid_step.m`**

Cascaded PID with anti-windup. Uses `persistent` for integrator and derivative state. Gains injected as `coder.Constant` struct. Mirrors the logic in `MATLAB/control/step/step_pid.m` and `inverted-pendulum-stm32/src/controller/pid_balance.rs`.

```matlab
function force = pid_step(gains, dt, max_force, state)  %#codegen
% PID_STEP Cascaded PID balance controller (codegen entry point).
%   gains:     struct with fields theta (Kp,Ki,Kd) and x (Kp,Ki,Kd), single
%   dt:        single — balance loop period [s]
%   max_force: single — force saturation limit [N]
%   state:     single(4x1) — [x; x_dot; theta; theta_dot]
%   force:     single       — control force [N]

    assert(isa(state, 'single'));
    assert(all(size(state) == [4 1]));

    persistent angle_integral angle_prev_error angle_first_call
    persistent pos_integral pos_prev_error pos_first_call

    if isempty(angle_integral)
        angle_integral = single(0);
        angle_prev_error = single(0);
        angle_first_call = true;
        pos_integral = single(0);
        pos_prev_error = single(0);
        pos_first_call = true;
    end

    % 角度PID（内側ループ）: setpoint=theta, measurement=0
    theta = state(3);
    [u_theta, angle_integral, angle_prev_error, angle_first_call] = ...
        pid_update(gains.theta, dt, max_force, theta, single(0), ...
                   angle_integral, angle_prev_error, angle_first_call);

    % 位置PID（外側ループ）: setpoint=position, measurement=0
    position = state(1);
    [u_x, pos_integral, pos_prev_error, pos_first_call] = ...
        pid_update(gains.x, dt, max_force, position, single(0), ...
                   pos_integral, pos_prev_error, pos_first_call);

    force = u_theta + u_x;
end

function [output, integral, prev_error, first_call] = ...
        pid_update(cfg, dt, max_force, setpoint, measurement, ...
                   integral, prev_error, first_call)
    error_val = setpoint - measurement;
    p_term = cfg.Kp * error_val;
    integral = integral + cfg.Ki * error_val * dt;

    if first_call
        d_term = single(0);
        first_call = false;
    else
        d_term = cfg.Kd * (error_val - prev_error) / dt;
    end

    unclamped = p_term + integral + d_term;
    output = max(min(unclamped, max_force), -max_force);

    if unclamped > max_force && integral > single(0)
        integral = max(max_force - p_term - d_term, single(0));
    elseif unclamped < -max_force && integral < single(0)
        integral = min(-max_force - p_term - d_term, single(0));
    end

    prev_error = error_val;
end
```

- [ ] **Step 3: Create `mpc_step.m`**

ADMM-based MPC with warm-start via persistent variables. Mirrors `MATLAB/sim/mpc_controller.m` and `inverted-pendulum-stm32/src/controller/mpc.rs`.

```matlab
function force = mpc_step(N, rho, max_iter, u_min, u_max, ...
                           H_inv_rho, F_mat, state)  %#codegen
% MPC_STEP MPC controller with ADMM QP solver (codegen entry point).
%   N:          single — prediction horizon (cast to index internally)
%   rho:        single — ADMM penalty parameter
%   max_iter:   single — max ADMM iterations (cast to index internally)
%   u_min:      single — lower input constraint [N]
%   u_max:      single — upper input constraint [N]
%   H_inv_rho:  single(10x10) — precomputed (H + rho*I)^{-1}
%   F_mat:      single(10x4) — gradient coefficient matrix
%   state:      single(4x1)  — current state vector

    assert(isa(state, 'single'));
    assert(all(size(state) == [4 1]));

    n_var = int32(N);

    persistent U Z W first_call
    if isempty(first_call)
        U = zeros(10, 1, 'single');
        Z = zeros(10, 1, 'single');
        W = zeros(10, 1, 'single');
        first_call = true;
    end

    % warm-start: 前回の解を1ステップシフト
    if ~first_call
        U = [U(2:n_var); U(n_var)];
        Z = [Z(2:n_var); Z(n_var)];
        W = zeros(10, 1, 'single');
    else
        first_call = false;
    end

    % 勾配ベクトル
    f = F_mat * state;

    % ADMM反復
    n_iter = int32(max_iter);
    for iter = 1:n_iter
        % U更新
        rhs = -f + rho * (Z - W);
        U = H_inv_rho * rhs;

        % Z更新: box制約射影
        for i = 1:n_var
            val = U(i) + W(i);
            Z(i) = max(min(val, u_max), u_min);
        end

        % W更新: 双対変数
        W = W + U - Z;
    end

    force = U(1);
end
```

- [ ] **Step 4: Create `mrac_step.m`**

MRAC with LQR-anchored adaptation. Uses persistent for reference model state and adaptive gains. Mirrors `MATLAB/control/step/step_mrac.m` and `inverted-pendulum-stm32/src/controller/mrac.rs`.

```matlab
function force = mrac_step(c, dt, max_force, state)  %#codegen
% MRAC_STEP Model Reference Adaptive Controller (codegen entry point).
%   c:         struct — MRAC parameters from design_mrac()
%   dt:        single — balance loop period [s]
%   max_force: single — force saturation limit [N]
%   state:     single(4x1) — [x; x_dot; theta; theta_dot]
%   force:     single       — control force [N]

    assert(isa(state, 'single'));
    assert(all(size(state) == [4 1]));

    persistent ref_state adaptive_gains initialized startup_timer

    if isempty(initialized)
        ref_state = zeros(4, 1, 'single');
        adaptive_gains = zeros(5, 1, 'single');
        initialized = false;
        startup_timer = single(0);
    end

    x = state;
    phi = [x; single(1)];

    if ~initialized
        ref_state = x;
        initialized = true;
        startup_timer = single(0);
    else
        ref_state = ref_state + dt * (c.Am * ref_state);
        startup_timer = startup_timer + dt;
    end

    error_val = x - ref_state;
    s = error_val' * c.PB;
    phi_norm = phi' * phi;

    adaptation_enabled = ...
        abs(state(3)) < c.EnableAngleLimit && ...
        abs(state(4)) < c.EnableAngularVelocityLimit && ...
        abs(state(1)) < c.EnablePositionLimit && ...
        abs(state(2)) < c.EnableVelocityLimit && ...
        startup_timer >= c.AdaptationDelaySec && ...
        abs(s) > c.ErrorDeadzone;

    if adaptation_enabled
        scale = s / (c.NormalizationEps + phi_norm);
        for idx = 1:5
            update_val = c.GammaDiag(idx) * phi(idx) * scale ...
                - c.Sigma * adaptive_gains(idx);
            adaptive_gains(idx) = adaptive_gains(idx) + dt * update_val;
            adaptive_gains(idx) = max(min(adaptive_gains(idx), ...
                c.MaxAdaptiveGain), -c.MaxAdaptiveGain);
        end
    else
        adaptive_gains = adaptive_gains + dt * (-c.Sigma * adaptive_gains);
    end

    adaptive_force = max(min(adaptive_gains' * phi, ...
        c.MaxAdaptiveForce), -c.MaxAdaptiveForce);
    nominal_force = -(c.Kx * x);
    force = max(min(nominal_force - adaptive_force, max_force), -max_force);
end
```

- [ ] **Step 5: Create `observer_step.m`**

Discrete Luenberger observer. Mirrors `inverted-pendulum-stm32/src/controller/observer.rs`.

```matlab
function x_hat = observer_step(Ad, Bd, Ld, position, theta, force)  %#codegen
% OBSERVER_STEP Discrete Luenberger state observer (codegen entry point).
%   Ad:       single(4x4) — discrete state transition matrix
%   Bd:       single(4x1) — discrete input matrix
%   Ld:       single(4x2) — observer gain matrix
%   position: single       — measured cart position [m]
%   theta:    single       — measured pendulum angle [rad]
%   force:    single       — applied force [N] (previous step)
%   x_hat:    single(4x1)  — estimated state vector

    assert(isa(position, 'single'));
    assert(isa(theta, 'single'));
    assert(isa(force, 'single'));

    persistent state_hat prev_force initialized

    if isempty(initialized)
        state_hat = zeros(4, 1, 'single');
        prev_force = single(0);
        initialized = false;
    end

    if ~initialized
        state_hat = single([position; 0; theta; 0]);
        prev_force = force;
        initialized = true;
        x_hat = state_hat;
        return;
    end

    % イノベーション
    y_meas = single([position; theta]);
    y_hat = single([state_hat(1); state_hat(3)]);
    innovation = y_meas - y_hat;

    % 状態更新: x_hat[k+1] = Ad * x_hat[k] + Bd * u[k] + Ld * innovation
    state_hat = Ad * state_hat + Bd * prev_force + Ld * innovation;
    prev_force = force;

    x_hat = state_hat;
end
```

- [ ] **Step 6: Verify entry points run in MATLAB**

Run each entry point manually in MATLAB to confirm correctness before code generation.

Run in MATLAB:
```matlab
addpath(genpath('.')); p = params(); [A,B,C,D] = linearize_system(p);
K = single(design_lqr(A,B)); s = single([0;0;0.1;0]);

% LQR
f = lqr_step(K, s); fprintf('LQR: %.4f\n', f);

% Observer
obs = design_observer(A,B,C);
Ad_s = single(obs.Ad); Bd_s = single(obs.Bd); Ld_s = single(obs.Ld);
x1 = observer_step(Ad_s, Bd_s, Ld_s, single(0), single(0.1), single(0));
fprintf('Observer: [%.4f, %.4f, %.4f, %.4f]\n', x1);

% PID
gains.theta.Kp=single(50); gains.theta.Ki=single(5); gains.theta.Kd=single(15);
gains.x.Kp=single(10); gains.x.Ki=single(0.5); gains.x.Kd=single(15);
f2 = pid_step(gains, single(0.001), single(10), s); fprintf('PID: %.4f\n', f2);

% MPC
mpc_d = design_mpc(A,B);
f3 = mpc_step(single(10), single(1), single(50), single(-10), single(10), ...
    single(mpc_d.H_inv_rho), single(mpc_d.F), s);
fprintf('MPC: %.4f\n', f3);

% MRAC
mrac_d = design_mrac(p);
mrac_c.Am=single(mrac_d.Am); mrac_c.PB=single(mrac_d.PB);
mrac_c.Kx=single(mrac_d.Kx); mrac_c.GammaDiag=single(diag(mrac_d.GammaDiag));
mrac_c.Sigma=single(mrac_d.Sigma);
mrac_c.NormalizationEps=single(mrac_d.NormalizationEps);
mrac_c.AdaptationDelaySec=single(mrac_d.AdaptationDelaySec);
mrac_c.ErrorDeadzone=single(mrac_d.ErrorDeadzone);
mrac_c.MaxAdaptiveGain=single(mrac_d.MaxAdaptiveGain);
mrac_c.MaxAdaptiveForce=single(mrac_d.MaxAdaptiveForce);
mrac_c.EnableAngleLimit=single(mrac_d.EnableAngleLimit);
mrac_c.EnableAngularVelocityLimit=single(mrac_d.EnableAngularVelocityLimit);
mrac_c.EnablePositionLimit=single(mrac_d.EnablePositionLimit);
mrac_c.EnableVelocityLimit=single(mrac_d.EnableVelocityLimit);
f4 = mrac_step(mrac_c, single(0.001), single(10), s);
fprintf('MRAC: %.4f\n', f4);
```

Expected: All print non-zero force values without errors.

- [ ] **Step 7: Commit**

```bash
git add MATLAB/control/codegen/entry_points/
git commit -m "feat(matlab): add MATLAB Coder entry point functions for all controllers"
```

---

## Task 2: Coder Configuration and Code Generation Script

Create the Embedded Coder configuration and the orchestrator that generates both MEX and C library targets.

**Files:**
- Create: `MATLAB/control/codegen/codegen_config.m`
- Create: `MATLAB/control/codegen/codegen_controller.m`

- [ ] **Step 1: Create `codegen_config.m`**

Returns Embedded Coder configuration objects for both `lib` (C static library) and `mex` targets.

```matlab
function [cfg_lib, cfg_mex] = codegen_config(lib_output_dir, mex_output_dir)
% CODEGEN_CONFIG Embedded Coder configuration for controller code generation.
%   cfg_lib: coder.EmbeddedCodeConfig — C static library target
%   cfg_mex: coder.MexCodeConfig      — MEX target for simulation

    %% C library config (for Rust FFI)
    cfg_lib = coder.config('lib');
    cfg_lib.TargetLang = 'C';
    cfg_lib.GenerateReport = true;
    cfg_lib.GenCodeOnly = true;  % build.rs がコンパイルを担当
    cfg_lib.SupportNonFinite = false;
    cfg_lib.DynamicMemoryAllocation = 'Off';
    cfg_lib.FilePartitionMethod = 'SingleFile';  % コントローラごとに1ファイル
    cfg_lib.GenerateExampleMain = false;

    % ハードウェアターゲット: ARM Cortex-M (STM32F303K8)
    hw = coder.HardwareImplementation;
    hw.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-M';
    hw.TargetHWDeviceType = 'ARM Compatible->ARM Cortex-M';
    cfg_lib.HardwareImplementation = hw;

    % 出力ディレクトリ
    cfg_lib.CodeGenDirectory = lib_output_dir;

    %% MEX config (for MATLAB simulation)
    cfg_mex = coder.config('mex');
    cfg_mex.GenerateReport = true;
    cfg_mex.EnableAutoExtrinsicCalls = false;

    % MEX出力ディレクトリ
    cfg_mex.CodeGenDirectory = mex_output_dir;
end
```

- [ ] **Step 2: Create `codegen_controller.m`**

Orchestrates code generation for all 5 controllers. Takes design outputs as arguments, constructs `coder.Constant` types, and runs `codegen` for each entry point.

```matlab
function codegen_controller(K_lqr, pid_gains, mpc_params, mrac_params, obs_params)
% CODEGEN_CONTROLLER Generate C library and MEX for all controllers.
%   K_lqr:       1x4 double — LQR gain from design_lqr()
%   pid_gains:   struct     — PID gains from design_pid()
%   mpc_params:  struct     — MPC parameters from design_mpc()
%   mrac_params: struct     — MRAC parameters from design_mrac()
%   obs_params:  struct     — Observer parameters from design_observer()

    %% 出力先パス
    root_dir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    lib_output = fullfile(root_dir, '..', 'inverted-pendulum-controller', 'codegen');
    mex_output = fullfile(root_dir, 'sim', 'mex');

    % 出力ディレクトリ作成
    if ~exist(lib_output, 'dir'), mkdir(lib_output); end
    if ~exist(mex_output, 'dir'), mkdir(mex_output); end

    [cfg_lib, cfg_mex] = codegen_config(lib_output, mex_output);

    %% 共通: single型の状態入力
    state_type = coder.typeof(single(0), [4 1]);

    fprintf('\n=== Controller Code Generation ===\n\n');

    %% 1. LQR
    fprintf('Generating LQR...\n');
    K_s = single(K_lqr);
    K_const = coder.Constant(K_s);
    codegen lqr_step -args {K_const, state_type} -config cfg_lib
    codegen lqr_step -args {K_const, state_type} -config cfg_mex
    fprintf('  LQR: done\n');

    %% 2. PID
    fprintf('Generating PID...\n');
    pid_s = to_single_pid(pid_gains);
    pid_const = coder.Constant(pid_s);
    dt_const = coder.Constant(single(0.001));  % 1kHz balance loop
    maxf_const = coder.Constant(single(10.0));
    codegen pid_step -args {pid_const, dt_const, maxf_const, state_type} -config cfg_lib
    codegen pid_step -args {pid_const, dt_const, maxf_const, state_type} -config cfg_mex
    fprintf('  PID: done\n');

    %% 3. MPC
    fprintf('Generating MPC...\n');
    N_const = coder.Constant(single(mpc_params.N));
    rho_const = coder.Constant(single(mpc_params.rho));
    iter_const = coder.Constant(single(mpc_params.max_iter));
    umin_const = coder.Constant(single(mpc_params.u_min));
    umax_const = coder.Constant(single(mpc_params.u_max));
    H_const = coder.Constant(single(mpc_params.H_inv_rho));
    F_const = coder.Constant(single(mpc_params.F));
    codegen mpc_step -args {N_const, rho_const, iter_const, umin_const, umax_const, ...
                            H_const, F_const, state_type} -config cfg_lib
    codegen mpc_step -args {N_const, rho_const, iter_const, umin_const, umax_const, ...
                            H_const, F_const, state_type} -config cfg_mex
    fprintf('  MPC: done\n');

    %% 4. MRAC
    fprintf('Generating MRAC...\n');
    mrac_s = to_single_mrac(mrac_params);
    mrac_const = coder.Constant(mrac_s);
    codegen mrac_step -args {mrac_const, dt_const, maxf_const, state_type} -config cfg_lib
    codegen mrac_step -args {mrac_const, dt_const, maxf_const, state_type} -config cfg_mex
    fprintf('  MRAC: done\n');

    %% 5. Observer
    fprintf('Generating Observer...\n');
    Ad_const = coder.Constant(single(obs_params.Ad));
    Bd_const = coder.Constant(single(obs_params.Bd));
    Ld_const = coder.Constant(single(obs_params.Ld));
    scalar_type = coder.typeof(single(0));
    codegen observer_step -args {Ad_const, Bd_const, Ld_const, ...
                                  scalar_type, scalar_type, scalar_type} -config cfg_lib
    codegen observer_step -args {Ad_const, Bd_const, Ld_const, ...
                                  scalar_type, scalar_type, scalar_type} -config cfg_mex
    fprintf('  Observer: done\n');

    fprintf('\n=== Code Generation Complete ===\n');
    fprintf('C library: %s\n', lib_output);
    fprintf('MEX files: %s\n', mex_output);
end

function pid_s = to_single_pid(pid_gains)
    pid_s.theta.Kp = single(pid_gains.theta.Kp);
    pid_s.theta.Ki = single(pid_gains.theta.Ki);
    pid_s.theta.Kd = single(pid_gains.theta.Kd);
    pid_s.x.Kp = single(pid_gains.x.Kp);
    pid_s.x.Ki = single(pid_gains.x.Ki);
    pid_s.x.Kd = single(pid_gains.x.Kd);
end

function mrac_s = to_single_mrac(m)
    mrac_s.Kx = single(m.Kx);
    mrac_s.Am = single(m.Am);
    mrac_s.PB = single(m.PB);
    mrac_s.GammaDiag = single(diag(m.GammaDiag));
    mrac_s.Sigma = single(m.Sigma);
    mrac_s.NormalizationEps = single(m.NormalizationEps);
    mrac_s.AdaptationDelaySec = single(m.AdaptationDelaySec);
    mrac_s.ErrorDeadzone = single(m.ErrorDeadzone);
    mrac_s.MaxAdaptiveGain = single(m.MaxAdaptiveGain);
    mrac_s.MaxAdaptiveForce = single(m.MaxAdaptiveForce);
    mrac_s.EnableAngleLimit = single(m.EnableAngleLimit);
    mrac_s.EnableAngularVelocityLimit = single(m.EnableAngularVelocityLimit);
    mrac_s.EnablePositionLimit = single(m.EnablePositionLimit);
    mrac_s.EnableVelocityLimit = single(m.EnableVelocityLimit);
end
```

- [ ] **Step 3: Run code generation in MATLAB and verify output**

Run in MATLAB:
```matlab
addpath(genpath('.')); p = params(); [A,B,C,D] = linearize_system(p);
K_lqr = design_lqr(A,B); pid_gains = design_pid();
mpc_params = design_mpc(A,B); mrac_params = design_mrac(p);
obs_params = design_observer(A,B,C);
codegen_controller(K_lqr, pid_gains, mpc_params, mrac_params, obs_params);
```

Expected: All 5 controllers generate without errors. Check that:
- `../inverted-pendulum-controller/codegen/` contains `.c` and `.h` files
- `sim/mex/` contains MEX binaries

- [ ] **Step 4: Commit**

```bash
git add MATLAB/control/codegen/codegen_config.m MATLAB/control/codegen/codegen_controller.m
git commit -m "feat(matlab): add Embedded Coder configuration and code generation orchestrator"
```

---

## Task 3: build.m Entry Point

Create the top-level MATLAB build script that runs the full pipeline: design → code generation.

**Files:**
- Create: `MATLAB/build.m`

- [ ] **Step 1: Create `build.m`**

```matlab
%% build.m — コード生成パイプライン
% 使用方法:
%   1. MATLABで実行: build
%   2. コマンドラインから: matlab -batch "cd MATLAB; build"
%
% 出力:
%   - Cライブラリ: ../inverted-pendulum-controller/codegen/
%   - MEXファイル: sim/mex/

%% パス設定
rootDir = fileparts(which(mfilename));
addpath(fullfile(rootDir, 'plant'));
addpath(genpath(fullfile(rootDir, 'control')));
addpath(fullfile(rootDir, 'sim'));

%% 1. プラントモデル
p = params();
[A, B, C, D] = linearize_system(p);
fprintf('Plant linearized.\n');

%% 2. 制御器設計
K_lqr = design_lqr(A, B);
fprintf('LQR designed.\n');

pid_gains = design_pid();
fprintf('PID designed.\n');

obs = design_observer(A, B, C);
fprintf('Observer designed.\n');

mrac = design_mrac(p);
fprintf('MRAC designed.\n');

mpc = design_mpc(A, B);
fprintf('MPC designed.\n');

%% 3. コード生成 (C library + MEX)
codegen_controller(K_lqr, pid_gains, mpc, mrac, obs);

fprintf('\nbuild.m completed successfully.\n');
```

- [ ] **Step 2: Run build.m and verify end-to-end**

Run in MATLAB:
```matlab
cd MATLAB; build
```

Expected: "build.m completed successfully." and generated files in both output directories.

- [ ] **Step 3: Commit**

```bash
git add MATLAB/build.m
git commit -m "feat(matlab): add build.m code generation pipeline entry point"
```

---

## Task 4: inverted-pendulum-controller Rust Crate

Create the new Rust crate with FFI bindings, safe wrappers, and build.rs for MATLAB invocation + C compilation.

**Files:**
- Create: `inverted-pendulum-controller/Cargo.toml`
- Create: `inverted-pendulum-controller/src/ffi.rs`
- Create: `inverted-pendulum-controller/src/lib.rs`
- Create: `inverted-pendulum-controller/build.rs`
- Create: `inverted-pendulum-controller/.gitignore`

- [ ] **Step 1: Create `Cargo.toml`**

```toml
[package]
name = "inverted-pendulum-controller"
version = "0.1.0"
edition = "2021"
description = "MATLAB Coder generated controller library for inverted pendulum"

[build-dependencies]
cc = "1"
glob = "0.3"
```

- [ ] **Step 2: Create `.gitignore`**

```
/codegen/
/target/
```

- [ ] **Step 3: Create `src/ffi.rs`**

The exact function signatures depend on what MATLAB Coder generates. When `coder.Constant` parameters are used, they are baked into the generated C and do NOT appear in the function signature. The generated C functions will have signatures like:

```rust
//! FFI bindings to MATLAB Coder generated C controller functions.
//!
//! Generated function signatures (coder.Constant params are baked in):
//!   float lqr_step(const float state[4]);
//!   float pid_step(const float state[4]);
//!   float mpc_step(const float state[4]);
//!   float mrac_step(const float state[4]);
//!   void  observer_step(float position, float theta, float force, float x_hat[4]);
//!
//! Each function also has an _init() variant to reset persistent variables.

extern "C" {
    pub fn lqr_step_init();
    pub fn lqr_step(state: *const f32) -> f32;

    pub fn pid_step_init();
    pub fn pid_step(state: *const f32) -> f32;

    pub fn mpc_step_init();
    pub fn mpc_step(state: *const f32) -> f32;

    pub fn mrac_step_init();
    pub fn mrac_step(state: *const f32) -> f32;

    pub fn observer_step_init();
    pub fn observer_step(position: f32, theta: f32, force: f32, x_hat: *mut f32);
}
```

**Note:** The exact signatures MUST be verified against the actual MATLAB Coder output headers after Task 2 Step 3 completes. MATLAB Coder may generate different names (e.g., `lqr_step_initialize` instead of `lqr_step_init`). Update this file to match the generated `.h` headers exactly.

- [ ] **Step 4: Create `src/lib.rs`**

```rust
#![no_std]

mod ffi;

/// LQR制御器の内部状態をリセット
pub fn init_lqr() {
    unsafe { ffi::lqr_step_init(); }
}

/// PID制御器の内部状態（積分器等）をリセット
pub fn init_pid() {
    unsafe { ffi::pid_step_init(); }
}

/// MPC制御器の内部状態（ADMM warm start）をリセット
pub fn init_mpc() {
    unsafe { ffi::mpc_step_init(); }
}

/// MRAC制御器の内部状態（適応パラメータ等）をリセット
pub fn init_mrac() {
    unsafe { ffi::mrac_step_init(); }
}

/// 状態オブザーバの内部状態をリセット
pub fn init_observer() {
    unsafe { ffi::observer_step_init(); }
}

/// LQR制御力を計算
/// state: [x, x_dot, theta, theta_dot]
/// 戻り値: 制御力 [N]
pub fn lqr_compute(state: &[f32; 4]) -> f32 {
    unsafe { ffi::lqr_step(state.as_ptr()) }
}

/// PID制御力を計算（カスケードPID: 角度+位置）
pub fn pid_compute(state: &[f32; 4]) -> f32 {
    unsafe { ffi::pid_step(state.as_ptr()) }
}

/// MPC制御力を計算（ADMM QPソルバ）
pub fn mpc_compute(state: &[f32; 4]) -> f32 {
    unsafe { ffi::mpc_step(state.as_ptr()) }
}

/// MRAC制御力を計算（適応制御）
pub fn mrac_compute(state: &[f32; 4]) -> f32 {
    unsafe { ffi::mrac_step(state.as_ptr()) }
}

/// 状態オブザーバ更新
/// 戻り値: 推定状態 [x, x_dot, theta, theta_dot]
pub fn observer_update(position: f32, theta: f32, force: f32) -> [f32; 4] {
    let mut x_hat = [0.0f32; 4];
    unsafe { ffi::observer_step(position, theta, force, x_hat.as_mut_ptr()); }
    x_hat
}
```

- [ ] **Step 5: Create `build.rs`**

```rust
use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let codegen_dir = manifest_dir.join("codegen");
    let matlab_dir = manifest_dir.parent().unwrap().join("MATLAB");

    // MATLAB設計ファイル変更時に再ビルド
    println!("cargo:rerun-if-changed={}",
        matlab_dir.join("control").join("codegen").display());
    println!("cargo:rerun-if-changed={}",
        matlab_dir.join("control").join("design").display());
    println!("cargo:rerun-if-changed={}",
        matlab_dir.join("plant").display());
    println!("cargo:rerun-if-changed={}",
        matlab_dir.join("build.m").display());

    // codegen/ が存在しないか空の場合、MATLABを起動してコード生成
    let needs_codegen = !codegen_dir.exists()
        || std::fs::read_dir(&codegen_dir)
            .map(|mut d| d.next().is_none())
            .unwrap_or(true);

    if needs_codegen {
        println!("cargo:warning=Running MATLAB code generation...");
        let status = Command::new("matlab")
            .args(["-batch", "cd MATLAB; build"])
            .current_dir(manifest_dir.parent().unwrap())
            .status()
            .expect("Failed to launch MATLAB. Is it installed and on PATH?");

        if !status.success() {
            panic!("MATLAB code generation failed with exit code: {:?}", status.code());
        }
    }

    // 生成されたCソースをコンパイル
    let c_files: Vec<PathBuf> = glob::glob(
        codegen_dir.join("**").join("*.c").to_str().unwrap()
    )
    .expect("Failed to glob codegen directory")
    .filter_map(|e| e.ok())
    .collect();

    if c_files.is_empty() {
        panic!("No C source files found in {:?}. Code generation may have failed.", codegen_dir);
    }

    let mut build = cc::Build::new();
    for f in &c_files {
        build.file(f);
    }

    // ヘッダーインクルードパス（MATLAB Coderは複数サブディレクトリに生成することがある）
    build.include(&codegen_dir);
    for entry in std::fs::read_dir(&codegen_dir).unwrap() {
        let entry = entry.unwrap();
        if entry.file_type().unwrap().is_dir() {
            build.include(entry.path());
        }
    }

    build
        .flag("-ffreestanding")
        .flag("-fno-builtin")
        .warnings(false)
        .compile("controller");
}
```

- [ ] **Step 6: Verify the crate structure**

```bash
ls -la inverted-pendulum-controller/
ls -la inverted-pendulum-controller/src/
cat inverted-pendulum-controller/Cargo.toml
```

Expected: All files present with correct content.

- [ ] **Step 7: Commit**

```bash
git add inverted-pendulum-controller/
git commit -m "feat: add inverted-pendulum-controller crate with FFI bindings and build.rs"
```

---

## Task 5: Verify FFI Signatures Against Generated Headers

After MATLAB code generation (Task 2 Step 3), verify that the `ffi.rs` declarations match the actual generated C headers. This step is critical because MATLAB Coder may use different naming conventions.

**Files:**
- Modify: `inverted-pendulum-controller/src/ffi.rs` (if needed)

- [ ] **Step 1: Inspect generated headers**

```bash
find inverted-pendulum-controller/codegen -name "*.h" -exec echo "=== {} ===" \; -exec cat {} \;
```

Expected: Header files with function declarations. Common MATLAB Coder patterns:
- `lqr_step_initialize()` (not `lqr_step_init()`)
- `lqr_step_terminate()` (cleanup, may not be needed for `no_std`)
- `lqr_step(const float state[4])` → returns `float`

- [ ] **Step 2: Update `ffi.rs` to match actual headers**

Compare each function signature in the generated `.h` files against `ffi.rs`. Update names, parameter types, and return types to match exactly. For example, if MATLAB Coder generates `lqr_step_initialize` instead of `lqr_step_init`:

```rust
extern "C" {
    // Update names to match actual generated headers
    pub fn lqr_step_initialize();  // was: lqr_step_init
    pub fn lqr_step(state: *const f32) -> f32;
    // ... etc
}
```

Also update `lib.rs` init functions accordingly:

```rust
pub fn init_lqr() {
    unsafe { ffi::lqr_step_initialize(); }  // match actual name
}
```

- [ ] **Step 3: Build the controller crate**

```bash
cd inverted-pendulum-controller && cargo build
```

Expected: Builds successfully. If link errors occur, the FFI signatures don't match — re-check headers.

- [ ] **Step 4: Commit if changes were made**

```bash
cd inverted-pendulum-controller
git add src/ffi.rs src/lib.rs
git commit -m "fix: align FFI signatures with MATLAB Coder generated headers"
```

---

## Task 6: Integrate Controller Crate into STM32 Firmware

Replace the hand-written controller implementations with calls to the generated controller crate.

**Files:**
- Modify: `inverted-pendulum-stm32/Cargo.toml`
- Modify: `inverted-pendulum-stm32/src/controller/mod.rs`
- Modify: `inverted-pendulum-stm32/src/tasks/mod.rs`
- Modify: `inverted-pendulum-stm32/src/main.rs`
- Delete: `inverted-pendulum-stm32/src/controller/lqr.rs`
- Delete: `inverted-pendulum-stm32/src/controller/lqr_constants.rs`
- Delete: `inverted-pendulum-stm32/src/controller/pid_balance.rs`
- Delete: `inverted-pendulum-stm32/src/controller/pid_balance_constants.rs`
- Delete: `inverted-pendulum-stm32/src/controller/mpc.rs`
- Delete: `inverted-pendulum-stm32/src/controller/mpc_constants.rs`
- Delete: `inverted-pendulum-stm32/src/controller/mrac.rs`
- Delete: `inverted-pendulum-stm32/src/controller/mrac_constants.rs`
- Delete: `inverted-pendulum-stm32/src/controller/observer.rs`
- Delete: `inverted-pendulum-stm32/src/controller/observer_constants.rs`
- Delete: `inverted-pendulum-stm32/src/tasks/mpc.rs`

- [ ] **Step 1: Add dependency to Cargo.toml**

Add to `inverted-pendulum-stm32/Cargo.toml` under `[dependencies]`:

```toml
inverted-pendulum-controller = { path = "../inverted-pendulum-controller" }
```

- [ ] **Step 2: Delete old controller files**

```bash
cd inverted-pendulum-stm32/src/controller
rm lqr.rs lqr_constants.rs
rm pid_balance.rs pid_balance_constants.rs
rm mpc.rs mpc_constants.rs
rm mrac.rs mrac_constants.rs
rm observer.rs observer_constants.rs
```

- [ ] **Step 3: Delete MPC async task**

```bash
rm inverted-pendulum-stm32/src/tasks/mpc.rs
```

- [ ] **Step 4: Update `tasks/mod.rs`**

Replace the full content of `inverted-pendulum-stm32/src/tasks/mod.rs`:

```rust
pub mod control;
pub mod encoder;
pub mod uart;
```

(Remove `pub mod mpc;`)

- [ ] **Step 5: Rewrite `controller/mod.rs`**

Replace the full content of `inverted-pendulum-stm32/src/controller/mod.rs`:

```rust
mod filter;
mod pid;

use crate::config::{
    clamp, BALANCE_DT, CURRENT_DT, FORCE_TO_CURRENT, MAX_FORCE, MAX_VOLTAGE,
};
use filter::LowPassFilter;
use pid::Pid;

use inverted_pendulum_controller as ctrl;

// フィルタカットオフ周波数
const THETA_FILTER_CUTOFF: f32 = 50.0; // [Hz]
const THETA_DOT_FILTER_CUTOFF: f32 = 25.0; // [Hz]
const CURRENT_FILTER_CUTOFF: f32 = 500.0; // [Hz]

// 電流PID
const CURRENT_PID_KP: f32 = 0.928;
const CURRENT_PID_KI: f32 = 10178.8;
const CURRENT_PID_KD: f32 = 0.0;

/// センサー状態（制御ループへの入力）
pub struct State {
    pub theta: f32,      // 振り子角度 [rad]
    pub position_r: f32, // 右車輪位置 [m]
    pub position_l: f32, // 左車輪位置 [m]
    pub velocity_r: f32, // 右車輪速度 [m/s]
    pub velocity_l: f32, // 左車輪速度 [m/s]
    pub current_r: f32,  // 右モーター電流 [A]
    pub current_l: f32,  // 左モーター電流 [A]
    pub vin: f32,        // バッテリー電圧 [V]
}

/// モーター出力デューティ比
pub struct MotorOutput {
    pub left: f32,
    pub right: f32,
}

pub use inverted_pendulum_protocol::ControlMode;

/// 統合制御システム
/// 共通の前処理・電流制御パイプラインと、モード切り替え可能な力計算を統合
pub struct ControlSystem {
    mode: ControlMode,

    // 状態推定
    theta_filter: LowPassFilter,
    theta_dot_filter: LowPassFilter,
    prev_theta: f32,
    theta_initialized: bool,

    // 電流制御（共通パイプライン）
    target_current: f32,
    target_voltage: f32,
    pid_l: Pid,
    pid_r: Pid,
    current_filter_l: LowPassFilter,
    current_filter_r: LowPassFilter,
    prev_voltage_l: f32,
    prev_voltage_r: f32,
}

impl ControlSystem {
    pub fn new() -> Self {
        ctrl::init_lqr();
        ctrl::init_pid();
        ctrl::init_mpc();
        ctrl::init_mrac();
        ctrl::init_observer();

        Self {
            mode: ControlMode::Lqr,
            theta_filter: LowPassFilter::new(BALANCE_DT, THETA_FILTER_CUTOFF),
            theta_dot_filter: LowPassFilter::new(BALANCE_DT, THETA_DOT_FILTER_CUTOFF),
            prev_theta: 0.0,
            theta_initialized: false,
            target_current: 0.0,
            target_voltage: 0.0,
            pid_l: Pid::new(
                CURRENT_PID_KP,
                CURRENT_PID_KI,
                CURRENT_PID_KD,
                CURRENT_DT,
                -MAX_VOLTAGE,
                MAX_VOLTAGE,
            ),
            pid_r: Pid::new(
                CURRENT_PID_KP,
                CURRENT_PID_KI,
                CURRENT_PID_KD,
                CURRENT_DT,
                -MAX_VOLTAGE,
                MAX_VOLTAGE,
            ),
            current_filter_l: LowPassFilter::new(CURRENT_DT, CURRENT_FILTER_CUTOFF),
            current_filter_r: LowPassFilter::new(CURRENT_DT, CURRENT_FILTER_CUTOFF),
            prev_voltage_l: 0.0,
            prev_voltage_r: 0.0,
        }
    }

    pub fn set_mode(&mut self, mode: ControlMode) {
        if self.mode != mode {
            self.mode = mode;
            match mode {
                ControlMode::Debug => {}
                ControlMode::Lqr => ctrl::init_lqr(),
                ControlMode::Pid => ctrl::init_pid(),
                ControlMode::Mrac => ctrl::init_mrac(),
                ControlMode::Mpc => ctrl::init_mpc(),
            }
            ctrl::init_observer();
        }
    }

    /// 振り子制御ループ (1kHz): 状態 → 力 → 電流目標値を更新
    pub fn update_balance(&mut self, state: &State) {
        // 前処理: 左右平均
        let position = (state.position_r + state.position_l) / 2.0;
        let theta = self.theta_filter.update(state.theta);
        let theta_dot_raw = if self.theta_initialized {
            (theta - self.prev_theta) / BALANCE_DT
        } else {
            self.theta_initialized = true;
            0.0
        };
        self.prev_theta = theta;
        let _theta_dot = self.theta_dot_filter.update(theta_dot_raw);

        // オブザーバで状態推定
        // 注意: オブザーバのBd行列は力入力で設計されているため、
        // target_currentを力に逆変換して渡す（MATLAB simulationと一致）
        let prev_force = self.target_current / FORCE_TO_CURRENT;
        let x_hat = ctrl::observer_update(position, theta, prev_force);

        // モード依存: 力の計算
        let force = match self.mode {
            ControlMode::Debug => {
                self.target_voltage = 0.0;
                0.0
            }
            ControlMode::Lqr => {
                self.target_voltage = 0.0;
                ctrl::lqr_compute(&x_hat)
            }
            ControlMode::Pid => {
                self.target_voltage = 0.0;
                ctrl::pid_compute(&x_hat)
            }
            ControlMode::Mrac => {
                self.target_voltage = 0.0;
                ctrl::mrac_compute(&x_hat)
            }
            ControlMode::Mpc => {
                self.target_voltage = 0.0;
                ctrl::mpc_compute(&x_hat)
            }
        };

        self.target_current = clamp(force, -MAX_FORCE, MAX_FORCE) * FORCE_TO_CURRENT;
    }

    /// 電流制御ループ (5kHz): 電流PID → デューティ比
    pub fn update_current(&mut self, state: &State) -> MotorOutput {
        let vin = if state.vin > 1.0 { state.vin } else { 7.2 };

        let corrected_l = correct_current_sign(state.current_l, self.prev_voltage_l);
        let corrected_r = correct_current_sign(state.current_r, self.prev_voltage_r);

        let filtered_l = self.current_filter_l.update(corrected_l);
        let filtered_r = self.current_filter_r.update(corrected_r);

        let voltage_l = self.pid_l.update(self.target_current, filtered_l);
        let voltage_r = self.pid_r.update(self.target_current, filtered_r);

        self.prev_voltage_l = voltage_l;
        self.prev_voltage_r = voltage_r;

        let duty_l = clamp(voltage_l / vin, -1.0, 1.0);
        let duty_r = clamp(voltage_r / vin, -1.0, 1.0);

        MotorOutput {
            left: duty_l,
            right: duty_r,
        }
    }

    pub fn reset(&mut self) {
        self.theta_filter.reset();
        self.theta_dot_filter.reset();
        self.prev_theta = 0.0;
        self.theta_initialized = false;
        ctrl::init_observer();
        self.target_current = 0.0;
        self.target_voltage = 0.0;
        self.pid_l.reset();
        self.pid_r.reset();
        self.current_filter_l.reset();
        self.current_filter_r.reset();
        self.prev_voltage_l = 0.0;
        self.prev_voltage_r = 0.0;
        match self.mode {
            ControlMode::Debug => {}
            ControlMode::Lqr => ctrl::init_lqr(),
            ControlMode::Pid => ctrl::init_pid(),
            ControlMode::Mrac => ctrl::init_mrac(),
            ControlMode::Mpc => ctrl::init_mpc(),
        }
    }
}

/// 電流センサーの符号補正
fn correct_current_sign(measured_current: f32, voltage_command: f32) -> f32 {
    const DEADBAND: f32 = 0.5;
    let abs_current = measured_current.abs();
    if voltage_command.abs() < DEADBAND {
        abs_current * (voltage_command / DEADBAND)
    } else if voltage_command > 0.0 {
        abs_current
    } else {
        -abs_current
    }
}
```

- [ ] **Step 6: Update `main.rs` — remove MPC task spawn**

In `inverted-pendulum-stm32/src/main.rs`, remove line 163:
```rust
// DELETE this line:
spawner.spawn(tasks::mpc::mpc_task().unwrap());
```

Also remove the `use` of `tasks::mpc` if present.

- [ ] **Step 7: Build the STM32 firmware**

```bash
cd inverted-pendulum-stm32 && cargo build
```

Expected: Compiles successfully with the controller crate providing all controller functions.

- [ ] **Step 8: Commit**

```bash
cd inverted-pendulum-stm32
git add -A
git commit -m "refactor(stm32): replace hand-written controllers with MATLAB Coder generated FFI"
```

---

## Task 7: Update MATLAB Simulation to Use MEX

Modify `FirmwareExperiment.m` to call MEX-generated controllers instead of the MATLAB step functions.

**Files:**
- Modify: `MATLAB/sim/FirmwareExperiment.m`
- Create: `MATLAB/sim/mex/.gitignore`

- [ ] **Step 1: Create `MATLAB/sim/mex/.gitignore`**

```
# MEX build artifacts
*.mex*
*.mexa64
*.mexw64
*.mexmaci64
*.mexmaca64
codegen/
```

- [ ] **Step 2: Update `FirmwareExperiment.m` balance loop**

In `step_balance_loop` (line 220), replace the mode-specific controller calls to use MEX functions. The MEX functions have the same interface as the entry points but with `_mex` suffix.

Replace lines 252-327 (the `switch obj.mode_name` block inside `step_balance_loop`) with:

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
                    x_ctrl = single([position; observer_velocity; theta; observer_theta_dot]);
                    force = double(pid_step_mex(x_ctrl));
                    runtime.target_force = util.clamp(force, -obj.options.max_force, obj.options.max_force);
                    runtime.target_current = runtime.target_force * FirmwareExperiment.force_to_current(obj.p);
                    runtime.target_voltage = 0.0;
                    velocity_est = observer_velocity;
                    theta_dot_est = observer_theta_dot;
                    current_used = NaN;
                    current_state = NaN;

                case 'lqr'
                    x_ctrl = single([position; observer_velocity; theta; observer_theta_dot]);
                    force = double(lqr_step_mex(x_ctrl));
                    runtime.target_force = util.clamp(force, -obj.options.max_force, obj.options.max_force);
                    runtime.target_current = runtime.target_force * FirmwareExperiment.force_to_current(obj.p);
                    runtime.target_voltage = 0.0;
                    velocity_est = observer_velocity;
                    theta_dot_est = observer_theta_dot;
                    current_used = NaN;
                    current_state = NaN;

                case 'mrac'
                    x_ctrl = single([position; observer_velocity; theta; observer_theta_dot]);
                    force = double(mrac_step_mex(x_ctrl));
                    runtime.target_force = util.clamp(force, -obj.options.max_force, obj.options.max_force);
                    runtime.target_current = runtime.target_force * FirmwareExperiment.force_to_current(obj.p);
                    runtime.target_voltage = 0.0;
                    velocity_est = observer_velocity;
                    theta_dot_est = observer_theta_dot;
                    current_used = NaN;
                    current_state = NaN;

                case 'mpc'
                    x_ctrl = single([position; observer_velocity; theta; observer_theta_dot]);
                    if runtime.mpc_counter == 0
                        force = double(mpc_step_mex(x_ctrl));
                        runtime.mpc_force = util.clamp(force, -obj.options.max_force, obj.options.max_force);
                    end
                    runtime.target_force = runtime.mpc_force;
                    runtime.target_current = runtime.target_force * FirmwareExperiment.force_to_current(obj.p);
                    runtime.target_voltage = 0.0;
                    velocity_est = observer_velocity;
                    theta_dot_est = observer_theta_dot;
                    current_used = NaN;
                    current_state = NaN;
                    runtime.mpc_counter = mod(runtime.mpc_counter + 1, runtime.mpc_decimation);

                otherwise
                    error('FirmwareExperiment:UnsupportedMode', 'Unsupported mode: %s', obj.mode_name);
            end
```

Also update `init_runtime` (lines 143-159): remove PID/MRAC controller initialization since MEX functions manage their own persistent state. Replace with MEX init calls:

```matlab
            % コード生成された制御器の初期化（persistent変数リセット）
            switch obj.mode_name
                case 'pid'
                    pid_step_mex('reset');  % or call _init MEX if separate
                case 'mrac'
                    mrac_step_mex('reset');
                case 'mpc'
                    mpc_step_mex('reset');
                    runtime.mpc_counter = 0;
                    runtime.mpc_force = 0.0;
                    runtime.mpc_decimation = 1;
                    if isstruct(obj.ctrl_param) && isfield(obj.ctrl_param, 'Ts') && obj.ctrl_param.Ts > 0.0
                        runtime.mpc_decimation = max(1, round(obj.ctrl_param.Ts / balance_dt));
                    end
            end
```

**Note:** The exact MEX initialization mechanism depends on how MATLAB Coder generates the init functions. The MEX wrapper may need a separate `*_initialize` call, or it may be handled by calling the step function with a special flag. This must be verified after code generation in Task 2. The observer initialization also needs to use `observer_step_mex` with the corresponding init call.

Also update the observer call at line 246 to use MEX:

```matlab
            % オブザーバ（MEX経由）
            x_hat = observer_step_mex(single(position), single(theta), single(runtime.target_current));
            observer_velocity = double(x_hat(2));
            observer_theta_dot = double(x_hat(4));
```

- [ ] **Step 3: Add MEX directory to path in `main.m`**

In `MATLAB/main.m`, add after the existing `addpath` calls (around line 8):

```matlab
addpath(fullfile(rootDir, 'sim', 'mex'));
```

Also add it to `MATLAB/build.m` path setup.

- [ ] **Step 4: Run simulation with MEX controllers**

Run in MATLAB:
```matlab
cd MATLAB; build; main
```

Expected: Simulation runs with MEX-generated controllers. Results should be numerically very close to previous results (within single-precision tolerance).

- [ ] **Step 5: Commit**

```bash
git add MATLAB/sim/FirmwareExperiment.m MATLAB/sim/mex/.gitignore MATLAB/main.m MATLAB/build.m
git commit -m "refactor(matlab): use MEX-generated controllers in FirmwareExperiment simulation"
```

---

## Task 8: Cross-Compilation Verification

Verify the full toolchain works end-to-end: MATLAB generates C → build.rs compiles for ARM → STM32 firmware links correctly.

**Files:** (no new files)

- [ ] **Step 1: Clean build from scratch**

```bash
cd inverted-pendulum-controller && rm -rf codegen/ target/
cd ../inverted-pendulum-stm32 && rm -rf target/
```

- [ ] **Step 2: Trigger full build**

```bash
cd inverted-pendulum-stm32 && cargo build
```

Expected: build.rs detects missing codegen, invokes MATLAB, generates C code, compiles with `arm-none-eabi-gcc` via `cc` crate, links into STM32 firmware.

- [ ] **Step 3: Verify binary size**

```bash
arm-none-eabi-size target/thumbv7em-none-eabi/debug/inverted-pendulum-stm32
```

Expected: Binary fits within STM32F303K8 flash (64KB). The generated C code should be comparable in size to the hand-written Rust.

- [ ] **Step 4: Flash and test on hardware (if available)**

```bash
cd inverted-pendulum-stm32 && cargo run
```

Expected: Firmware boots, mode switching works, control response is correct.

- [ ] **Step 5: Commit any final adjustments**

```bash
git add -A
git commit -m "chore: verify end-to-end MATLAB Coder → Rust FFI toolchain"
```
