# MPC Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a linear MPC controller with ADMM QP solver for the inverted pendulum, integrated into the existing firmware simulator framework.

**Architecture:** Condensed-form QP with offline precomputation of Hessian and its factorization. Online computation is ADMM iterations (matrix-vector multiply + clamp). MPC runs at 100Hz within the existing 5kHz simulation loop. Force-based control with existing current loop.

**Tech Stack:** MATLAB (control design + simulation), existing FirmwareExperiment framework

---

### Task 1: MPC Parameter Design (`control/design_mpc.m`)

**Files:**
- Create: `MATLAB/control/design_mpc.m`

- [ ] **Step 1: Create `design_mpc.m` with offline precomputation**

```matlab
function mpc = design_mpc(A, B)
% DESIGN_MPC MPC制御器のオフライン設計
%   線形化モデルを離散化し、condensed QP行列を事前計算する。
%   mpc: struct with fields Ad, Bd, Q, R, P, N, u_min, u_max,
%         H, H_inv_rho, S_x, S_u, Q_bar, R_bar, rho, max_iter

    %% パラメータ
    Ts = 0.01;          % 制御周期 100Hz
    N  = 20;            % 予測ホライズン
    Q  = diag([100, 0, 50, 100]);
    R  = 10;
    u_max = 10.0;       % 入力制約 [N]
    rho = 1.0;          % ADMMペナルティ
    max_iter = 50;      % ADMM最大反復

    nx = size(A, 1);
    nu = size(B, 2);

    %% 離散化 (ZOH)
    sys_c = ss(A, B, eye(nx), zeros(nx, nu));
    sys_d = c2d(sys_c, Ts, 'zoh');
    Ad = sys_d.A;
    Bd = sys_d.B;

    %% 終端コスト (DARE)
    [P, ~, ~] = dare(Ad, Bd, Q, R);

    %% Condensed form 行列構築
    % S_x (N*nx x nx): 状態予測行列 (Ad の累乗)
    % S_u (N*nx x N*nu): 入力→状態マッピング
    S_x = zeros(N * nx, nx);
    S_u = zeros(N * nx, N * nu);

    Ad_power = eye(nx);
    for k = 1:N
        Ad_power = Ad_power * Ad;
        row_range = (k-1)*nx + (1:nx);
        S_x(row_range, :) = Ad_power;

        for j = 1:k
            col_range = (j-1)*nu + (1:nu);
            power = k - j;
            if power == 0
                S_u(row_range, col_range) = Bd;
            else
                S_u(row_range, col_range) = Ad^power * Bd;
            end
        end
    end

    %% ブロック対角重み
    Q_bar = blkdiag(kron(eye(N-1), Q), P);
    R_bar = kron(eye(N), R);

    %% QP Hessian と勾配係数
    H = S_u' * Q_bar * S_u + R_bar;
    H = (H + H') / 2;  % 対称性を保証
    F = S_u' * Q_bar * S_x;  % f = F * x0

    %% ADMM事前計算: (H + rho*I)^{-1}
    H_inv_rho = inv(H + rho * eye(N * nu));

    %% 出力
    mpc.Ts = Ts;
    mpc.N = N;
    mpc.nx = nx;
    mpc.nu = nu;
    mpc.Ad = Ad;
    mpc.Bd = Bd;
    mpc.Q = Q;
    mpc.R = R;
    mpc.P = P;
    mpc.u_min = -u_max;
    mpc.u_max = u_max;
    mpc.rho = rho;
    mpc.max_iter = max_iter;
    mpc.H = H;
    mpc.F = F;
    mpc.H_inv_rho = H_inv_rho;

    fprintf('MPC designed: Ts=%.3fs, N=%d, horizon=%.0fms\n', Ts, N, Ts*N*1000);
    fprintf('  QP size: %d variables, box constraints\n', N * nu);
    fprintf('  ADMM: rho=%.1f, max_iter=%d\n', rho, max_iter);
end
```

- [ ] **Step 2: Verify design_mpc runs correctly**

Run in MATLAB:
```matlab
p = params();
[A, B, ~, ~] = linearize_system(p);
mpc = design_mpc(A, B);
fprintf('H condition number: %.2f\n', cond(mpc.H));
fprintf('P (terminal cost) max eigenvalue: %.2f\n', max(eig(mpc.P)));
```
Expected: H is positive definite, condition number reasonable (< 1e6), no errors.

- [ ] **Step 3: Commit**

```bash
git add MATLAB/control/design_mpc.m
git commit -m "feat(matlab): add MPC offline design with condensed QP and ADMM precomputation"
```

---

### Task 2: ADMM Solver and MPC Controller Step

**Files:**
- Create: `MATLAB/sim/mpc_controller.m`

- [ ] **Step 1: Create `mpc_controller.m` with ADMM solver and control interface**

```matlab
function [force, mpc_state] = mpc_controller(mpc_param, x_current, mpc_state)
% MPC_CONTROLLER ADMMベースのMPC制御ステップ
%   mpc_param: design_mpc() の出力
%   x_current: 現在の状態 [x; x_dot; theta; theta_dot]
%   mpc_state: 前回のADMM warm-start用状態 (初回は [] を渡す)
%
%   戻り値:
%     force: 最適制御入力 u_0 [N]
%     mpc_state: 次回のwarm-start用状態

    N = mpc_param.N;
    nu = mpc_param.nu;
    n_var = N * nu;

    %% warm-start 初期化
    if isempty(mpc_state)
        mpc_state.U = zeros(n_var, 1);
        mpc_state.Z = zeros(n_var, 1);
        mpc_state.W = zeros(n_var, 1);
    else
        % シフト warm-start: 前回の解を1ステップずらす
        mpc_state.U = [mpc_state.U(nu+1:end); mpc_state.U(end-nu+1:end)];
        mpc_state.Z = [mpc_state.Z(nu+1:end); mpc_state.Z(end-nu+1:end)];
        mpc_state.W = zeros(n_var, 1);
    end

    %% 勾配ベクトル計算 (オンライン部分)
    f = mpc_param.F * x_current;

    %% ADMM反復
    rho = mpc_param.rho;
    U = mpc_state.U;
    Z = mpc_state.Z;
    W = mpc_state.W;
    u_min = mpc_param.u_min;
    u_max = mpc_param.u_max;

    for iter = 1:mpc_param.max_iter
        % U更新: (H + rho*I)^{-1} * (-f + rho*(Z - W))
        U = mpc_param.H_inv_rho * (-f + rho * (Z - W));

        % Z更新: box制約への射影 (clamp)
        Z_old = Z;
        Z = min(max(U + W, u_min), u_max);

        % W更新: 双対変数
        W = W + U - Z;
    end

    %% 結果
    force = U(1);  % 最初の制御入力のみ適用 (receding horizon)
    mpc_state.U = U;
    mpc_state.Z = Z;
    mpc_state.W = W;
end
```

- [ ] **Step 2: Verify standalone MPC controller**

Run in MATLAB:
```matlab
p = params();
[A, B, ~, ~] = linearize_system(p);
mpc = design_mpc(A, B);
x0 = [0; 0; 0.1; 0];
[force, state] = mpc_controller(mpc, x0, []);
fprintf('MPC force for x0=[0,0,0.1,0]: %.4f N\n', force);
% LQR comparison
K = design_lqr(A, B);
fprintf('LQR force for same x0: %.4f N\n', -K * x0);
```
Expected: Both produce a negative force (pushing pendulum back upright). MPC force should be similar magnitude to LQR but potentially different due to horizon and constraints.

- [ ] **Step 3: Commit**

```bash
git add MATLAB/sim/mpc_controller.m
git commit -m "feat(matlab): add ADMM-based MPC controller with warm-start"
```

---

### Task 3: Integrate MPC into FirmwareExperiment

**Files:**
- Modify: `MATLAB/sim/FirmwareExperiment.m`

- [ ] **Step 1: Add 'mpc' mode to `normalize_mode`**

In the string switch block (around line 355), add after the 'mrac' case:

```matlab
                    case {'mode5', 'mpc'}
                        spec = struct('mode_number', 5, 'firmware_mode', 4, 'name', 'mpc');
```

In the numeric switch block (around line 378), add after case 4:

```matlab
                    case 5
                        spec = struct('mode_number', 5, 'firmware_mode', 4, 'name', 'mpc');
```

- [ ] **Step 2: Add default controller param for MPC**

In `default_controller_param` (around line 389), add after the 'mrac' case:

```matlab
                case 'mpc'
                    [A, B, ~, ~] = linearize_system(p);
                    ctrl_param = design_mpc(A, B);
```

- [ ] **Step 3: Add MPC runtime initialization in `init_runtime`**

In the switch block (around line 129), add after the 'mrac' case:

```matlab
                case 'mpc'
                    runtime.mpc_state = [];  % ADMM warm-start (初回は空)
```

- [ ] **Step 4: Add MPC balance loop frequency override**

In `default_options` (around line 406), the default `balance_loop_frequency` is 1000. We need to allow per-mode override. In the `run` method, after line 46 (`balance_decimation = round(balance_ratio);`), we don't change the framework — instead, we set the balance frequency to 100Hz when constructing the experiment for MPC mode.

In `init_runtime`, right before the mode switch, add MPC balance frequency logic. Actually, the cleaner approach: in the constructor, override options for MPC:

In the constructor (after line 33 `obj.options = ...`), add:

```matlab
            if strcmp(spec.name, 'mpc')
                obj.options.balance_loop_frequency = 100.0;
            end
```

- [ ] **Step 5: Add MPC case in `step_balance_loop`**

In the switch block (around line 229), add after the 'mrac' case and before 'otherwise':

```matlab
                case 'mpc'
                    x_ctrl = [position; observer_velocity; theta; observer_theta_dot];
                    [force, runtime.mpc_state] = mpc_controller(obj.ctrl_param, x_ctrl, runtime.mpc_state);
                    runtime.target_force = FirmwareExperiment.clamp(force, -obj.options.max_force, obj.options.max_force);
                    runtime.target_current = runtime.target_force * FirmwareExperiment.force_to_current(obj.p);
                    runtime.target_voltage = 0.0;
                    velocity_est = observer_velocity;
                    theta_dot_est = observer_theta_dot;
                    current_used = NaN;
                    current_state = NaN;
```

- [ ] **Step 6: Add MPC to current loop handling**

MPC uses force→current→voltage (same as LQR), so the existing `otherwise` branch in `step_current_loop` already handles it correctly. No changes needed.

- [ ] **Step 7: Verify MPC simulation runs**

Run in MATLAB:
```matlab
p = params();
x0 = [0; 0; 0.1; 0];
result = simulate_firmware(p, x0, 10, 'mpc');
fprintf('MPC settling: angle at 1s=%.2f deg, 5s=%.2f deg, 10s=%.4f deg\n', ...
    rad2deg(result.x(find(result.t>=1,1),3)), ...
    rad2deg(result.x(find(result.t>=5,1),3)), ...
    rad2deg(result.x(end,3)));
fprintf('Max angle: %.2f deg\n', max(abs(rad2deg(result.x(:,3)))));
fprintf('Max force: %.2f N\n', max(abs(result.target_force)));
```
Expected: Pendulum stabilizes (angle converges toward 0), max angle stays bounded.

- [ ] **Step 8: Commit**

```bash
git add MATLAB/sim/FirmwareExperiment.m
git commit -m "feat(matlab): integrate MPC mode into FirmwareExperiment simulator"
```

---

### Task 4: Add MPC to main.m comparison

**Files:**
- Modify: `MATLAB/main.m`

- [ ] **Step 1: Add MPC design call**

After the MRAC design (around line 45), add:

```matlab
mpc = design_mpc(A, B);
```

- [ ] **Step 2: Add 'mpc' to modes array**

Change the modes/labels/colors/styles arrays (around line 51-54) to:

```matlab
modes = {'pid', 'lqr', 'mrac', 'mpc'};
labels = {'PID', 'LQR', 'MRAC', 'MPC'};
colors = {'r', 'b', [0 0.6 0], [0.8 0.4 0]};
styles = {'--', '-', '-.', ':'};
```

- [ ] **Step 3: Handle MPC voltage display in control input plot**

In the control input subplot (around line 93), MPC uses force like LQR, so the existing else branch handles it. But update the condition to also check for 'mpc' using force:

The existing code checks `strcmp(modes{i}, 'mrac')` for voltage. MPC uses target_force, so the else branch already handles it correctly. No change needed.

- [ ] **Step 4: Run full comparison**

Run `main.m` and verify MPC appears in all plots alongside PID, LQR, and MRAC.

Expected: 4 controllers shown in comparison plots with MPC as orange dotted line.

- [ ] **Step 5: Commit**

```bash
git add MATLAB/main.m
git commit -m "feat(matlab): add MPC to controller comparison in main.m"
```

---

### Task 5: Tuning and Validation

**Files:**
- Modify: `MATLAB/control/design_mpc.m` (if tuning needed)

- [ ] **Step 1: Run full simulation and evaluate performance**

Run `main.m` and check MPC performance metrics:
- Settling time (±0.5 deg)
- Max angle overshoot
- Max cart displacement
- Control effort (max force)

Compare with LQR baseline:
- LQR settling: ~1.8s
- LQR max angle: 5.73 deg (initial condition)
- LQR max displacement: 0.116 m

- [ ] **Step 2: Tune Q/R weights if needed**

If MPC performance is unsatisfactory:
- Increase Q(3,3) (angle weight) for faster angle recovery
- Decrease R for more aggressive control
- Adjust N (horizon) if prediction is too short/long
- Verify constraint activity (is u_max being hit?)

- [ ] **Step 3: Verify ADMM convergence**

Add temporary convergence check in `mpc_controller.m`:
```matlab
% After ADMM loop, check primal residual
primal_res = norm(U - Z);
fprintf('ADMM primal residual: %.6e\n', primal_res);
```
Expected: residual < 1e-4 within 50 iterations for most timesteps.

- [ ] **Step 4: Final commit after tuning**

```bash
git add MATLAB/control/design_mpc.m MATLAB/sim/mpc_controller.m
git commit -m "chore(matlab): tune MPC parameters for optimal performance"
```
