%% main.m - 倒立振子制御シミュレーション（LQR vs PID）
clear; close all; clc;

%% 1. パラメータ読込
p = params();

%% 2. 線形化
[A, B, C, D] = linearize_system(p);
fprintf('System linearized around upright equilibrium.\n');
fprintf('Eigenvalues of A:\n');
disp(eig(A));

%% 3. 可制御性チェック
Co = ctrb(A, B);
if rank(Co) == size(A, 1)
    fprintf('System is controllable.\n\n');
else
    error('System is NOT controllable!');
end

%% 4. 制御器設計
K_lqr = design_lqr(A, B);
fprintf('LQR gain K:\n');
disp(K_lqr);

pid_gains = design_pid();
fprintf('PID gains (theta): Kp=%.1f, Ki=%.1f, Kd=%.1f\n', ...
    pid_gains.theta.Kp, pid_gains.theta.Ki, pid_gains.theta.Kd);
fprintf('PID gains (x):     Kp=%.1f, Ki=%.1f, Kd=%.1f\n\n', ...
    pid_gains.x.Kp, pid_gains.x.Ki, pid_gains.x.Kd);

%% 5. シミュレーション
x0 = [0; 0; 0.1; 0];   % 初期状態（約5.7度傾き）
tspan = [0, 10];         % 10秒間

fprintf('Running LQR simulation...\n');
[t_lqr, x_lqr, u_lqr] = simulate_nonlinear('lqr', p, x0, tspan, K_lqr);

fprintf('Running PID simulation...\n');
[t_pid, x_pid, u_pid] = simulate_nonlinear('pid', p, x0, tspan, pid_gains);

%% 6. 性能比較プロット
compare_controllers(t_lqr, x_lqr, u_lqr, t_pid, x_pid, u_pid);

%% 7. アニメーション（LQR）
fprintf('\nShowing LQR animation...\n');
animate_pendulum(t_lqr, x_lqr, p);
