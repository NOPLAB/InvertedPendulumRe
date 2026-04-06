%% main.m - 倒立振子制御シミュレーション（ファームウェア準拠シミュレータ）
clear; close all; clc;

%% 0. パス設定
rootDir = fileparts(which(mfilename));
addpath(fullfile(rootDir, 'plant'));
addpath(fullfile(rootDir, 'control'));
addpath(fullfile(rootDir, 'sim'));
addpath(fullfile(rootDir, 'simulink'));

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

obs = design_observer(A, B, C);
fprintf('Observer poles (continuous): [%s]\n', num2str(obs.continuous_poles', '%.1f '));
fprintf('Observer poles (discrete):   [%s]\n\n', num2str(obs.discrete_poles', '%.6f '));

mrac = design_mrac(p);
fprintf('MRAC nominal gain Kx:\n');
disp(mrac.Kx);

%% 5. ファームウェアシミュレーション（全モード）
x0 = [0; 0; 0.1; 0];   % 初期状態（約5.7度傾き）
t_end = 10;              % 10秒間

modes = {'pid', 'lqr', 'mrac'};
labels = {'PID', 'LQR', 'MRAC'};
colors = {'r', 'b', [0 0.6 0]};
styles = {'--', '-', '-.'};

results = cell(size(modes));
for i = 1:numel(modes)
    fprintf('Running %s firmware simulation...\n', labels{i});
    results{i} = simulate_firmware(p, x0, t_end, modes{i});
end

%% 6. 性能比較プロット
figure('Name', 'Controller Comparison (Firmware)', 'NumberTitle', 'off', ...
    'Position', [100, 100, 1000, 800]);

% 振子角度
subplot(4, 1, 1);
for i = 1:numel(results)
    plot(results{i}.t, rad2deg(results{i}.x(:,3)), ...
        'Color', colors{i}, 'LineStyle', styles{i}, 'LineWidth', 1.5);
    hold on;
end
ylabel('Angle [deg]');
title('Pendulum Angle');
legend(labels, 'Location', 'best');
grid on;

% 台車位置
subplot(4, 1, 2);
for i = 1:numel(results)
    plot(results{i}.t, results{i}.x(:,1), ...
        'Color', colors{i}, 'LineStyle', styles{i}, 'LineWidth', 1.5);
    hold on;
end
ylabel('Position [m]');
title('Cart Position');
legend(labels, 'Location', 'best');
grid on;

% 制御入力（力 / 電圧）
subplot(4, 1, 3);
for i = 1:numel(results)
    if strcmp(modes{i}, 'mrac')
        plot(results{i}.t, results{i}.target_voltage, ...
            'Color', colors{i}, 'LineStyle', styles{i}, 'LineWidth', 1.0);
    else
        plot(results{i}.t, results{i}.target_force, ...
            'Color', colors{i}, 'LineStyle', styles{i}, 'LineWidth', 1.0);
    end
    hold on;
end
ylabel('Force [N] / Voltage [V]');
title('Control Input (Force or Voltage)');
legend(labels, 'Location', 'best');
grid on;

% モーター電流
subplot(4, 1, 4);
for i = 1:numel(results)
    plot(results{i}.t, results{i}.current, ...
        'Color', colors{i}, 'LineStyle', styles{i}, 'LineWidth', 1.0);
    hold on;
end
ylabel('Current [A]');
xlabel('Time [s]');
title('Motor Current');
legend(labels, 'Location', 'best');
grid on;

% 性能指標
fprintf('\n=== Performance Comparison (Firmware Simulator) ===\n');
for i = 1:numel(results)
    print_metrics(labels{i}, results{i});
end

%% --- ローカル関数 ---
function print_metrics(name, result)
theta_deg = rad2deg(result.x(:, 3));
t = result.t;

% 整定時間（角度が±0.5度以内に収まる最初の時刻）
settled = abs(theta_deg) < 0.5;
idx = find(settled, 1, 'first');
if ~isempty(idx) && all(settled(idx:end))
    ts = t(idx);
else
    ts = NaN;
end

% 最大オーバーシュート
overshoot = max(abs(theta_deg));

fprintf('\n[%s]\n', name);
fprintf('  Settling time (+-0.5 deg): %.3f s\n', ts);
fprintf('  Max angle:                 %.2f deg\n', overshoot);
fprintf('  Max cart displacement:     %.3f m\n', max(abs(result.x(:,1))));
end
