%% main.m - 倒立振子制御シミュレーション（ファームウェア準拠シミュレータ）
clear; close all; clc;

%% 0. パス設定
rootDir = fileparts(which(mfilename));
addpath(fullfile(rootDir, 'plant'));
addpath(genpath(fullfile(rootDir, 'control')));
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
mpc_obs = design_observer(A, B, C, 0.01);

mrac = design_mrac(p);
fprintf('MRAC nominal gain Kx:\n');
disp(mrac.Kx);

mpc = design_mpc(A, B);
controller_params = {pid_gains, K_lqr, mrac, mpc};

%% 5. ファームウェアシミュレーション（全モード）
x0 = [0; 0; 0.1; 0];   % 初期状態（約5.7度傾き）
t_end = 15;              % 15秒間

% 外乱: t=10sに台車へ5Nのインパルス（50ms間）
disturbance = struct('time', 10.0, 'duration', 0.05, 'force', 5.0);
sim_option_factory = sim_options();
sim_options = sim_option_factory.default_options(struct( ...
    'disturbance', disturbance, ...
    'observer', struct('Ad', obs.Ad, 'Bd', obs.Bd, 'Ld', obs.Ld), ...
    'mpc_observer', struct('Ad', mpc_obs.Ad, 'Bd', mpc_obs.Bd, 'Ld', mpc_obs.Ld)));

modes = {'pid', 'lqr', 'mrac', 'mpc'};
labels = {'PID', 'LQR', 'MRAC', 'MPC'};
colors = {'r', 'b', [0 0.6 0], [0.8 0.4 0]};
styles = {'--', '-', '-.', ':'};

results = cell(size(modes));
for i = 1:numel(modes)
    fprintf('Running %s firmware simulation...\n', labels{i});
    results{i} = simulate_firmware(p, x0, t_end, modes{i}, controller_params{i}, sim_options);
end

%% 6. 性能比較プロット
figure('Name', 'Controller Comparison (Firmware)', 'NumberTitle', 'off', ...
    'Position', [100, 100, 1000, 800]);

subplot_titles = {'Pendulum Angle', 'Cart Position', 'Control Input (Force)', 'Motor Current'};
subplot_ylabels = {'Angle [deg]', 'Position [m]', 'Force [N]', 'Current [A]'};

% 振子角度
subplot(4, 1, 1);
for i = 1:numel(results)
    plot(results{i}.t, rad2deg(results{i}.x(:,3)), ...
        'Color', colors{i}, 'LineStyle', styles{i}, 'LineWidth', 1.5);
    hold on;
end
ylabel(subplot_ylabels{1});
title(subplot_titles{1});
legend(labels, 'Location', 'best');
grid on;

% 台車位置
subplot(4, 1, 2);
for i = 1:numel(results)
    plot(results{i}.t, results{i}.x(:,1), ...
        'Color', colors{i}, 'LineStyle', styles{i}, 'LineWidth', 1.5);
    hold on;
end
ylabel(subplot_ylabels{2});
title(subplot_titles{2});
legend(labels, 'Location', 'best');
grid on;

% 制御入力（力）
subplot(4, 1, 3);
for i = 1:numel(results)
    plot(results{i}.t, results{i}.target_force, ...
        'Color', colors{i}, 'LineStyle', styles{i}, 'LineWidth', 1.0);
    hold on;
end
ylabel(subplot_ylabels{3});
title(subplot_titles{3});
legend(labels, 'Location', 'best');
grid on;

% モーター電流
subplot(4, 1, 4);
for i = 1:numel(results)
    plot(results{i}.t, results{i}.current, ...
        'Color', colors{i}, 'LineStyle', styles{i}, 'LineWidth', 1.0);
    hold on;
end
ylabel(subplot_ylabels{4});
xlabel('Time [s]');
title(subplot_titles{4});
legend(labels, 'Location', 'best');
grid on;

% 外乱タイミングの縦線を全サブプロットに追加
for sp = 1:4
    subplot(4, 1, sp);
    xline(disturbance.time, 'k--', 'Disturbance', 'LineWidth', 1.0, ...
        'LabelOrientation', 'horizontal', 'LabelHorizontalAlignment', 'left');
end

% 性能指標
fprintf('\n=== Performance Comparison (Firmware Simulator) ===\n');
for i = 1:numel(results)
    print_metrics(labels{i}, results{i});
end

%% 7. Robustness analysis with uncertain parameters
robust_options = struct( ...
    'enabled', true, ...
    'sample_count', 48, ...
    'seed', 42, ...
    'success_angle_deg', 20.0, ...
    'success_final_angle_deg', 2.0, ...
    'angle_plot_limit_deg', 35.0, ...
    'position_plot_limit_m', 0.8, ...
    'parallel', struct( ...
    'enabled', true, ...
    'auto_start_pool', true, ...
    'min_sample_count', 4, ...
    'preferred_pool', 'Processes'), ...
    'uncertainty', struct( ...
    'percentage', struct( ...
    'M', 30.0, ...
    'm', 30.0, ...
    'l', 20.0, ...
    'r', 10.0, ...
    'Kt', 20.0, ...
    'Ke', 20.0, ...
    'Ra', 20.0, ...
    'La', 20.0, ...
    'G', 10.0), ...
    'plus_minus', struct( ...
    'bx', 0.20)));

if robust_options.enabled
    run_robustness_analysis( ...
        p, x0, t_end, modes, labels, colors, results, controller_params, ...
        sim_options, disturbance, robust_options);
end

%% --- ローカル関数 ---
function print_metrics(name, result)
metrics = evaluate_response_metrics(result);

fprintf('\n[%s]\n', name);
fprintf('  Settling time (+-0.5 deg): %.3f s\n', metrics.settling_time);
fprintf('  Max angle:                 %.2f deg\n', metrics.max_angle_deg);
fprintf('  Max cart displacement:     %.3f m\n', metrics.max_cart_displacement);
end

function metrics = evaluate_response_metrics(result, success_angle_deg, success_final_angle_deg)
if nargin < 2
    success_angle_deg = 20.0;
end
if nargin < 3
    success_final_angle_deg = 2.0;
end

theta_deg = rad2deg(result.x(:, 3));
t = result.t;

last_outside_idx = find(abs(theta_deg) > 0.5, 1, 'last');
if isempty(last_outside_idx)
    ts = t(1);
elseif last_outside_idx < numel(t)
    ts = t(last_outside_idx + 1);
else
    ts = NaN;
end

metrics = struct( ...
    'settling_time', ts, ...
    'max_angle_deg', max(abs(theta_deg)), ...
    'final_angle_deg', abs(theta_deg(end)), ...
    'max_cart_displacement', max(abs(result.x(:, 1))), ...
    'is_stable', false);
metrics.is_stable = isfinite(metrics.settling_time) && ...
    metrics.max_angle_deg <= success_angle_deg && ...
    metrics.final_angle_deg <= success_final_angle_deg;
end

function run_robustness_analysis( ...
    p_nominal, x0, t_end, modes, labels, colors, nominal_results, controller_params, ...
    sim_options, disturbance, robust_options)

if exist('ureal', 'file') ~= 2
    warning('Robust Control Toolbox not found. Skipping uncertainty sweep.');
    return;
end

fprintf('\n=== Robustness Analysis (Robust Control Toolbox) ===\n');
print_uncertainty_definition(robust_options.uncertainty);

uncertain_params = create_uncertain_params(p_nominal, robust_options.uncertainty);
sampled_params = sample_uncertain_params( ...
    uncertain_params, p_nominal, robust_options.sample_count, robust_options.seed);

sample_count = numel(sampled_params);
robust_results = cell(numel(modes), sample_count);
robust_metrics = cell(numel(modes), sample_count);
parallel_cfg = resolve_parallel_config(robust_options, sample_count);

for i = 1:numel(modes)
    if parallel_cfg.use_parallel
        fprintf('Running uncertainty sweep for %s (%d samples, parfor/%d workers)...\n', ...
            labels{i}, sample_count, parallel_cfg.num_workers);
        [mode_results, mode_metrics] = run_uncertainty_sweep_parallel( ...
            sampled_params, x0, t_end, modes{i}, controller_params{i}, sim_options, ...
            robust_options.success_angle_deg, robust_options.success_final_angle_deg);
    else
        fprintf('Running uncertainty sweep for %s (%d samples, serial)...\n', ...
            labels{i}, sample_count);
        [mode_results, mode_metrics] = run_uncertainty_sweep_serial( ...
            sampled_params, x0, t_end, modes{i}, controller_params{i}, sim_options, ...
            robust_options.success_angle_deg, robust_options.success_final_angle_deg);
    end

    robust_results(i, :) = mode_results;
    robust_metrics(i, :) = mode_metrics;
end

plot_robust_response_figure( ...
    nominal_results, robust_results, labels, colors, disturbance, robust_options, 'theta');
plot_robust_response_figure( ...
    nominal_results, robust_results, labels, colors, disturbance, robust_options, 'position');
plot_robust_time_response_grid( ...
    nominal_results, robust_results, labels, colors, disturbance, robust_options);
print_robustness_summary(labels, robust_metrics, sampled_params, p_nominal);
end

function print_uncertainty_definition(uncertainty)
percentage_names = fieldnames(uncertainty.percentage);
if ~isempty(percentage_names)
    fprintf('Percentage uncertainty:\n');
    for i = 1:numel(percentage_names)
        name = percentage_names{i};
        fprintf('  %s: +-%.1f %%\n', name, uncertainty.percentage.(name));
    end
end

plus_minus_names = fieldnames(uncertainty.plus_minus);
if ~isempty(plus_minus_names)
    fprintf('Absolute uncertainty:\n');
    for i = 1:numel(plus_minus_names)
        name = plus_minus_names{i};
        fprintf('  %s: +/-%g\n', name, uncertainty.plus_minus.(name));
    end
end
end

function plot_robust_response_figure( ...
    nominal_results, robust_results, labels, colors, disturbance, robust_options, signal_name)

if strcmp(signal_name, 'theta')
    figure_name = 'Robustness Sweep - Pendulum Angle';
    y_label = 'Angle [deg]';
else
    figure_name = 'Robustness Sweep - Cart Position';
    y_label = 'Position [m]';
end

figure('Name', figure_name, 'NumberTitle', 'off', ...
    'Position', [120, 120, 1100, 750]);

for i = 1:numel(labels)
    subplot(2, 2, i);
    hold on;

    for k = 1:size(robust_results, 2)
        if strcmp(signal_name, 'theta')
            y = rad2deg(robust_results{i, k}.x(:, 3));
        else
            y = robust_results{i, k}.x(:, 1);
        end
        plot(robust_results{i, k}.t, y, 'Color', [0.80 0.80 0.80], 'LineWidth', 0.8);
    end

    if strcmp(signal_name, 'theta')
        nominal_y = rad2deg(nominal_results{i}.x(:, 3));
        title(sprintf('%s: Pendulum Angle', labels{i}));
        ylim([-robust_options.angle_plot_limit_deg, robust_options.angle_plot_limit_deg]);
    else
        nominal_y = nominal_results{i}.x(:, 1);
        title(sprintf('%s: Cart Position', labels{i}));
        ylim([-robust_options.position_plot_limit_m, robust_options.position_plot_limit_m]);
    end

    plot(nominal_results{i}.t, nominal_y, 'Color', colors{i}, 'LineWidth', 2.0);
    xline(disturbance.time, 'k--', 'LineWidth', 1.0);
    xlabel('Time [s]');
    ylabel(y_label);
    grid on;
end
end

function plot_robust_time_response_grid( ...
    nominal_results, robust_results, labels, colors, disturbance, robust_options)

figure('Name', 'Robustness Sweep - Time Responses', 'NumberTitle', 'off', ...
    'Position', [140, 80, 1500, 900]);
tiledlayout(numel(labels), 4, 'TileSpacing', 'compact', 'Padding', 'compact');

signal_names = {'theta', 'position', 'force', 'current'};
column_titles = {'Pendulum Angle', 'Cart Position', 'Control Force', 'Motor Current'};
y_labels = {'Angle [deg]', 'Position [m]', 'Force [N]', 'Current [A]'};

for i = 1:numel(labels)
    for j = 1:numel(signal_names)
        nexttile;
        hold on;

        for k = 1:size(robust_results, 2)
            y = extract_signal(robust_results{i, k}, signal_names{j});
            plot(robust_results{i, k}.t, y, 'Color', [0.85 0.85 0.85], 'LineWidth', 0.6);
        end

        nominal_y = extract_signal(nominal_results{i}, signal_names{j});
        plot(nominal_results{i}.t, nominal_y, 'Color', colors{i}, 'LineWidth', 1.6);
        xline(disturbance.time, 'k--', 'LineWidth', 0.8);
        grid on;

        if i == 1
            title(column_titles{j});
        end
        if j == 1
            ylabel(sprintf('%s\n%s', labels{i}, y_labels{j}));
        else
            ylabel(y_labels{j});
        end
        if i == numel(labels)
            xlabel('Time [s]');
        end

        if strcmp(signal_names{j}, 'theta')
            ylim([-robust_options.angle_plot_limit_deg, robust_options.angle_plot_limit_deg]);
        elseif strcmp(signal_names{j}, 'position')
            ylim([-robust_options.position_plot_limit_m, robust_options.position_plot_limit_m]);
        end
    end
end
end

function parallel_cfg = resolve_parallel_config(robust_options, sample_count)
parallel_cfg = struct('use_parallel', false, 'num_workers', 0);

if ~isfield(robust_options, 'parallel')
    return;
end

cfg = robust_options.parallel;
if ~cfg.enabled || sample_count < cfg.min_sample_count
    return;
end

if exist('parfor', 'builtin') ~= 5 && exist('parfor', 'file') ~= 2
    return;
end
if exist('gcp', 'file') ~= 2
    return;
end
if ~license('test', 'Distrib_Computing_Toolbox')
    return;
end

pool = gcp('nocreate');
if ~isempty(pool) && pool_is_thread_based(pool)
    delete(pool);
    pool = [];
end
if isempty(pool) && cfg.auto_start_pool
    pool = ensure_parallel_pool(cfg.preferred_pool);
end

if isempty(pool)
    return;
end

parallel_cfg.use_parallel = true;
parallel_cfg.num_workers = pool.NumWorkers;
end

function pool = ensure_parallel_pool(preferred_pool)
pool = gcp('nocreate');
if ~isempty(pool)
    if pool_is_thread_based(pool)
        delete(pool);
    else
        return;
    end
end

try
    if ~isempty(preferred_pool)
        pool = parpool(preferred_pool);
    else
        pool = parpool;
    end
catch first_error
    try
        pool = parpool('Processes');
    catch second_error
        try
            pool = parpool('local');
        catch third_error
            try
                pool = parpool;
            catch fourth_error
                warning('main:ParallelPoolFallback', '%s', sprintf( ...
                    ['Failed to start process-based parallel pool. Falling back to serial execution.\n' ...
                    '  %s\n' ...
                    '  %s\n' ...
                    '  %s\n' ...
                    '  %s'], ...
                    first_error.message, second_error.message, third_error.message, fourth_error.message));
                pool = [];
            end
        end
    end
end
end

function tf = pool_is_thread_based(pool)
tf = false;
if isempty(pool)
    return;
end

try
    profile_name = string(pool.Cluster.Profile);
    tf = contains(lower(profile_name), "thread");
catch
end

if tf
    return;
end

try
    cluster_type = string(pool.Cluster.Type);
    tf = contains(lower(cluster_type), "thread");
catch
end
end

function [mode_results, mode_metrics] = run_uncertainty_sweep_parallel( ...
    sampled_params, x0, t_end, mode, controller_param, sim_options, ...
    success_angle_deg, success_final_angle_deg)

sample_count = numel(sampled_params);
mode_results = cell(1, sample_count);
mode_metrics = cell(1, sample_count);

parfor k = 1:sample_count
    result_k = simulate_firmware( ...
        sampled_params{k}, x0, t_end, mode, controller_param, sim_options);
    metric_k = evaluate_response_metrics( ...
        result_k, success_angle_deg, success_final_angle_deg);

    mode_results{k} = result_k;
    mode_metrics{k} = metric_k;
end
end

function [mode_results, mode_metrics] = run_uncertainty_sweep_serial( ...
    sampled_params, x0, t_end, mode, controller_param, sim_options, ...
    success_angle_deg, success_final_angle_deg)

sample_count = numel(sampled_params);
mode_results = cell(1, sample_count);
mode_metrics = cell(1, sample_count);

for k = 1:sample_count
    result_k = simulate_firmware( ...
        sampled_params{k}, x0, t_end, mode, controller_param, sim_options);
    metric_k = evaluate_response_metrics( ...
        result_k, success_angle_deg, success_final_angle_deg);

    mode_results{k} = result_k;
    mode_metrics{k} = metric_k;
end
end

function print_robustness_summary(labels, robust_metrics, sampled_params, p_nominal)
fprintf('\n=== Robustness Summary ===\n');
for i = 1:numel(labels)
    metric_row = robust_metrics(i, :);
    stable = cellfun(@(m) m.is_stable, metric_row);
    max_angle = cellfun(@(m) m.max_angle_deg, metric_row);
    max_cart = cellfun(@(m) m.max_cart_displacement, metric_row);
    settling_time = cellfun(@(m) m.settling_time, metric_row);

    [worst_angle, worst_idx] = max(max_angle);
    stable_count = nnz(stable);
    finite_ts = settling_time(isfinite(settling_time));
    if isempty(finite_ts)
        worst_settling = NaN;
    else
        worst_settling = max(finite_ts);
    end

    fprintf('\n[%s]\n', labels{i});
    fprintf('  Stable samples:           %d / %d (%.1f %%)\n', ...
        stable_count, numel(metric_row), 100.0 * stable_count / numel(metric_row));
    fprintf('  Worst max angle:          %.2f deg\n', worst_angle);
    fprintf('  Worst max cart position:  %.3f m\n', max(max_cart));
    fprintf('  Worst settling time:      %.3f s\n', worst_settling);
    fprintf('  Worst-case sample params: %s\n', ...
        format_param_delta(sampled_params{worst_idx}, p_nominal));
end
end

function text = format_param_delta(sampled_param, nominal_param)
names = fieldnames(sampled_param);
parts = strings(0, 1);

for i = 1:numel(names)
    name = names{i};
    nominal_value = nominal_param.(name);
    sampled_value = sampled_param.(name);

    if ~isscalar(sampled_value) || ~isnumeric(sampled_value)
        continue;
    end

    if nominal_value ~= 0.0
        delta_percent = 100.0 * (sampled_value - nominal_value) / nominal_value;
        parts(end + 1) = sprintf('%s=%+.1f%%%%', name, delta_percent); %#ok<AGROW>
    elseif sampled_value ~= nominal_value
        parts(end + 1) = sprintf('%s=%+.4g', name, sampled_value - nominal_value); %#ok<AGROW>
    end
end

text = strjoin(cellstr(parts), ', ');
end

function y = extract_signal(result, signal_name)
switch signal_name
    case 'theta'
        y = rad2deg(result.x(:, 3));
    case 'position'
        y = result.x(:, 1);
    case 'force'
        y = result.target_force;
    case 'current'
        y = result.current;
    otherwise
        error('Unknown signal: %s', signal_name);
end
end
