%% test_baseline.m - Baseline regression test for firmware simulator
%   CAPTURE mode: if test_baseline.mat does not exist, run all simulations
%                 and save results as baseline.
%   VERIFY mode:  if test_baseline.mat exists, re-run simulations and
%                 compare against saved baseline. Report PASS/FAIL per mode.

clear; close all; clc;

%% 0. Path setup
rootDir = fileparts(which(mfilename));
addpath(fullfile(rootDir, 'plant'));
addpath(fullfile(rootDir, 'control'));
addpath(fullfile(rootDir, 'sim'));

%% 1. Simulation parameters
p = params();
x0 = [0; 0; 0.1; 0];
t_end = 15;
disturbance = struct('time', 10.0, 'duration', 0.05, 'force', 5.0);
sim_options = struct('disturbance', disturbance);

modes = {'pid', 'lqr', 'mrac', 'mpc'};

%% 2. Run all simulations
results = struct();
for i = 1:numel(modes)
    mode = modes{i};
    fprintf('Running %s simulation...\n', mode);
    r = simulate_firmware(p, x0, t_end, mode, [], sim_options);
    results.(mode).t = r.t;
    results.(mode).x = r.x;
    results.(mode).target_force = r.target_force;
    results.(mode).target_voltage = r.target_voltage;
    results.(mode).current = r.current;
end

%% 3. Capture or Verify
baseline_file = fullfile(rootDir, 'test_baseline.mat');
tol = 1e-10;

if ~isfile(baseline_file)
    %% CAPTURE mode
    baseline = results; %#ok<NASGU>
    save(baseline_file, 'baseline');
    fprintf('\n=== CAPTURE MODE ===\n');
    fprintf('Baseline saved to %s\n', baseline_file);
    for i = 1:numel(modes)
        mode = modes{i};
        fprintf('  %s: %d time steps captured\n', mode, numel(results.(mode).t));
    end
else
    %% VERIFY mode
    loaded = load(baseline_file, 'baseline');
    baseline = loaded.baseline;
    fprintf('\n=== VERIFY MODE (tolerance = %.0e) ===\n', tol);

    all_pass = true;
    for i = 1:numel(modes)
        mode = modes{i};

        % State error
        state_err = max(abs(results.(mode).x(:) - baseline.(mode).x(:)));

        % Force/voltage error
        force_err = max(abs(results.(mode).target_force(:) - baseline.(mode).target_force(:)));
        voltage_err = max(abs(results.(mode).target_voltage(:) - baseline.(mode).target_voltage(:)));
        ctrl_err = max(force_err, voltage_err);

        pass = (state_err <= tol) && (ctrl_err <= tol);

        if pass
            status = 'PASS';
        else
            status = 'FAIL';
            all_pass = false;
        end

        fprintf('  %s: %s  (max state err = %.2e, max force/voltage err = %.2e)\n', ...
            upper(mode), status, state_err, ctrl_err);
    end

    fprintf('\n');
    if all_pass
        fprintf('All modes PASSED.\n');
    else
        fprintf('Some modes FAILED!\n');
    end
end
