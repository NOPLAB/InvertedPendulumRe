function codegen_controller(K_lqr, pid_gains, mpc_params, mrac_params, obs_params)
% CODEGEN_CONTROLLER Generate C library and MEX for all controllers.
%   K_lqr:       1x4 double — LQR gain from design_lqr()
%   pid_gains:   struct     — PID gains from design_pid()
%   mpc_params:  struct     — MPC parameters from design_mpc()
%   mrac_params: struct     — MRAC parameters from design_mrac()
%   obs_params:  struct     — Observer parameters from design_observer()

    %% 出力先パス（codegen は '..' を含むパスでエラーになるため解決済みパスを使用）
    root_dir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    repo_root = fileparts(root_dir);
    lib_output = fullfile(repo_root, 'inverted-pendulum-controller', 'codegen');
    mex_output = fullfile(root_dir, 'sim', 'mex');

    % 出力ディレクトリ作成
    if ~exist(lib_output, 'dir'), mkdir(lib_output); end
    if ~exist(mex_output, 'dir'), mkdir(mex_output); end

    [cfg_lib, cfg_mex] = codegen_config();

    %% 共通: single型の状態入力
    state_type = coder.typeof(single(0), [4 1]);

    fprintf('\n=== Controller Code Generation ===\n\n');

    %% 1. LQR
    fprintf('Generating LQR...\n');
    K_s = single(K_lqr);
    K_const = coder.Constant(K_s);
    codegen('lqr_step', '-args', {K_const, state_type}, '-config', cfg_lib, '-d', lib_output)
    codegen('lqr_step', '-args', {K_const, state_type}, '-config', cfg_mex, '-d', mex_output)
    fprintf('  LQR: done\n');

    %% 2. PID
    fprintf('Generating PID...\n');
    pid_s = to_single_pid(pid_gains);
    pid_const = coder.Constant(pid_s);
    dt_const = coder.Constant(single(0.001));  % 1kHz balance loop
    maxf_const = coder.Constant(single(10.0));
    codegen('pid_step', '-args', {pid_const, dt_const, maxf_const, state_type}, '-config', cfg_lib, '-d', lib_output)
    codegen('pid_step', '-args', {pid_const, dt_const, maxf_const, state_type}, '-config', cfg_mex, '-d', mex_output)
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
    mpc_args = {N_const, rho_const, iter_const, umin_const, umax_const, H_const, F_const, state_type};
    codegen('mpc_step', '-args', mpc_args, '-config', cfg_lib, '-d', lib_output)
    codegen('mpc_step', '-args', mpc_args, '-config', cfg_mex, '-d', mex_output)
    fprintf('  MPC: done\n');

    %% 4. MRAC
    fprintf('Generating MRAC...\n');
    mrac_s = to_single_mrac(mrac_params);
    mrac_const = coder.Constant(mrac_s);
    codegen('mrac_step', '-args', {mrac_const, dt_const, maxf_const, state_type}, '-config', cfg_lib, '-d', lib_output)
    codegen('mrac_step', '-args', {mrac_const, dt_const, maxf_const, state_type}, '-config', cfg_mex, '-d', mex_output)
    fprintf('  MRAC: done\n');

    %% 5. Observer
    fprintf('Generating Observer...\n');
    Ad_const = coder.Constant(single(obs_params.Ad));
    Bd_const = coder.Constant(single(obs_params.Bd));
    Ld_const = coder.Constant(single(obs_params.Ld));
    scalar_type = coder.typeof(single(0));
    obs_args = {Ad_const, Bd_const, Ld_const, scalar_type, scalar_type, scalar_type};
    codegen('observer_step', '-args', obs_args, '-config', cfg_lib, '-d', lib_output)
    codegen('observer_step', '-args', obs_args, '-config', cfg_mex, '-d', mex_output)
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
