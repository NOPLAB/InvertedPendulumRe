function run_simulink_model()
% RUN_SIMULINK_MODEL Simulink版 倒立振子制御モデルの実行
%   LQR と PID の両方でシミュレーションし、結果を比較プロットする

    modelName = 'InvertedPendulum_Simulink';

    % パス設定
    scriptDir = fileparts(mfilename('fullpath'));
    rootDir   = fileparts(scriptDir);
    addpath(fullfile(rootDir, 'plant'));
    addpath(fullfile(rootDir, 'control'));
    addpath(fullfile(rootDir, 'sim'));

    % === 1. パラメータ準備 ===
    p = params();
    [A, B, ~, ~] = linearize_system(p);
    K_lqr = design_lqr(A, B);

    % 初期条件: theta = 0.1 rad (~5.7°)
    x0 = [0; 0; 0.1; 0];

    % ワークスペースに変数を設定
    assignin('base', 'x0', x0);
    assignin('base', 'K_lqr', K_lqr);

    fprintf('K_lqr = [%.4f, %.4f, %.4f, %.4f]\n', K_lqr);
    fprintf('x0 = [%.1f, %.1f, %.1f, %.1f]\n', x0);

    % === 2. モデルを毎回再生成して、設計パラメータとの不整合を防ぐ ===
    fprintf('Simulink モデルを再生成します...\n');
    build_simulink_model();

    % モデルを読み込み
    if ~bdIsLoaded(modelName)
        load_system(modelName);
    end

    % === 3. LQR モードでシミュレーション ===
    fprintf('\n--- LQR シミュレーション ---\n');
    switchBlock = [modelName '/Controller_Switch'];
    set_param(switchBlock, 'sw', '0');  % 上側 = LQR

    simOut_lqr = sim(modelName);
    t_lqr   = simOut_lqr.tout;
    x_lqr   = simOut_lqr.get('x_out');
    u_lqr   = simOut_lqr.get('u_out');

    fprintf('LQR 完了: %d ステップ\n', length(t_lqr));

    % === 4. PID モードでシミュレーション ===
    fprintf('\n--- PID シミュレーション ---\n');
    set_param(switchBlock, 'sw', '1');  % 下側 = PID

    simOut_pid = sim(modelName);
    t_pid   = simOut_pid.tout;
    x_pid   = simOut_pid.get('x_out');
    u_pid   = simOut_pid.get('u_out');

    fprintf('PID 完了: %d ステップ\n', length(t_pid));

    % === 5. 比較プロット ===
    compare_controllers(t_lqr, x_lqr, u_lqr, t_pid, x_pid, u_pid);

    fprintf('\nSimulink シミュレーション完了。\n');
end
