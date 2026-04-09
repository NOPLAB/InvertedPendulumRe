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
addpath(fullfile(rootDir, 'sim', 'mex'));

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
