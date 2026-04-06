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
