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
