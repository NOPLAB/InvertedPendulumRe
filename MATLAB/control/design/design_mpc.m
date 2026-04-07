function mpc = design_mpc(A, B)
% DESIGN_MPC MPC制御器のオフライン設計
%   線形化モデルを離散化し、condensed QP行列を事前計算する。
%   mpc: struct with fields Ad, Bd, Q, R, P, N, u_min, u_max,
%         H, H_inv_rho, S_x, S_u, Q_bar, R_bar, rho, max_iter

    %% パラメータ
    Ts = 0.01;          % 制御周期 100Hz
    N  = 10;            % 予測ホライズン
    Q  = diag([100, 0, 500, 100]);
    R  = 1;
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
