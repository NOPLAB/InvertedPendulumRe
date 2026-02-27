function K = design_lqr(A, B)
% DESIGN_LQR LQR制御器の設計
%   K: 状態フィードバックゲイン (u = -K*x)

    % 重み行列
    Q = diag([1, 1, 10, 1]);  % 角度θを重視
    R = 0.01;                  % 制御入力コスト

    K = lqr(A, B, Q, R);
end
