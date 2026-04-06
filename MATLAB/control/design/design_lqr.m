function K = design_lqr(A, B)
% DESIGN_LQR LQR制御器の設計
%   K: 状態フィードバックゲイン (u = -K*x)

    % 重み行列 (旧プロジェクト design_feedback_controller.m 準拠)
    Q = diag([100, 0, 50, 100]);  % [x位置, x速度, 角度, 角速度]
    R = 10;                        % 制御入力コスト

    K = lqr(A, B, Q, R);
end
