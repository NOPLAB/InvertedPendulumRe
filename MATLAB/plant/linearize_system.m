function [A, B, C, D] = linearize_system(p)
% LINEARIZE_SYSTEM 直立平衡点(theta=0)周りで線形化した状態空間モデル
%   状態: x = [x; x_dot; theta; theta_dot]
%   入力: u = F (台車への力)
%   出力: y = [x; theta]
%
%   慣性モーメント・ギア・タイヤ半径を考慮した詳細モデル
%   (cart_pendulum_system.m と同等)

    M  = p.M;
    m  = p.m;
    l  = p.l;
    r  = p.r;
    g  = p.g;
    Jp = p.Jp;
    Iw = p.Iw;
    bx = p.bx;
    btheta = p.btheta;

    % 中間変数
    r2 = r^2;
    l2 = l^2;

    % Jp + m*l^2
    Jml2 = Jp + m * l2;

    % 2*Iw + M*r^2 + m*r^2
    den_base = 2*Iw + M*r2 + m*r2;

    % 分母: Jp*(2*Iw + M*r^2 + m*r^2) + m*l^2*(2*Iw + M*r^2)
    % = Jp*(2*Iw) + Jp*(M*r^2) + Jp*(m*r^2) + m*l^2*(2*Iw) + m*l^2*(M*r^2)
    Delta = Jp * den_base + m * l2 * (2*Iw + M*r2);

    A = [0,  1,                          0,  0;
         0, -bx * r2 * Jml2 / Delta,    -g * m^2 * l2 * r2 / Delta,       btheta * l * m * r2 / Delta;
         0,  0,                          0,  1;
         0,  bx * l * m * r2 / Delta,    g * l * m * den_base / Delta,    -btheta * den_base / Delta];

    B = [0;
         r2 * Jml2 / Delta;
         0;
        -l * m * r2 / Delta];

    C = [1 0 0 0;
         0 0 1 0];

    D = zeros(2, 1);
end
