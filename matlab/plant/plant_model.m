function dxdt = plant_model(~, x, u, p)
% PLANT_MODEL 台車型倒立振子の非線形運動方程式
%   状態: x = [x; x_dot; theta; theta_dot]
%   入力: u = F (台車への水平力)
%   theta = 0 が直立位置

    x_dot   = x(2);
    theta   = x(3);
    th_dot  = x(4);
    F       = u;

    sin_th = sin(theta);
    cos_th = cos(theta);

    M = p.M;
    m = p.m;
    l = p.l;
    g = p.g;
    b = p.b;

    % 分母
    D = M + m - m * cos_th^2;

    % 台車加速度
    x_ddot = (F - b * x_dot + m * l * th_dot^2 * sin_th - m * g * sin_th * cos_th) / D;

    % 振子角加速度
    th_ddot = ((M + m) * g * sin_th - cos_th * (F - b * x_dot + m * l * th_dot^2 * sin_th)) / (l * D);

    dxdt = [x_dot; x_ddot; th_dot; th_ddot];
end
