function [A, B, C, D] = linearize_system(p)
% LINEARIZE_SYSTEM 直立平衡点(theta=0)周りで線形化した状態空間モデル
%   状態: x = [x; x_dot; theta; theta_dot]
%   入力: u = F

    M = p.M;
    m = p.m;
    l = p.l;
    g = p.g;
    b = p.b;

    % theta=0 周りの線形化
    % 分母: D = M + m - m = M （cos(0)=1）
    A = [0,    1,            0,       0;
         0, -b/M,      -m*g/M,       0;
         0,    0,            0,       1;
         0, b/(M*l), (M+m)*g/(M*l),  0];

    B = [0;
         1/M;
         0;
        -1/(M*l)];

    C = eye(4);
    D = zeros(4, 1);
end
