function [t, x, u_hist] = simulate_nonlinear(controller_type, p, x0, tspan, ctrl_param)
% SIMULATE_NONLINEAR 非線形シミュレーション
%   controller_type: 'lqr' または 'pid'
%   p: システムパラメータ
%   x0: 初期状態 [x; x_dot; theta; theta_dot]
%   tspan: シミュレーション時間 [t_start, t_end]
%   ctrl_param: LQRの場合はゲインK、PIDの場合はpid_gains構造体

    dt = 0.001;
    t = tspan(1):dt:tspan(2);
    N = length(t);
    x = zeros(4, N);
    u_hist = zeros(1, N);
    x(:, 1) = x0;

    % PID用の積分値
    int_theta = 0;
    int_x = 0;
    prev_theta = x0(3);
    prev_x_pos = x0(1);

    for k = 1:N-1
        state = x(:, k);

        switch controller_type
            case 'lqr'
                K = ctrl_param;
                u = -K * state;

            case 'pid'
                g = ctrl_param;

                % 角度PID
                e_theta = state(3);
                int_theta = int_theta + e_theta * dt;
                d_theta = (state(3) - prev_theta) / dt;
                u_theta = g.theta.Kp * e_theta + g.theta.Ki * int_theta + g.theta.Kd * d_theta;
                prev_theta = state(3);

                % 位置PID
                e_x = state(1);
                int_x = int_x + e_x * dt;
                d_x = (state(1) - prev_x_pos) / dt;
                u_x = g.x.Kp * e_x + g.x.Ki * int_x + g.x.Kd * d_x;
                prev_x_pos = state(1);

                u = u_theta + u_x;
        end

        % 制御入力を記録
        u_hist(k) = u;

        % Runge-Kutta 4次
        k1 = plant_model(t(k),          state,            u, p);
        k2 = plant_model(t(k) + dt/2,   state + dt/2*k1,  u, p);
        k3 = plant_model(t(k) + dt/2,   state + dt/2*k2,  u, p);
        k4 = plant_model(t(k) + dt,     state + dt*k3,    u, p);
        x(:, k+1) = state + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
    end

    u_hist(N) = u_hist(N-1);
    x = x';
    u_hist = u_hist';
    t = t';
end
