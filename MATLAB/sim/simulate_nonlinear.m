function [t, x, u_hist] = simulate_nonlinear(controller_type, p, x0, tspan, ctrl_param)
% SIMULATE_NONLINEAR 非線形シミュレーション
%   controller_type: 'lqr' / 'pid' / 'mrac'
%   p: システムパラメータ
%   x0: 初期状態 [x; x_dot; theta; theta_dot]
%   tspan: シミュレーション時間 [t_start, t_end]
%   ctrl_param:
%     'lqr'  -> ゲインK
%     'pid'  -> pid_gains構造体
%     'mrac' -> design_mrac() が返す構造体

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

    % MRAC用内部状態
    xm = x0;
    theta_hat = zeros(4, 1);
    prev_u = 0;
    theta_filt = x0(3);
    theta_dot_est = 0;
    theta_est_initialized = false;
    alpha_theta = lpf_alpha(cTheta(ctrl_param), dt);
    alpha_theta_dot = lpf_alpha(cThetaDot(ctrl_param), dt);

    for k = 1:N-1
        state = x(:, k);

        switch controller_type
            case 'lqr'
                K = ctrl_param;
                u = -K * state;

            case 'pid'
                g = ctrl_param;

                % Firmware と同じ符号規約:
                % theta > 0 / x > 0 に対して正の制御入力を出す。
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

            case 'mrac'
                c = ctrl_param;
                if ~theta_est_initialized
                    theta_filt = state(3);
                    theta_dot_est = 0;
                    theta_est_initialized = true;
                else
                    theta_prev = theta_filt;
                    theta_filt = alpha_theta * state(3) + (1 - alpha_theta) * theta_filt;
                    theta_dot_raw = (theta_filt - theta_prev) / dt;
                    theta_dot_est = alpha_theta_dot * theta_dot_raw ...
                        + (1 - alpha_theta_dot) * theta_dot_est;
                end

                ctrl_state = [state(1); state(2); theta_filt; theta_dot_est];
                phi = ctrl_state;
                phi(3) = soft_zone(phi(3), c.ThetaSoftZone, c.InnerAngleGain);
                phi(4) = soft_zone(phi(4), c.ThetaDotSoftZone, c.InnerAngularVelocityGain);
                xm_ctrl = xm;
                xm_ctrl(3) = soft_zone(xm_ctrl(3), c.ThetaSoftZone, c.InnerAngleGain);
                xm_ctrl(4) = soft_zone(xm_ctrl(4), c.ThetaDotSoftZone, c.InnerAngularVelocityGain);
                e = phi - xm_ctrl;
                s = e' * c.PB;
                denom = c.NormalizationEps + phi' * phi;

                u_nom = -c.Kx * ctrl_state;
                u_ad = -theta_hat' * phi;
                u_raw = u_nom + u_ad;
                max_du = c.MaxForceSlewRate * dt;
                u_slewed = prev_u + min(max(u_raw - prev_u, -max_du), max_du);
                u = min(max(u_slewed, -c.MaxForce), c.MaxForce);

                adapt_enabled = abs(ctrl_state(3)) < c.EnableAngleLimit ...
                    && abs(ctrl_state(4)) < c.EnableAngularVelocityLimit ...
                    && abs(u_raw) < c.MaxForce ...
                    && abs(s) > c.ErrorDeadzone ...
                    && t(k) >= c.AdaptationDelaySec;

                if adapt_enabled
                    theta_hat_dot = c.GammaDiag * phi * (s / denom) - c.Sigma * theta_hat;
                else
                    theta_hat_dot = -c.Sigma * theta_hat;
                end

                theta_hat = theta_hat + dt * theta_hat_dot;
                theta_hat = min(max(theta_hat, -c.MaxAdaptiveGain), c.MaxAdaptiveGain);
                xm = xm + dt * (c.Am * xm);
                prev_u = u;
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

function y = soft_zone(x, zone, inner_gain)
    ax = abs(x);
    sx = sign(x);
    if ax <= zone
        y = inner_gain * x;
    else
        y = sx * (inner_gain * zone + (ax - zone));
    end
end

function cutoff = cTheta(c)
    if isfield(c, 'ThetaFilterCutoff')
        cutoff = c.ThetaFilterCutoff;
    else
        cutoff = 50.0;
    end
end

function cutoff = cThetaDot(c)
    if isfield(c, 'ThetaDotFilterCutoff')
        cutoff = c.ThetaDotFilterCutoff;
    else
        cutoff = 25.0;
    end
end

function alpha = lpf_alpha(cutoff_freq, sample_time)
    tau = 1 / (2 * pi * cutoff_freq);
    alpha = sample_time / (tau + sample_time);
end
