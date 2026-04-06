function api = step_pid()
% STEP_PID Cascaded PID balance controller.
%   api = step_pid();
%   balance_pid = api.init(gains, dt, max_force);
%   [force, balance_pid] = api.step(balance_pid, state);

    api.init = @init_balance_pid;
    api.step = @balance_pid_step;
end

function balance_pid = init_balance_pid(gains, dt, max_force)
    balance_pid.angle = init_pid(gains.theta, dt, -max_force, max_force);
    balance_pid.position = init_pid(gains.x, dt, -max_force, max_force);
end

function pid = init_pid(cfg, dt, min_output, max_output)
    pid.kp = cfg.Kp;
    pid.ki = cfg.Ki;
    pid.kd = cfg.Kd;
    pid.dt = dt;
    pid.min_output = min_output;
    pid.max_output = max_output;
    pid.integral = 0.0;
    pid.prev_error = 0.0;
    pid.first_call = true;
end

function [force, balance_pid] = balance_pid_step(balance_pid, state)
    % Rust firmware と同じ符号規約:
    % theta > 0 / position > 0 のとき正の補正力を出す。
    [u_theta, balance_pid.angle] = pid_update( ...
        balance_pid.angle, state.theta, 0.0);
    [u_x, balance_pid.position] = pid_update( ...
        balance_pid.position, state.position, 0.0);
    force = u_theta + u_x;
end

function [output, pid] = pid_update(pid, setpoint, measurement)
    util = sim_utils();
    error_value = setpoint - measurement;
    p_term = pid.kp * error_value;
    pid.integral = pid.integral + pid.ki * error_value * pid.dt;

    if pid.first_call
        d_term = 0.0;
        pid.first_call = false;
    else
        d_term = pid.kd * (error_value - pid.prev_error) / pid.dt;
    end

    unclamped = p_term + pid.integral + d_term;
    output = util.clamp(unclamped, pid.min_output, pid.max_output);

    if unclamped > pid.max_output && pid.integral > 0.0
        pid.integral = max(pid.max_output - p_term - d_term, 0.0);
    elseif unclamped < pid.min_output && pid.integral < 0.0
        pid.integral = min(pid.min_output - p_term - d_term, 0.0);
    end

    pid.prev_error = error_value;
end
