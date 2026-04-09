function force = pid_step(gains, dt, max_force, state)  %#codegen
% PID_STEP Cascaded PID balance controller (codegen entry point).
%   gains:     struct with fields theta (Kp,Ki,Kd) and x (Kp,Ki,Kd), single
%   dt:        single — balance loop period [s]
%   max_force: single — force saturation limit [N]
%   state:     single(4x1) — [x; x_dot; theta; theta_dot]
%   force:     single       — control force [N]

    assert(isa(state, 'single'));
    assert(all(size(state) == [4 1]));

    persistent angle_integral angle_prev_error angle_first_call
    persistent pos_integral pos_prev_error pos_first_call

    if isempty(angle_integral)
        angle_integral = single(0);
        angle_prev_error = single(0);
        angle_first_call = true;
        pos_integral = single(0);
        pos_prev_error = single(0);
        pos_first_call = true;
    end

    % 角度PID（内側ループ）: setpoint=theta, measurement=0
    theta = state(3);
    [u_theta, angle_integral, angle_prev_error, angle_first_call] = ...
        pid_update(gains.theta, dt, max_force, theta, single(0), ...
                   angle_integral, angle_prev_error, angle_first_call);

    % 位置PID（外側ループ）: setpoint=position, measurement=0
    position = state(1);
    [u_x, pos_integral, pos_prev_error, pos_first_call] = ...
        pid_update(gains.x, dt, max_force, position, single(0), ...
                   pos_integral, pos_prev_error, pos_first_call);

    force = u_theta + u_x;
end

function [output, integral, prev_error, first_call] = ...
        pid_update(cfg, dt, max_force, setpoint, measurement, ...
                   integral, prev_error, first_call)
    error_val = setpoint - measurement;
    p_term = cfg.Kp * error_val;
    integral = integral + cfg.Ki * error_val * dt;

    if first_call
        d_term = single(0);
        first_call = false;
    else
        d_term = cfg.Kd * (error_val - prev_error) / dt;
    end

    unclamped = p_term + integral + d_term;
    output = max(min(unclamped, max_force), -max_force);

    if unclamped > max_force && integral > single(0)
        integral = max(max_force - p_term - d_term, single(0));
    elseif unclamped < -max_force && integral < single(0)
        integral = min(-max_force - p_term - d_term, single(0));
    end

    prev_error = error_val;
end
