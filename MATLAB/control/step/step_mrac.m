function api = step_mrac()
% STEP_MRAC Model Reference Adaptive Control.
%   api = step_mrac();
%   controller = api.init(ctrl_param);
%   [voltage, controller, current_used, current_state] = api.step(controller, state, measured_current, p, dt);

    api.init = @init_mrac_controller;
    api.step = @mrac_step;
end

function controller = init_mrac_controller(c)
    controller.reference_state = zeros(5, 1);
    controller.adaptive_gains = zeros(5, 1);
    controller.initialized = false;
    controller.prev_voltage = 0.0;
    controller.current_state = 0.0;
    controller.startup_timer = 0.0;
    controller.ctrl = c;
end

function [voltage, controller, current_used, current_state] = mrac_step(controller, state, measured_current, p, dt)
    util = sim_utils();
    c = controller.ctrl;

    motor_speed = p.G * state.velocity / p.r;
    controller.current_state = step_current_estimate( ...
        controller.current_state, controller.prev_voltage, motor_speed, p, dt);

    if isfield(c, 'UseCurrentEstimate') && c.UseCurrentEstimate
        current = controller.current_state;
    else
        current = measured_current;
    end

    x = [ ...
        state.position; ...
        state.velocity; ...
        soft_zone(state.theta, c.ThetaSoftZone, c.InnerAngleGain); ...
        soft_zone(state.theta_dot, c.ThetaDotSoftZone, c.InnerAngularVelocityGain); ...
        soft_zone(current, c.CurrentSoftZone, c.InnerCurrentGain)];

    if ~controller.initialized
        controller.reference_state = x;
        controller.initialized = true;
        controller.startup_timer = 0.0;
    else
        controller.reference_state = controller.reference_state + dt * (c.Am * controller.reference_state);
        controller.startup_timer = controller.startup_timer + dt;
    end

    error_value = x - controller.reference_state;
    s = error_value' * c.PB;
    x_norm = x' * x;

    adaptation_enabled = abs(state.theta) < c.EnableAngleLimit ...
        && abs(state.theta_dot) < c.EnableAngularVelocityLimit ...
        && abs(current) < c.EnableCurrentLimit ...
        && controller.startup_timer >= c.AdaptationDelaySec ...
        && abs(s) > c.ErrorDeadzone;

    if adaptation_enabled
        scale = s / (c.NormalizationEps + x_norm);
        for idx = 1:5
            update = c.GammaDiag(idx, idx) * x(idx) * scale ...
                - c.Sigma * controller.adaptive_gains(idx);
            controller.adaptive_gains(idx) = controller.adaptive_gains(idx) + dt * update;
            controller.adaptive_gains(idx) = util.clamp( ...
                controller.adaptive_gains(idx), -c.MaxAdaptiveGain, c.MaxAdaptiveGain);
        end
    else
        controller.adaptive_gains = controller.adaptive_gains + dt * (-c.Sigma * controller.adaptive_gains);
    end

    raw_voltage = -c.Kx * x - controller.adaptive_gains' * x;
    max_delta = c.MaxVoltageSlewRate * dt;
    slewed_voltage = controller.prev_voltage + util.clamp( ...
        raw_voltage - controller.prev_voltage, -max_delta, max_delta);
    compensated_voltage = apply_low_speed_drive_compensation( ...
        slewed_voltage, state.theta, state.theta_dot, state.velocity, c);
    voltage = util.clamp(compensated_voltage, -c.MaxVoltage, c.MaxVoltage);
    controller.prev_voltage = voltage;
    current_used = current;
    current_state = controller.current_state;
end

function y = soft_zone(x, zone, inner_gain)
    abs_x = abs(x);
    if abs_x <= zone
        y = inner_gain * x;
    else
        y = sign(x) * (inner_gain * zone + (abs_x - zone));
    end
end

function y = apply_low_speed_drive_compensation(voltage, theta, theta_dot, velocity, c)
    abs_voltage = abs(voltage);
    if abs_voltage < c.MinActiveVoltage
        y = 0.0;
    elseif abs(theta) < c.StictionEnableAngleLimit ...
            && abs(theta_dot) < c.StictionEnableAngularVelocityLimit ...
            && abs(velocity) < c.LowSpeedVelocityThreshold ...
            && abs_voltage < c.MinDriveVoltage
        y = sign(voltage) * c.MinDriveVoltage;
    else
        y = voltage;
    end
end

function next = step_current_estimate(current, motor_voltage, motor_speed, p, dt)
    numerator = current + (dt / p.La) * (motor_voltage - p.Ke * motor_speed);
    denominator = 1.0 + (dt * p.Ra / p.La);
    next = numerator / denominator;
end
