function result = simulate_firmware_mode4(p, x0, t_end, ctrl_param, options)
% SIMULATE_FIRMWARE_MODE4 Simulate the current mode4 firmware logic.
%   Includes:
%   - 5 kHz motor update
%   - 1 kHz balance loop
%   - encoder pulse quantization
%   - theta ADC quantization
%   - theta/theta_dot filtering used by firmware
%   - direct voltage hold for mode4
%   - motor current dynamics

    if nargin < 5
        options = struct();
    end

    opts = default_options(options);
    dt = 1 / 5000;
    balance_decimation = 5;
    steps = floor(t_end / dt) + 1;

    state = x0(:);

    hist.t = zeros(steps, 1);
    hist.x = zeros(steps, 4);
    hist.force = zeros(steps, 1);
    hist.voltage = zeros(steps, 1);
    hist.target_voltage = zeros(steps, 1);
    hist.current = zeros(steps, 1);
    hist.current_est = zeros(steps, 1);
    hist.theta_meas = zeros(steps, 1);
    hist.theta_dot_est = zeros(steps, 1);
    hist.velocity_est = zeros(steps, 1);

    controller = init_mrac_controller(ctrl_param);

    qei_r_prev = 0;
    qei_l_prev = 0;
    current_actual = 0;
    target_voltage = 0;

    for k = 1:steps
        t = (k - 1) * dt;

        x_pos = state(1);
        x_dot = state(2);
        theta_true = state(3);

        theta_meas = measure_theta(theta_true, opts);
        [qei_r, qei_l] = measure_encoder(x_pos, opts, p);

        if mod(k - 1, balance_decimation) == 0
            velocity_r = (qei_r - qei_r_prev) * pulse_to_position(p) / (balance_decimation * dt);
            velocity_l = -(qei_l - qei_l_prev) * pulse_to_position(p) / (balance_decimation * dt);
            qei_r_prev = qei_r;
            qei_l_prev = qei_l;

            measured_current = correct_current_sign(abs(current_actual), controller.prev_voltage);
            ctrl_state = update_ctrl_state( ...
                controller, theta_meas, measured_current, velocity_r, velocity_l, qei_r, qei_l, p);
            controller = ctrl_state.controller;
            target_voltage = ctrl_state.voltage;
        end

        voltage_cmd = clamp(target_voltage, -opts.vin, opts.vin);
        duty = clamp(voltage_cmd / opts.vin, -1.0, 1.0);
        motor_voltage = duty * opts.vin;

        if opts.enable_motor_electrical_dynamics
            motor_speed = p.G * x_dot / p.r;
            current_actual = step_motor_current(current_actual, motor_voltage, motor_speed, p, dt);
        else
            current_actual = (motor_voltage - p.Ke * p.G * x_dot / p.r) / p.Ra;
        end
        applied_force = 2.0 * p.G * p.Kt * current_actual / p.r;

        if opts.enable_force_saturation
            applied_force = clamp(applied_force, -opts.force_limit, opts.force_limit);
        end

        state = rk4_step(@(x, u) plant_model(0, x, u, p), state, applied_force, dt);

        hist.t(k) = t;
        hist.x(k, :) = state.';
        hist.force(k) = applied_force;
        hist.voltage(k) = motor_voltage;
        hist.target_voltage(k) = target_voltage;
        hist.current(k) = current_actual;
        hist.current_est(k) = ctrl_state.processed.current;
        hist.theta_meas(k) = theta_meas;
        hist.theta_dot_est(k) = controller.theta_dot_filter.output;
        hist.velocity_est(k) = ctrl_state.processed.velocity;
    end

    result = hist;
end

function opts = default_options(options)
    opts = struct( ...
        'vin', 12.0, ...
        'enable_encoder_quantization', true, ...
        'enable_theta_quantization', true, ...
        'enable_motor_electrical_dynamics', true, ...
        'enable_force_saturation', false, ...
        'force_limit', 10.0);

    fields = fieldnames(options);
    for i = 1:numel(fields)
        opts.(fields{i}) = options.(fields{i});
    end
end

function controller = init_mrac_controller(c)
    controller.reference_state = zeros(5, 1);
    controller.adaptive_gains = zeros(5, 1);
    controller.initialized = false;
    controller.prev_voltage = 0.0;
    controller.current_state = 0.0;
    controller.startup_timer = 0.0;
    controller.prev_theta = 0.0;
    controller.theta_initialized = false;
    controller.theta_filter = init_lpf(1 / 1000, c.ThetaFilterCutoff);
    controller.theta_dot_filter = init_lpf(1 / 1000, c.ThetaDotFilterCutoff);
    controller.ctrl = c;
end

function lpf = init_lpf(sample_time, cutoff_freq)
    lpf.alpha = lpf_alpha(cutoff_freq, sample_time);
    lpf.output = 0.0;
    lpf.initialized = false;
end

function lpf = lpf_update(lpf, input)
    if ~lpf.initialized
        lpf.output = input;
        lpf.initialized = true;
    else
        lpf.output = lpf.alpha * input + (1 - lpf.alpha) * lpf.output;
    end
end

function ctrl_state = update_ctrl_state(controller, measured_theta, measured_current, velocity_r, velocity_l, qei_r, qei_l, p)
    c = controller.ctrl;

    position_r = qei_r * pulse_to_position(p);
    position_l = -qei_l * pulse_to_position(p);
    position = (position_r + position_l) / 2.0;
    velocity = (velocity_r + velocity_l) / 2.0;

    controller.theta_filter = lpf_update(controller.theta_filter, measured_theta);
    theta = controller.theta_filter.output;

    if controller.theta_initialized
        theta_dot_raw = (theta - controller.prev_theta) / (1 / 1000);
    else
        controller.theta_initialized = true;
        theta_dot_raw = 0.0;
    end
    controller.prev_theta = theta;
    controller.theta_dot_filter = lpf_update(controller.theta_dot_filter, theta_dot_raw);
    theta_dot = controller.theta_dot_filter.output;

    motor_speed = p.G * velocity / p.r;
    controller.current_state = step_current_estimate( ...
        controller.current_state, controller.prev_voltage, motor_speed, p, 1 / 1000);

    if isfield(c, 'UseCurrentEstimate') && c.UseCurrentEstimate
        current = controller.current_state;
    else
        current = measured_current;
    end

    processed.position = position;
    processed.velocity = velocity;
    processed.theta = theta;
    processed.theta_dot = theta_dot;
    processed.current = current;

    [voltage, controller] = mrac_step(controller, processed);

    ctrl_state.voltage = voltage;
    ctrl_state.processed = processed;
    ctrl_state.controller = controller;
end

function [voltage, controller] = mrac_step(controller, state)
    c = controller.ctrl;
    x = [ ...
        state.position; ...
        state.velocity; ...
        soft_zone(state.theta, c.ThetaSoftZone, c.InnerAngleGain); ...
        soft_zone(state.theta_dot, c.ThetaDotSoftZone, c.InnerAngularVelocityGain); ...
        soft_zone(state.current, c.CurrentSoftZone, c.InnerCurrentGain)];

    if ~controller.initialized
        controller.reference_state = x;
        controller.initialized = true;
        controller.startup_timer = 0.0;
    else
        controller.reference_state = controller.reference_state + (1 / 1000) * (c.Am * controller.reference_state);
        controller.startup_timer = controller.startup_timer + 1 / 1000;
    end

    error = x - controller.reference_state;
    s = error' * c.PB;
    x_norm = x' * x;

    adaptation_enabled = abs(state.theta) < c.EnableAngleLimit ...
        && abs(state.theta_dot) < c.EnableAngularVelocityLimit ...
        && abs(state.current) < c.EnableCurrentLimit ...
        && controller.startup_timer >= c.AdaptationDelaySec ...
        && abs(s) > c.ErrorDeadzone;

    if adaptation_enabled
        scale = s / (c.NormalizationEps + x_norm);
        for i = 1:5
            update = c.GammaDiag(i, i) * x(i) * scale - c.Sigma * controller.adaptive_gains(i);
            controller.adaptive_gains(i) = controller.adaptive_gains(i) + (1 / 1000) * update;
            controller.adaptive_gains(i) = clamp( ...
                controller.adaptive_gains(i), ...
                -c.MaxAdaptiveGain, ...
                c.MaxAdaptiveGain);
        end
    else
        controller.adaptive_gains = controller.adaptive_gains + (1 / 1000) * (-c.Sigma * controller.adaptive_gains);
    end

    raw_voltage = -c.Kx * x - controller.adaptive_gains' * x;
    max_delta = c.MaxVoltageSlewRate / 1000;
    slewed_voltage = controller.prev_voltage + clamp(raw_voltage - controller.prev_voltage, -max_delta, max_delta);
    compensated_voltage = apply_low_speed_drive_compensation( ...
        slewed_voltage, state.theta, state.theta_dot, state.velocity, c);
    voltage = clamp(compensated_voltage, -c.MaxVoltage, c.MaxVoltage);
    controller.prev_voltage = voltage;
end

function y = measure_theta(theta, opts)
    if ~opts.enable_theta_quantization
        y = theta;
        return;
    end

    theta_lsb = ((333.3 * (2 * pi / 360.0)) * (3.3 / 5.0)) / 4096.0;
    y = round(theta / theta_lsb) * theta_lsb;
end

function [qei_r, qei_l] = measure_encoder(position, opts, p)
    if opts.enable_encoder_quantization
        pulses = round(position / pulse_to_position(p));
    else
        pulses = position / pulse_to_position(p);
    end
    qei_r = pulses;
    qei_l = -pulses;
end

function value = pulse_to_position(p)
    value = (2.0 * pi * p.r) / (48 * p.G);
end

function y = correct_current_sign(measured_current, voltage_command)
    deadband = 0.5;
    abs_current = abs(measured_current);
    if abs(voltage_command) < deadband
        y = abs_current * (voltage_command / deadband);
    elseif voltage_command > 0.0
        y = abs_current;
    else
        y = -abs_current;
    end
end

function next = step_motor_current(current, motor_voltage, motor_speed, p, dt)
    a = -p.Ra / p.La;
    b = (motor_voltage - p.Ke * motor_speed) / p.La;
    exp_term = exp(a * dt);
    next = exp_term * current + ((exp_term - 1.0) / a) * b;
end

function next = step_current_estimate(current, motor_voltage, motor_speed, p, dt)
    numerator = current + (dt / p.La) * (motor_voltage - p.Ke * motor_speed);
    denominator = 1.0 + (dt * p.Ra / p.La);
    next = numerator / denominator;
end

function next = rk4_step(f, state, input, dt)
    k1 = f(state, input);
    k2 = f(state + dt/2 * k1, input);
    k3 = f(state + dt/2 * k2, input);
    k4 = f(state + dt * k3, input);
    next = state + dt / 6 * (k1 + 2*k2 + 2*k3 + k4);
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

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function alpha = lpf_alpha(cutoff_freq, sample_time)
    tau = 1 / (2 * pi * cutoff_freq);
    alpha = sample_time / (tau + sample_time);
end
