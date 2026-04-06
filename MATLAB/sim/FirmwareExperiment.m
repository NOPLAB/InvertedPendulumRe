classdef FirmwareExperiment
    % FIRMWAREEXPERIMENT Generic firmware-like simulator for mode1-mode4.
    %   Mode mapping:
    %     mode1 / debug -> no control output
    %     mode2 / pid   -> force target + current loop
    %     mode3 / lqr   -> force target + current loop
    %     mode4 / mrac  -> direct voltage control

    properties (SetAccess = private)
        p
        mode
        firmware_mode
        mode_name
        ctrl_param
        options
    end

    methods
        function obj = FirmwareExperiment(p, mode, ctrl_param, options)
            if nargin < 4
                options = struct();
            end
            if nargin < 3
                ctrl_param = [];
            end

            spec = FirmwareExperiment.normalize_mode(mode);

            obj.p = p;
            obj.mode = spec.mode_number;
            obj.firmware_mode = spec.firmware_mode;
            obj.mode_name = spec.name;
            obj.options = FirmwareExperiment.default_options(options);

            if isempty(ctrl_param)
                obj.ctrl_param = FirmwareExperiment.default_controller_param(p, spec);
            else
                obj.ctrl_param = ctrl_param;
            end
        end

        function result = run(obj, x0, t_end)
            opts = obj.options;
            current_dt = 1 / opts.current_loop_frequency;
            balance_ratio = opts.current_loop_frequency / opts.balance_loop_frequency;
            balance_decimation = round(balance_ratio);

            if abs(balance_ratio - balance_decimation) > 1e-12
                error('FirmwareExperiment:InvalidTiming', ...
                    'current_loop_frequency must be an integer multiple of balance_loop_frequency.');
            end

            steps = floor(t_end / current_dt) + 1;
            state = x0(:);
            runtime = obj.init_runtime(state);
            balance_sample = obj.empty_balance_sample();
            hist = obj.init_history(steps);

            for k = 1:steps
                t = (k - 1) * current_dt;

                theta_meas = FirmwareExperiment.measure_theta(state(3), opts);
                [qei_r, qei_l] = FirmwareExperiment.measure_encoder(state(1), opts, obj.p);

                if mod(k - 1, balance_decimation) == 0
                    [balance_sample, runtime] = obj.step_balance_loop( ...
                        runtime, theta_meas, qei_r, qei_l, balance_decimation * current_dt);
                end

                [motor_voltage, applied_force, runtime] = obj.step_current_loop(runtime, state(2), current_dt);
                state = FirmwareExperiment.rk4_step( ...
                    @(x, u) plant_model(0, x, u, obj.p), state, applied_force, current_dt);

                hist.t(k) = t;
                hist.x(k, :) = state.';
                hist.force(k) = applied_force;
                hist.target_force(k) = runtime.target_force;
                hist.target_current(k) = runtime.target_current;
                hist.target_voltage(k) = runtime.target_voltage;
                hist.voltage(k) = motor_voltage;
                hist.current(k) = runtime.current_actual;
                hist.current_meas(k) = balance_sample.current_meas;
                hist.current_ctrl(k) = runtime.current_ctrl;
                hist.current_est(k) = balance_sample.current_used;
                hist.current_state_est(k) = balance_sample.current_state;
                hist.theta_meas(k) = theta_meas;
                hist.theta_filt(k) = balance_sample.theta;
                hist.theta_dot_direct(k) = balance_sample.theta_dot_direct;
                hist.theta_dot_est(k) = balance_sample.theta_dot_est;
                hist.position_meas(k) = balance_sample.position;
                hist.velocity_meas(k) = balance_sample.velocity;
                hist.velocity_est(k) = balance_sample.velocity_est;
                hist.qei_r(k) = balance_sample.qei_r;
                hist.qei_l(k) = balance_sample.qei_l;
            end

            result = hist;
            result.mode = obj.mode;
            result.firmware_mode = obj.firmware_mode;
            result.mode_name = obj.mode_name;
            result.options = opts;
            result.ctrl_param = obj.ctrl_param;
        end
    end

    methods (Access = private)
        function runtime = init_runtime(obj, initial_state)
            opts = obj.options;
            balance_dt = 1 / opts.balance_loop_frequency;
            current_dt = 1 / opts.current_loop_frequency;
            [qei_r0, qei_l0] = FirmwareExperiment.measure_encoder(initial_state(1), opts, obj.p);

            runtime.theta_filter = FirmwareExperiment.init_lpf(balance_dt, obj.theta_filter_cutoff());
            runtime.theta_dot_filter = FirmwareExperiment.init_lpf(balance_dt, obj.theta_dot_filter_cutoff());
            runtime.prev_theta = 0.0;
            runtime.theta_initialized = false;
            runtime.qei_r_offset = qei_r0;
            runtime.qei_l_offset = qei_l0;
            runtime.qei_r_prev = qei_r0;
            runtime.qei_l_prev = qei_l0;
            runtime.current_actual = 0.0;
            runtime.target_force = 0.0;
            runtime.target_current = 0.0;
            runtime.target_voltage = 0.0;
            runtime.current_ctrl = NaN;
            runtime.motor_prev_voltage = 0.0;
            runtime.observer = FirmwareExperiment.init_observer(opts.observer);

            switch obj.mode_name
                case 'pid'
                    runtime.balance_pid = FirmwareExperiment.init_balance_pid( ...
                        obj.ctrl_param, balance_dt, opts.max_force);
                case 'mrac'
                    runtime.mrac = FirmwareExperiment.init_mrac_controller(obj.ctrl_param);
            end

            if ~strcmp(obj.mode_name, 'mrac')
                runtime.current_filter = FirmwareExperiment.init_lpf(current_dt, opts.current_filter_cutoff);
                runtime.current_pid = FirmwareExperiment.init_pid( ...
                    opts.current_pid, current_dt, -opts.max_voltage, opts.max_voltage);
            end
        end

        function cutoff = theta_filter_cutoff(obj)
            cutoff = obj.options.theta_filter_cutoff;
            if isstruct(obj.ctrl_param) && isfield(obj.ctrl_param, 'ThetaFilterCutoff')
                cutoff = obj.ctrl_param.ThetaFilterCutoff;
            end
        end

        function cutoff = theta_dot_filter_cutoff(obj)
            cutoff = obj.options.theta_dot_filter_cutoff;
            if isstruct(obj.ctrl_param) && isfield(obj.ctrl_param, 'ThetaDotFilterCutoff')
                cutoff = obj.ctrl_param.ThetaDotFilterCutoff;
            end
        end

        function hist = init_history(~, steps)
            hist = struct();
            hist.t = zeros(steps, 1);
            hist.x = zeros(steps, 4);
            hist.force = zeros(steps, 1);
            hist.target_force = zeros(steps, 1);
            hist.target_current = zeros(steps, 1);
            hist.target_voltage = zeros(steps, 1);
            hist.voltage = zeros(steps, 1);
            hist.current = zeros(steps, 1);
            hist.current_meas = NaN(steps, 1);
            hist.current_ctrl = NaN(steps, 1);
            hist.current_est = NaN(steps, 1);
            hist.current_state_est = NaN(steps, 1);
            hist.theta_meas = NaN(steps, 1);
            hist.theta_filt = NaN(steps, 1);
            hist.theta_dot_direct = NaN(steps, 1);
            hist.theta_dot_est = NaN(steps, 1);
            hist.position_meas = NaN(steps, 1);
            hist.velocity_meas = NaN(steps, 1);
            hist.velocity_est = NaN(steps, 1);
            hist.qei_r = NaN(steps, 1);
            hist.qei_l = NaN(steps, 1);
        end

        function sample = empty_balance_sample(~)
            sample = struct( ...
                'position', 0.0, ...
                'velocity', 0.0, ...
                'velocity_est', 0.0, ...
                'theta', 0.0, ...
                'theta_dot_direct', 0.0, ...
                'theta_dot_est', 0.0, ...
                'current_meas', 0.0, ...
                'current_used', NaN, ...
                'current_state', NaN, ...
                'qei_r', 0.0, ...
                'qei_l', 0.0);
        end

        function [sample, runtime] = step_balance_loop(obj, runtime, theta_meas, qei_r, qei_l, balance_dt)
            position_r = (qei_r - runtime.qei_r_offset) * FirmwareExperiment.pulse_to_position(obj.p);
            position_l = -(qei_l - runtime.qei_l_offset) * FirmwareExperiment.pulse_to_position(obj.p);
            velocity_r = (qei_r - runtime.qei_r_prev) * FirmwareExperiment.pulse_to_position(obj.p) / balance_dt;
            velocity_l = -(qei_l - runtime.qei_l_prev) * FirmwareExperiment.pulse_to_position(obj.p) / balance_dt;

            runtime.qei_r_prev = qei_r;
            runtime.qei_l_prev = qei_l;

            position = (position_r + position_l) / 2.0;
            velocity = (velocity_r + velocity_l) / 2.0;

            runtime.theta_filter = FirmwareExperiment.lpf_update(runtime.theta_filter, theta_meas);
            theta = runtime.theta_filter.output;

            if runtime.theta_initialized
                theta_dot_raw = (theta - runtime.prev_theta) / balance_dt;
            else
                runtime.theta_initialized = true;
                theta_dot_raw = 0.0;
            end
            runtime.prev_theta = theta;
            runtime.theta_dot_filter = FirmwareExperiment.lpf_update(runtime.theta_dot_filter, theta_dot_raw);
            theta_dot_direct = runtime.theta_dot_filter.output;

            [runtime.observer, observer_velocity, observer_theta_dot] = FirmwareExperiment.update_observer( ...
                runtime.observer, position, theta, runtime.target_current, obj.p);

            signed_current = FirmwareExperiment.correct_current_sign( ...
                abs(runtime.current_actual), runtime.motor_prev_voltage);

            switch obj.mode_name
                case 'debug'
                    runtime.target_force = 0.0;
                    runtime.target_current = 0.0;
                    runtime.target_voltage = 0.0;
                    velocity_est = observer_velocity;
                    theta_dot_est = observer_theta_dot;
                    current_used = NaN;
                    current_state = NaN;

                case 'pid'
                    processed = struct( ...
                        'position', position, ...
                        'velocity', observer_velocity, ...
                        'theta', theta, ...
                        'theta_dot', observer_theta_dot);
                    [force, runtime.balance_pid] = FirmwareExperiment.balance_pid_step( ...
                        runtime.balance_pid, processed);
                    runtime.target_force = FirmwareExperiment.clamp(force, -obj.options.max_force, obj.options.max_force);
                    runtime.target_current = runtime.target_force * FirmwareExperiment.force_to_current(obj.p);
                    runtime.target_voltage = 0.0;
                    velocity_est = processed.velocity;
                    theta_dot_est = processed.theta_dot;
                    current_used = NaN;
                    current_state = NaN;

                case 'lqr'
                    x_ctrl = [position; observer_velocity; theta; observer_theta_dot];
                    gain = reshape(obj.ctrl_param, 1, []);
                    force = -(gain * x_ctrl);
                    runtime.target_force = FirmwareExperiment.clamp(force, -obj.options.max_force, obj.options.max_force);
                    runtime.target_current = runtime.target_force * FirmwareExperiment.force_to_current(obj.p);
                    runtime.target_voltage = 0.0;
                    velocity_est = observer_velocity;
                    theta_dot_est = observer_theta_dot;
                    current_used = NaN;
                    current_state = NaN;

                case 'mrac'
                    processed = struct( ...
                        'position', position, ...
                        'velocity', velocity, ...
                        'theta', theta, ...
                        'theta_dot', theta_dot_direct);
                    [runtime.target_voltage, runtime.mrac, current_used, current_state] = ...
                        FirmwareExperiment.mrac_step(runtime.mrac, processed, signed_current, obj.p, balance_dt);
                    runtime.target_force = 0.0;
                    runtime.target_current = 0.0;
                    velocity_est = processed.velocity;
                    theta_dot_est = processed.theta_dot;

                otherwise
                    error('FirmwareExperiment:UnsupportedMode', 'Unsupported mode: %s', obj.mode_name);
            end

            sample = struct( ...
                'position', position, ...
                'velocity', velocity, ...
                'velocity_est', velocity_est, ...
                'theta', theta, ...
                'theta_dot_direct', theta_dot_direct, ...
                'theta_dot_est', theta_dot_est, ...
                'current_meas', signed_current, ...
                'current_used', current_used, ...
                'current_state', current_state, ...
                'qei_r', qei_r - runtime.qei_r_offset, ...
                'qei_l', qei_l - runtime.qei_l_offset);
        end

        function [motor_voltage, applied_force, runtime] = step_current_loop(obj, runtime, cart_velocity, current_dt)
            p = obj.p;
            opts = obj.options;

            switch obj.mode_name
                case 'mrac'
                    voltage_cmd = FirmwareExperiment.clamp(runtime.target_voltage, -opts.vin, opts.vin);
                    duty = FirmwareExperiment.clamp(voltage_cmd / opts.vin, -1.0, 1.0);
                    motor_voltage = duty * opts.vin;
                    runtime.motor_prev_voltage = motor_voltage;
                    runtime.current_ctrl = NaN;

                otherwise
                    signed_current = FirmwareExperiment.correct_current_sign( ...
                        abs(runtime.current_actual), runtime.motor_prev_voltage);
                    runtime.current_filter = FirmwareExperiment.lpf_update(runtime.current_filter, signed_current);
                    runtime.current_ctrl = runtime.current_filter.output;
                    [voltage_cmd, runtime.current_pid] = FirmwareExperiment.pid_update( ...
                        runtime.current_pid, runtime.target_current, runtime.current_ctrl);
                    duty = FirmwareExperiment.clamp(voltage_cmd / opts.vin, -1.0, 1.0);
                    motor_voltage = duty * opts.vin;
                    runtime.motor_prev_voltage = motor_voltage;
            end

            if opts.enable_motor_electrical_dynamics
                motor_speed = p.G * cart_velocity / p.r;
                runtime.current_actual = FirmwareExperiment.step_motor_current( ...
                    runtime.current_actual, motor_voltage, motor_speed, p, current_dt);
            else
                runtime.current_actual = (motor_voltage - p.Ke * p.G * cart_velocity / p.r) / p.Ra;
            end

            applied_force = 2.0 * p.G * p.Kt * runtime.current_actual / p.r;
            if opts.enable_force_saturation
                applied_force = FirmwareExperiment.clamp(applied_force, -opts.force_limit, opts.force_limit);
            end
        end
    end

    methods (Static)
        function result = simulate(p, x0, t_end, mode, ctrl_param, options)
            if nargin < 6
                options = struct();
            end
            if nargin < 5
                ctrl_param = [];
            end
            experiment = FirmwareExperiment(p, mode, ctrl_param, options);
            result = experiment.run(x0, t_end);
        end
    end

    methods (Static, Access = private)
        function spec = normalize_mode(mode)
            if isstring(mode) || ischar(mode)
                key = lower(strtrim(char(mode)));
                switch key
                    case {'mode1', 'debug'}
                        spec = struct('mode_number', 1, 'firmware_mode', 0, 'name', 'debug');
                    case {'mode2', 'pid'}
                        spec = struct('mode_number', 2, 'firmware_mode', 1, 'name', 'pid');
                    case {'mode3', 'lqr'}
                        spec = struct('mode_number', 3, 'firmware_mode', 2, 'name', 'lqr');
                    case {'mode4', 'mrac'}
                        spec = struct('mode_number', 4, 'firmware_mode', 3, 'name', 'mrac');
                    otherwise
                        error('FirmwareExperiment:InvalidMode', 'Unsupported mode: %s', char(mode));
                end
                return;
            end

            if isnumeric(mode) && isscalar(mode)
                switch double(mode)
                    case 1
                        spec = struct('mode_number', 1, 'firmware_mode', 0, 'name', 'debug');
                    case 2
                        spec = struct('mode_number', 2, 'firmware_mode', 1, 'name', 'pid');
                    case 3
                        spec = struct('mode_number', 3, 'firmware_mode', 2, 'name', 'lqr');
                    case 4
                        spec = struct('mode_number', 4, 'firmware_mode', 3, 'name', 'mrac');
                    otherwise
                        error('FirmwareExperiment:InvalidMode', ...
                            'Numeric mode must be one of 1, 2, 3, 4.');
                end
                return;
            end

            error('FirmwareExperiment:InvalidMode', 'mode must be a string or scalar number.');
        end

        function ctrl_param = default_controller_param(p, spec)
            switch spec.name
                case 'debug'
                    ctrl_param = [];
                case 'pid'
                    ctrl_param = design_pid();
                case 'lqr'
                    [A, B, ~, ~] = linearize_system(p);
                    ctrl_param = design_lqr(A, B);
                case 'mrac'
                    ctrl_param = design_mrac(p);
                otherwise
                    error('FirmwareExperiment:InvalidMode', 'Unsupported mode: %s', spec.name);
            end
        end

        function opts = default_options(options)
            opts = struct( ...
                'vin', 12.0, ...
                'current_loop_frequency', 5000.0, ...
                'balance_loop_frequency', 1000.0, ...
                'enable_encoder_quantization', true, ...
                'enable_theta_quantization', true, ...
                'enable_motor_electrical_dynamics', true, ...
                'enable_force_saturation', false, ...
                'force_limit', 10.0, ...
                'max_force', 10.0, ...
                'max_voltage', 12.0, ...
                'theta_filter_cutoff', 50.0, ...
                'theta_dot_filter_cutoff', 25.0, ...
                'current_filter_cutoff', 500.0, ...
                'current_pid', struct( ...
                    'Kp', 0.928, ...
                    'Ki', 10178.8, ...
                    'Kd', 0.0), ...
                'observer', struct( ...
                    'Ad', [ ...
                        1.0, 0.001, -1.92867233e-7, 7.19966164e-9; ...
                        0.0, 1.0, -3.85612705e-4, 1.43304687e-5; ...
                        0.0, 0.0, 1.00002547, 9.99049074e-4; ...
                        0.0, 0.0, 5.09314206e-2, 9.98107243e-1], ...
                    'Bd', [8.65667613e-7; 1.73131455e-3; -4.32569161e-6; -8.64865233e-3], ...
                    'Ld', [ ...
                        6.51586527e-2, 5.20789139e-3; ...
                        1.0445936, 1.67298784e-1; ...
                        4.21340529e-3, 6.08237631e-2; ...
                        1.25858405e-1, 9.00159305e-1]));

            opts = FirmwareExperiment.merge_structs(opts, options);
        end

        function out = merge_structs(base, override)
            out = base;
            if isempty(override)
                return;
            end

            fields = fieldnames(override);
            for idx = 1:numel(fields)
                name = fields{idx};
                if isfield(out, name) && isstruct(out.(name)) && isstruct(override.(name))
                    out.(name) = FirmwareExperiment.merge_structs(out.(name), override.(name));
                else
                    out.(name) = override.(name);
                end
            end
        end

        function lpf = init_lpf(sample_time, cutoff_freq)
            lpf.alpha = FirmwareExperiment.lpf_alpha(cutoff_freq, sample_time);
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

        function [output, pid] = pid_update(pid, setpoint, measurement)
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
            output = FirmwareExperiment.clamp(unclamped, pid.min_output, pid.max_output);

            if unclamped > pid.max_output && pid.integral > 0.0
                pid.integral = max(pid.max_output - p_term - d_term, 0.0);
            elseif unclamped < pid.min_output && pid.integral < 0.0
                pid.integral = min(pid.min_output - p_term - d_term, 0.0);
            end

            pid.prev_error = error_value;
        end

        function balance_pid = init_balance_pid(gains, dt, max_force)
            balance_pid.angle = FirmwareExperiment.init_pid(gains.theta, dt, -max_force, max_force);
            balance_pid.position = FirmwareExperiment.init_pid(gains.x, dt, -max_force, max_force);
        end

        function [force, balance_pid] = balance_pid_step(balance_pid, state)
            [u_theta, balance_pid.angle] = FirmwareExperiment.pid_update( ...
                balance_pid.angle, 0.0, state.theta);
            [u_x, balance_pid.position] = FirmwareExperiment.pid_update( ...
                balance_pid.position, 0.0, state.position);
            force = u_theta + u_x;
        end

        function observer = init_observer(cfg)
            observer.Ad = cfg.Ad;
            observer.Bd = cfg.Bd(:);
            observer.Ld = cfg.Ld;
            observer.x_hat = zeros(4, 1);
            observer.prev_force = 0.0;
            observer.initialized = false;
        end

        function [observer, velocity, theta_dot] = update_observer(observer, position, theta, target_current, p)
            applied_force = target_current / FirmwareExperiment.force_to_current(p);

            if ~observer.initialized
                observer.x_hat = [position; 0.0; theta; 0.0];
                observer.prev_force = applied_force;
                observer.initialized = true;
                velocity = 0.0;
                theta_dot = 0.0;
                return;
            end

            innovation = [position - observer.x_hat(1); theta - observer.x_hat(3)];
            next = observer.Ad * observer.x_hat + observer.Bd * observer.prev_force + observer.Ld * innovation;
            observer.x_hat = next;
            observer.prev_force = applied_force;
            velocity = next(2);
            theta_dot = next(4);
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
            c = controller.ctrl;

            motor_speed = p.G * state.velocity / p.r;
            controller.current_state = FirmwareExperiment.step_current_estimate( ...
                controller.current_state, controller.prev_voltage, motor_speed, p, dt);

            if isfield(c, 'UseCurrentEstimate') && c.UseCurrentEstimate
                current = controller.current_state;
            else
                current = measured_current;
            end

            x = [ ...
                state.position; ...
                state.velocity; ...
                FirmwareExperiment.soft_zone(state.theta, c.ThetaSoftZone, c.InnerAngleGain); ...
                FirmwareExperiment.soft_zone(state.theta_dot, c.ThetaDotSoftZone, c.InnerAngularVelocityGain); ...
                FirmwareExperiment.soft_zone(current, c.CurrentSoftZone, c.InnerCurrentGain)];

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
                    controller.adaptive_gains(idx) = FirmwareExperiment.clamp( ...
                        controller.adaptive_gains(idx), -c.MaxAdaptiveGain, c.MaxAdaptiveGain);
                end
            else
                controller.adaptive_gains = controller.adaptive_gains + dt * (-c.Sigma * controller.adaptive_gains);
            end

            raw_voltage = -c.Kx * x - controller.adaptive_gains' * x;
            max_delta = c.MaxVoltageSlewRate * dt;
            slewed_voltage = controller.prev_voltage + FirmwareExperiment.clamp( ...
                raw_voltage - controller.prev_voltage, -max_delta, max_delta);
            compensated_voltage = FirmwareExperiment.apply_low_speed_drive_compensation( ...
                slewed_voltage, state.theta, state.theta_dot, state.velocity, c);
            voltage = FirmwareExperiment.clamp(compensated_voltage, -c.MaxVoltage, c.MaxVoltage);
            controller.prev_voltage = voltage;
            current_used = current;
            current_state = controller.current_state;
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
            pulses = position / FirmwareExperiment.pulse_to_position(p);
            if opts.enable_encoder_quantization
                pulses = round(pulses);
            end
            qei_r = pulses;
            qei_l = -pulses;
        end

        function value = pulse_to_position(p)
            value = (2.0 * pi * p.r) / (48.0 * p.G);
        end

        function value = force_to_current(p)
            value = p.r / (2.0 * p.G * p.Kt);
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
            k2 = f(state + dt / 2 * k1, input);
            k3 = f(state + dt / 2 * k2, input);
            k4 = f(state + dt * k3, input);
            next = state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
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
    end
end
