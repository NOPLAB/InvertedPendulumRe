classdef FirmwareExperiment
    % FIRMWAREEXPERIMENT Generic firmware-like simulator for mode1-mode4.
    %   Mode mapping:
    %     mode1 / debug -> no control output
    %     mode2 / pid   -> force target + current loop
    %     mode3 / lqr   -> force target + current loop
    %     mode4 / mrac  -> adaptive force target + current loop

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

            util = sim_utils();
            spec = util.normalize_mode(mode);

            obj.p = p;
            obj.mode = spec.mode_number;
            obj.firmware_mode = spec.firmware_mode;
            obj.mode_name = spec.name;
            opt = sim_options();
            obj.options = opt.default_options(options);

            if strcmp(spec.name, 'mpc')
                obj.options.balance_loop_frequency = 100.0;
                % オブザーバをMPCのバランスループ周波数に合わせて再設計
                [A_lin, B_lin, C_lin, ~] = linearize_system(p);
                obs = design_observer(A_lin, B_lin, C_lin, 1.0 / obj.options.balance_loop_frequency);
                obj.options.observer.Ad = obs.Ad;
                obj.options.observer.Bd = obs.Bd;
                obj.options.observer.Ld = obs.Ld;
            end

            if isempty(ctrl_param)
                obj.ctrl_param = opt.default_controller_param(p, spec);
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

                % 外乱力の加算
                disturbance_force = FirmwareExperiment.eval_disturbance(opts.disturbance, t);
                total_force = applied_force + disturbance_force;

                state = FirmwareExperiment.rk4_step( ...
                    @(x, u) plant_model(0, x, u, obj.p), state, total_force, current_dt);

                hist.t(k) = t;
                hist.x(k, :) = state.';
                hist.force(k) = total_force;
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

            util = sim_utils();
            runtime.theta_filter = util.init_lpf(balance_dt, obj.theta_filter_cutoff());
            runtime.theta_dot_filter = util.init_lpf(balance_dt, obj.theta_dot_filter_cutoff());
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
                    pid_api = step_pid();
                    runtime.balance_pid = pid_api.init( ...
                        obj.ctrl_param, balance_dt, opts.max_force);
                case 'mrac'
                    mrac_api = step_mrac();
                    runtime.mrac = mrac_api.init(obj.ctrl_param);
                case 'mpc'
                    runtime.mpc_state = [];  % ADMM warm-start (初回は空)
            end

            runtime.current_filter = util.init_lpf(current_dt, opts.current_filter_cutoff);
            runtime.current_pid = FirmwareExperiment.init_pid( ...
                opts.current_pid, current_dt, -opts.max_voltage, opts.max_voltage);
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
            util = sim_utils();
            position_r = (qei_r - runtime.qei_r_offset) * FirmwareExperiment.pulse_to_position(obj.p);
            position_l = -(qei_l - runtime.qei_l_offset) * FirmwareExperiment.pulse_to_position(obj.p);
            velocity_r = (qei_r - runtime.qei_r_prev) * FirmwareExperiment.pulse_to_position(obj.p) / balance_dt;
            velocity_l = -(qei_l - runtime.qei_l_prev) * FirmwareExperiment.pulse_to_position(obj.p) / balance_dt;

            runtime.qei_r_prev = qei_r;
            runtime.qei_l_prev = qei_l;

            position = (position_r + position_l) / 2.0;
            velocity = (velocity_r + velocity_l) / 2.0;

            runtime.theta_filter = util.lpf_update(runtime.theta_filter, theta_meas);
            theta = runtime.theta_filter.output;

            if runtime.theta_initialized
                theta_dot_raw = (theta - runtime.prev_theta) / balance_dt;
            else
                runtime.theta_initialized = true;
                theta_dot_raw = 0.0;
            end
            runtime.prev_theta = theta;
            runtime.theta_dot_filter = util.lpf_update(runtime.theta_dot_filter, theta_dot_raw);
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
                    pid_api = step_pid();
                    [force, runtime.balance_pid] = pid_api.step( ...
                        runtime.balance_pid, processed);
                    runtime.target_force = util.clamp(force, -obj.options.max_force, obj.options.max_force);
                    runtime.target_current = runtime.target_force * FirmwareExperiment.force_to_current(obj.p);
                    runtime.target_voltage = 0.0;
                    velocity_est = processed.velocity;
                    theta_dot_est = processed.theta_dot;
                    current_used = NaN;
                    current_state = NaN;

                case 'lqr'
                    x_ctrl = [position; observer_velocity; theta; observer_theta_dot];
                    lqr_api = step_lqr();
                    force = lqr_api.step(obj.ctrl_param, x_ctrl);
                    runtime.target_force = util.clamp(force, -obj.options.max_force, obj.options.max_force);
                    runtime.target_current = runtime.target_force * FirmwareExperiment.force_to_current(obj.p);
                    runtime.target_voltage = 0.0;
                    velocity_est = observer_velocity;
                    theta_dot_est = observer_theta_dot;
                    current_used = NaN;
                    current_state = NaN;

                case 'mrac'
                    processed = struct( ...
                        'position', position, ...
                        'velocity', observer_velocity, ...
                        'theta', theta, ...
                        'theta_dot', observer_theta_dot);
                    mrac_api = step_mrac();
                    [force, runtime.mrac] = mrac_api.step( ...
                        runtime.mrac, processed, balance_dt, obj.options.max_force);
                    runtime.target_force = util.clamp(force, -obj.options.max_force, obj.options.max_force);
                    runtime.target_current = runtime.target_force * FirmwareExperiment.force_to_current(obj.p);
                    runtime.target_voltage = 0.0;
                    velocity_est = processed.velocity;
                    theta_dot_est = processed.theta_dot;
                    current_used = NaN;
                    current_state = NaN;

                case 'mpc'
                    x_ctrl = [position; observer_velocity; theta; observer_theta_dot];
                    mpc_api = step_mpc();
                    [force, runtime.mpc_state] = mpc_api.step( ...
                        obj.ctrl_param, x_ctrl, runtime.mpc_state);
                    runtime.target_force = util.clamp(force, -obj.options.max_force, obj.options.max_force);
                    runtime.target_current = runtime.target_force * FirmwareExperiment.force_to_current(obj.p);
                    runtime.target_voltage = 0.0;
                    velocity_est = observer_velocity;
                    theta_dot_est = observer_theta_dot;
                    current_used = NaN;
                    current_state = NaN;

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
            util = sim_utils();
            p = obj.p;
            opts = obj.options;

            signed_current = FirmwareExperiment.correct_current_sign( ...
                abs(runtime.current_actual), runtime.motor_prev_voltage);
            runtime.current_filter = util.lpf_update(runtime.current_filter, signed_current);
            runtime.current_ctrl = runtime.current_filter.output;
            [voltage_cmd, runtime.current_pid] = FirmwareExperiment.pid_update( ...
                runtime.current_pid, runtime.target_current, runtime.current_ctrl);
            duty = util.clamp(voltage_cmd / opts.vin, -1.0, 1.0);
            motor_voltage = duty * opts.vin;
            runtime.motor_prev_voltage = motor_voltage;

            if opts.enable_motor_electrical_dynamics
                motor_speed = p.G * cart_velocity / p.r;
                runtime.current_actual = FirmwareExperiment.step_motor_current( ...
                    runtime.current_actual, motor_voltage, motor_speed, p, current_dt);
            else
                runtime.current_actual = (motor_voltage - p.Ke * p.G * cart_velocity / p.r) / p.Ra;
            end

            applied_force = 2.0 * p.G * p.Kt * runtime.current_actual / p.r;
            if opts.enable_force_saturation
                applied_force = util.clamp(applied_force, -opts.force_limit, opts.force_limit);
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

        function next = rk4_step(f, state, input, dt)
            k1 = f(state, input);
            k2 = f(state + dt / 2 * k1, input);
            k3 = f(state + dt / 2 * k2, input);
            k4 = f(state + dt * k3, input);
            next = state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
        end

        function f = eval_disturbance(disturbance, t)
            % 外乱スケジュールの評価
            %   disturbance: struct配列 with fields: time, duration, force
            %   各要素は time <= t < time+duration の間に force [N] を加える
            f = 0.0;
            if isempty(disturbance)
                return;
            end
            for i = 1:numel(disturbance)
                d = disturbance(i);
                if t >= d.time && t < d.time + d.duration
                    f = f + d.force;
                end
            end
        end
    end
end
