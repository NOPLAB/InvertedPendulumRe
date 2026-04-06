function api = step_mrac()
% STEP_MRAC Model Reference Adaptive Control.
%   api = step_mrac();
%   controller = api.init(ctrl_param);
%   [force, controller] = api.step(controller, state, dt, max_force);

    api.init = @init_mrac_controller;
    api.step = @mrac_step;
end

function controller = init_mrac_controller(c)
    controller.reference_state = zeros(4, 1);
    controller.adaptive_gains = zeros(5, 1);
    controller.initialized = false;
    controller.startup_timer = 0.0;
    controller.ctrl = c;
end

function [force, controller] = mrac_step(controller, state, dt, max_force)
    util = sim_utils();
    c = controller.ctrl;

    x = [ ...
        state.position; ...
        state.velocity; ...
        state.theta; ...
        state.theta_dot];
    phi = [x; 1.0];

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
    phi_norm = phi' * phi;

    adaptation_enabled = abs(state.theta) < c.EnableAngleLimit ...
        && abs(state.theta_dot) < c.EnableAngularVelocityLimit ...
        && abs(state.position) < c.EnablePositionLimit ...
        && abs(state.velocity) < c.EnableVelocityLimit ...
        && controller.startup_timer >= c.AdaptationDelaySec ...
        && abs(s) > c.ErrorDeadzone;

    if adaptation_enabled
        scale = s / (c.NormalizationEps + phi_norm);
        for idx = 1:5
            update = c.GammaDiag(idx, idx) * phi(idx) * scale ...
                - c.Sigma * controller.adaptive_gains(idx);
            controller.adaptive_gains(idx) = controller.adaptive_gains(idx) + dt * update;
            controller.adaptive_gains(idx) = util.clamp( ...
                controller.adaptive_gains(idx), -c.MaxAdaptiveGain, c.MaxAdaptiveGain);
        end
    else
        controller.adaptive_gains = controller.adaptive_gains + dt * (-c.Sigma * controller.adaptive_gains);
    end

    adaptive_force = util.clamp( ...
        controller.adaptive_gains' * phi, -c.MaxAdaptiveForce, c.MaxAdaptiveForce);
    nominal_force = -(c.Kx * x);
    force = util.clamp(nominal_force - adaptive_force, -max_force, max_force);
end
