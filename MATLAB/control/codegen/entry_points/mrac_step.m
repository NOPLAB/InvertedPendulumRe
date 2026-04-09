function force = mrac_step(c, dt, max_force, state)  %#codegen
% MRAC_STEP Model Reference Adaptive Controller (codegen entry point).
%   c:         struct — MRAC parameters from design_mrac()
%   dt:        single — balance loop period [s]
%   max_force: single — force saturation limit [N]
%   state:     single(4x1) — [x; x_dot; theta; theta_dot]
%   force:     single       — control force [N]

    assert(isa(state, 'single'));
    assert(all(size(state) == [4 1]));

    persistent ref_state adaptive_gains initialized startup_timer

    if isempty(initialized)
        ref_state = zeros(4, 1, 'single');
        adaptive_gains = zeros(5, 1, 'single');
        initialized = false;
        startup_timer = single(0);
    end

    x = state;
    phi = [x; single(1)];

    if ~initialized
        ref_state = x;
        initialized = true;
        startup_timer = single(0);
    else
        ref_state = ref_state + dt * (c.Am * ref_state);
        startup_timer = startup_timer + dt;
    end

    error_val = x - ref_state;
    s = error_val' * c.PB;
    phi_norm = phi' * phi;

    adaptation_enabled = ...
        abs(state(3)) < c.EnableAngleLimit && ...
        abs(state(4)) < c.EnableAngularVelocityLimit && ...
        abs(state(1)) < c.EnablePositionLimit && ...
        abs(state(2)) < c.EnableVelocityLimit && ...
        startup_timer >= c.AdaptationDelaySec && ...
        abs(s) > c.ErrorDeadzone;

    if adaptation_enabled
        scale = s / (c.NormalizationEps + phi_norm);
        for idx = 1:5
            update_val = c.GammaDiag(idx) * phi(idx) * scale ...
                - c.Sigma * adaptive_gains(idx);
            adaptive_gains(idx) = adaptive_gains(idx) + dt * update_val;
            adaptive_gains(idx) = max(min(adaptive_gains(idx), ...
                c.MaxAdaptiveGain), -c.MaxAdaptiveGain);
        end
    else
        adaptive_gains = adaptive_gains + dt * (-c.Sigma * adaptive_gains);
    end

    adaptive_force = max(min(adaptive_gains' * phi, ...
        c.MaxAdaptiveForce), -c.MaxAdaptiveForce);
    nominal_force = -(c.Kx * x);
    force = max(min(nominal_force - adaptive_force, max_force), -max_force);
end
