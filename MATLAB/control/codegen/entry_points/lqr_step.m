function force = lqr_step(K, state)  %#codegen
% LQR_STEP LQR state feedback controller (codegen entry point).
%   K:     single(1x4) — LQR gain vector
%   state: single(4x1) — [x; x_dot; theta; theta_dot]
%   force: single       — control force [N]

    assert(isa(K, 'single'));
    assert(all(size(K) == [1 4]));
    assert(isa(state, 'single'));
    assert(all(size(state) == [4 1]));

    force = -(K * state);
end
