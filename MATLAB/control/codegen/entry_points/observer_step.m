function x_hat = observer_step(Ad, Bd, Ld, position, theta, force)  %#codegen
% OBSERVER_STEP Discrete Luenberger state observer (codegen entry point).
%   Ad:       single(4x4) — discrete state transition matrix
%   Bd:       single(4x1) — discrete input matrix
%   Ld:       single(4x2) — observer gain matrix
%   position: single       — measured cart position [m]
%   theta:    single       — measured pendulum angle [rad]
%   force:    single       — applied force [N] (previous step)
%   x_hat:    single(4x1)  — estimated state vector

    assert(isa(position, 'single'));
    assert(isa(theta, 'single'));
    assert(isa(force, 'single'));

    persistent state_hat prev_force initialized

    if isempty(initialized)
        state_hat = zeros(4, 1, 'single');
        prev_force = single(0);
        initialized = false;
    end

    if ~initialized
        state_hat = single([position; 0; theta; 0]);
        prev_force = force;
        initialized = true;
        x_hat = state_hat;
        return;
    end

    % イノベーション
    y_meas = single([position; theta]);
    y_hat = single([state_hat(1); state_hat(3)]);
    innovation = y_meas - y_hat;

    % 状態更新: x_hat[k+1] = Ad * x_hat[k] + Bd * u[k] + Ld * innovation
    state_hat = Ad * state_hat + Bd * prev_force + Ld * innovation;
    prev_force = force;

    x_hat = state_hat;
end
