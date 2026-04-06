function api = step_mpc()
% STEP_MPC MPC controller step (wraps mpc_controller).
%   api = step_mpc();
%   [force, mpc_state] = api.step(ctrl_param, x_ctrl, mpc_state);

    api.step = @mpc_step;
end

function [force, mpc_state] = mpc_step(ctrl_param, x_ctrl, mpc_state)
    [force, mpc_state] = mpc_controller(ctrl_param, x_ctrl, mpc_state);
end
