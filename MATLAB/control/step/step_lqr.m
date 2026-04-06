function api = step_lqr()
% STEP_LQR LQR state feedback controller.
%   api = step_lqr();
%   force = api.step(ctrl_param, x_ctrl);

    api.step = @lqr_step;
end

function force = lqr_step(ctrl_param, x_ctrl)
    gain = reshape(ctrl_param, 1, []);
    force = -(gain * x_ctrl);
end
