function export_pid_rust()
% EXPORT_PID_RUST Print Rust constants for the firmware PID balance controller.

    eu = export_utils();
    pid_gains = design_pid();

    fprintf('// Designed in MATLAB/control/design_pid.m\n');
    fprintf('// 角度制御（内側ループ）\n');
    eu.print_scalar('BALANCE_ANGLE_KP', pid_gains.theta.Kp);
    eu.print_scalar('BALANCE_ANGLE_KI', pid_gains.theta.Ki);
    eu.print_scalar('BALANCE_ANGLE_KD', pid_gains.theta.Kd);
    fprintf('\n');
    fprintf('// 位置制御（外側ループ）\n');
    eu.print_scalar('BALANCE_POS_KP', pid_gains.x.Kp);
    eu.print_scalar('BALANCE_POS_KI', pid_gains.x.Ki);
    eu.print_scalar('BALANCE_POS_KD', pid_gains.x.Kd);
end
