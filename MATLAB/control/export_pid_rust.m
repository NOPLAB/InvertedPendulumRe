function export_pid_rust()
% EXPORT_PID_RUST Print Rust constants for the firmware PID balance controller.

    pid_gains = design_pid();

    fprintf('// Designed in MATLAB/control/design_pid.m\n');
    fprintf('// 角度制御（内側ループ）\n');
    print_rust_scalar('BALANCE_ANGLE_KP', pid_gains.theta.Kp);
    print_rust_scalar('BALANCE_ANGLE_KI', pid_gains.theta.Ki);
    print_rust_scalar('BALANCE_ANGLE_KD', pid_gains.theta.Kd);
    fprintf('\n');
    fprintf('// 位置制御（外側ループ）\n');
    print_rust_scalar('BALANCE_POS_KP', pid_gains.x.Kp);
    print_rust_scalar('BALANCE_POS_KI', pid_gains.x.Ki);
    print_rust_scalar('BALANCE_POS_KD', pid_gains.x.Kd);
end

function print_rust_scalar(name, value)
    fprintf('const %s: f32 = %.9g;\n', name, value);
end
