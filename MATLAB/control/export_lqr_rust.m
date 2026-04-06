function export_lqr_rust()
% EXPORT_LQR_RUST Print Rust constants for the firmware LQR controller.

    p = params();
    [A, B, ~, ~] = linearize_system(p);
    K = design_lqr(A, B);

    fprintf('// Designed in MATLAB/control/design_lqr.m\n');
    fprintf('// Q = diag([100, 0, 50, 100]), R = 10\n');
    print_rust_scalar('K_POSITION', -K(1));
    print_rust_scalar('K_VELOCITY', -K(2));
    print_rust_scalar('K_ANGLE', -K(3));
    print_rust_scalar('K_ANGULAR_VELOCITY', -K(4));
end

function print_rust_scalar(name, value)
    fprintf('const %s: f32 = %.9g;\n', name, value);
end
