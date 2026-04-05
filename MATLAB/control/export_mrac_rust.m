function export_mrac_rust()
% EXPORT_MRAC_RUST Print Rust constants for the firmware MRAC controller.

    p = params();
    mrac = design_mrac(p);

    fprintf('// Designed in MATLAB/control/design_mrac.m\n');
    print_rust_vector('MRAC_KX', mrac.Kx);
    print_rust_matrix('MRAC_AM', mrac.Am);
    print_rust_matrix('MRAC_P', mrac.P);
    print_rust_vector('MRAC_PB', mrac.PB);
    print_rust_vector('MRAC_GAMMA_DIAG', diag(mrac.GammaDiag));
    print_rust_scalar('MRAC_SIGMA', mrac.Sigma);
    print_rust_scalar('MRAC_NORMALIZATION_EPS', mrac.NormalizationEps);
    print_rust_scalar('MRAC_MAX_VOLTAGE', mrac.MaxVoltage);
    print_rust_scalar('MRAC_MAX_VOLTAGE_SLEW_RATE', mrac.MaxVoltageSlewRate);
    print_rust_scalar('MRAC_ADAPTATION_DELAY_SEC', mrac.AdaptationDelaySec);
    print_rust_scalar('MRAC_ERROR_DEADZONE', mrac.ErrorDeadzone);
    print_rust_scalar('MRAC_MIN_ACTIVE_VOLTAGE', mrac.MinActiveVoltage);
    print_rust_scalar('MRAC_MIN_DRIVE_VOLTAGE', mrac.MinDriveVoltage);
    print_rust_scalar('MRAC_LOW_SPEED_VELOCITY_THRESHOLD', mrac.LowSpeedVelocityThreshold);
    print_rust_scalar('MRAC_STICTION_ENABLE_ANGLE_LIMIT', mrac.StictionEnableAngleLimit);
    print_rust_scalar('MRAC_STICTION_ENABLE_ANGULAR_VELOCITY_LIMIT', mrac.StictionEnableAngularVelocityLimit);
    print_rust_scalar('MRAC_THETA_SOFT_ZONE', mrac.ThetaSoftZone);
    print_rust_scalar('MRAC_THETA_DOT_SOFT_ZONE', mrac.ThetaDotSoftZone);
    print_rust_scalar('MRAC_INNER_ANGLE_GAIN', mrac.InnerAngleGain);
    print_rust_scalar('MRAC_INNER_ANGULAR_VELOCITY_GAIN', mrac.InnerAngularVelocityGain);
    print_rust_scalar('MRAC_CURRENT_SOFT_ZONE', mrac.CurrentSoftZone);
    print_rust_scalar('MRAC_INNER_CURRENT_GAIN', mrac.InnerCurrentGain);
    print_rust_scalar('MRAC_MAX_ADAPTIVE_GAIN', mrac.MaxAdaptiveGain);
    print_rust_scalar('MRAC_ENABLE_ANGLE_LIMIT', mrac.EnableAngleLimit);
    print_rust_scalar('MRAC_ENABLE_ANGULAR_VELOCITY_LIMIT', mrac.EnableAngularVelocityLimit);
    print_rust_scalar('MRAC_ENABLE_CURRENT_LIMIT', mrac.EnableCurrentLimit);
end

function print_rust_matrix(name, matrix)
    [rows, cols] = size(matrix);
    fprintf('const %s: [[f32; %d]; %d] = [\n', name, cols, rows);
    for r = 1:rows
        fprintf('    [');
        for c = 1:cols
            fprintf('%.9g', matrix(r, c));
            if c < cols
                fprintf(', ');
            end
        end
        fprintf('],\n');
    end
    fprintf('];\n\n');
end

function print_rust_vector(name, vector)
    vector = vector(:);
    fprintf('const %s: [f32; %d] = [', name, numel(vector));
    for idx = 1:numel(vector)
        fprintf('%.9g', vector(idx));
        if idx < numel(vector)
            fprintf(', ');
        end
    end
    fprintf('];\n\n');
end

function print_rust_scalar(name, value)
    fprintf('const %s: f32 = %.9g;\n\n', name, value);
end
