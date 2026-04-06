function export_mrac_rust()
% EXPORT_MRAC_RUST Print Rust constants for the firmware MRAC controller.

    eu = export_utils();
    p = params();
    mrac = design_mrac(p);

    fprintf('// Designed in MATLAB/control/design_mrac.m\n');
    eu.print_vector('MRAC_KX', mrac.Kx);
    eu.print_matrix('MRAC_AM', mrac.Am);
    eu.print_matrix('MRAC_P', mrac.P);
    eu.print_vector('MRAC_PB', mrac.PB);
    eu.print_vector('MRAC_GAMMA_DIAG', diag(mrac.GammaDiag));
    eu.print_scalar('MRAC_SIGMA', mrac.Sigma);
    eu.print_scalar('MRAC_NORMALIZATION_EPS', mrac.NormalizationEps);
    eu.print_scalar('MRAC_MAX_VOLTAGE', mrac.MaxVoltage);
    eu.print_scalar('MRAC_MAX_VOLTAGE_SLEW_RATE', mrac.MaxVoltageSlewRate);
    eu.print_scalar('MRAC_ADAPTATION_DELAY_SEC', mrac.AdaptationDelaySec);
    eu.print_scalar('MRAC_ERROR_DEADZONE', mrac.ErrorDeadzone);
    eu.print_scalar('MRAC_MIN_ACTIVE_VOLTAGE', mrac.MinActiveVoltage);
    eu.print_scalar('MRAC_MIN_DRIVE_VOLTAGE', mrac.MinDriveVoltage);
    eu.print_scalar('MRAC_LOW_SPEED_VELOCITY_THRESHOLD', mrac.LowSpeedVelocityThreshold);
    eu.print_scalar('MRAC_STICTION_ENABLE_ANGLE_LIMIT', mrac.StictionEnableAngleLimit);
    eu.print_scalar('MRAC_STICTION_ENABLE_ANGULAR_VELOCITY_LIMIT', mrac.StictionEnableAngularVelocityLimit);
    eu.print_scalar('MRAC_THETA_SOFT_ZONE', mrac.ThetaSoftZone);
    eu.print_scalar('MRAC_THETA_DOT_SOFT_ZONE', mrac.ThetaDotSoftZone);
    eu.print_scalar('MRAC_INNER_ANGLE_GAIN', mrac.InnerAngleGain);
    eu.print_scalar('MRAC_INNER_ANGULAR_VELOCITY_GAIN', mrac.InnerAngularVelocityGain);
    eu.print_scalar('MRAC_CURRENT_SOFT_ZONE', mrac.CurrentSoftZone);
    eu.print_scalar('MRAC_INNER_CURRENT_GAIN', mrac.InnerCurrentGain);
    eu.print_scalar('MRAC_MAX_ADAPTIVE_GAIN', mrac.MaxAdaptiveGain);
    eu.print_scalar('MRAC_ENABLE_ANGLE_LIMIT', mrac.EnableAngleLimit);
    eu.print_scalar('MRAC_ENABLE_ANGULAR_VELOCITY_LIMIT', mrac.EnableAngularVelocityLimit);
    eu.print_scalar('MRAC_ENABLE_CURRENT_LIMIT', mrac.EnableCurrentLimit);
end
