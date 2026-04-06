function export_mrac_rust()
% EXPORT_MRAC_RUST Print Rust constants for the firmware MRAC controller.

    eu = export_utils();
    p = params();
    mrac = design_mrac(p);

    fprintf('// Designed in MATLAB/control/design_mrac.m\n');
    eu.print_vector('MRAC_KX', mrac.Kx);
    eu.print_matrix('MRAC_AM', mrac.Am);
    eu.print_vector('MRAC_PB', mrac.PB);
    eu.print_vector('MRAC_GAMMA_DIAG', diag(mrac.GammaDiag));
    eu.print_scalar('MRAC_SIGMA', mrac.Sigma);
    eu.print_scalar('MRAC_NORMALIZATION_EPS', mrac.NormalizationEps);
    eu.print_scalar('MRAC_ADAPTATION_DELAY_SEC', mrac.AdaptationDelaySec);
    eu.print_scalar('MRAC_ERROR_DEADZONE', mrac.ErrorDeadzone);
    eu.print_scalar('MRAC_MAX_ADAPTIVE_GAIN', mrac.MaxAdaptiveGain);
    eu.print_scalar('MRAC_MAX_ADAPTIVE_FORCE', mrac.MaxAdaptiveForce);
    eu.print_scalar('MRAC_ENABLE_ANGLE_LIMIT', mrac.EnableAngleLimit);
    eu.print_scalar('MRAC_ENABLE_ANGULAR_VELOCITY_LIMIT', mrac.EnableAngularVelocityLimit);
    eu.print_scalar('MRAC_ENABLE_POSITION_LIMIT', mrac.EnablePositionLimit);
    eu.print_scalar('MRAC_ENABLE_VELOCITY_LIMIT', mrac.EnableVelocityLimit);
end
