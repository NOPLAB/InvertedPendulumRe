function export_observer_rust()
% EXPORT_OBSERVER_RUST Print Rust constants for the firmware observer.

    eu = export_utils();
    p = params();
    [A, B, C, ~] = linearize_system(p);
    obs = design_observer(A, B, C);

    fprintf('// Designed in MATLAB/control/design_observer.m\n');
    eu.print_matrix('AD', obs.Ad);
    eu.print_vector('BD', obs.Bd);
    eu.print_matrix('LD', obs.Ld);
end
