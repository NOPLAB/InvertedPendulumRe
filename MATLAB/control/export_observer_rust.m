function export_observer_rust()
% EXPORT_OBSERVER_RUST Print Rust constants for the firmware observer.

    p = params();
    [A, B, C, ~] = linearize_system(p);
    obs = design_observer(A, B, C);

    fprintf('// Designed in MATLAB/control/design_observer.m\n');
    print_rust_matrix('AD', obs.Ad);
    print_rust_vector('BD', obs.Bd);
    print_rust_matrix('LD', obs.Ld);
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
