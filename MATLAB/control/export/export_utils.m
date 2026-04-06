function util = export_utils()
% EXPORT_UTILS Shared Rust code generation helpers.
%   util = export_utils();
%   util.print_scalar(name, value)
%   util.print_vector(name, vector)
%   util.print_matrix(name, matrix)

    util.print_scalar = @print_rust_scalar;
    util.print_vector = @print_rust_vector;
    util.print_matrix = @print_rust_matrix;
end

function print_rust_scalar(name, value)
    fprintf('const %s: f32 = %.9g;\n\n', name, value);
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
