function sampled_params = sample_uncertain_params( ...
    uncertain_params, p_nominal, sample_count, seed)
% SAMPLE_UNCERTAIN_PARAMS Draw numeric parameter sets from ureal definitions.

    if nargin < 4
        seed = 'shuffle';
    end

    previous_rng = rng;
    cleanup = onCleanup(@() rng(previous_rng)); %#ok<NASGU>
    rng(seed);

    param_names = fieldnames(uncertain_params);
    sampled_values = struct();

    for i = 1:numel(param_names)
        name = param_names{i};
        values = usample(uncertain_params.(name), sample_count);
        sampled_values.(name) = reshape(double(values), 1, []);
    end

    sampled_params = cell(sample_count, 1);
    for k = 1:sample_count
        p_sample = p_nominal;
        for i = 1:numel(param_names)
            name = param_names{i};
            p_sample.(name) = sampled_values.(name)(k);
        end
        sampled_params{k} = p_sample;
    end
end
