function uncertain_params = create_uncertain_params(p_nominal, uncertainty)
% CREATE_UNCERTAIN_PARAMS Build ureal parameter definitions for robustness sweeps.
%   Only parameters that affect the current simulator are included by default.

    uncertain_params = struct();

    percentage_names = fieldnames(uncertainty.percentage);
    for i = 1:numel(percentage_names)
        name = percentage_names{i};
        uncertain_params.(name) = ureal( ...
            name, p_nominal.(name), 'Percentage', uncertainty.percentage.(name));
    end

    plus_minus_names = fieldnames(uncertainty.plus_minus);
    for i = 1:numel(plus_minus_names)
        name = plus_minus_names{i};
        uncertain_params.(name) = ureal( ...
            name, p_nominal.(name), 'PlusMinus', uncertainty.plus_minus.(name));
    end
end
