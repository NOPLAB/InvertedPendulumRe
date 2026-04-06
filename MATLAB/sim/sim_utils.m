function util = sim_utils()
% SIM_UTILS Shared utility functions for the firmware simulator.
    util.init_lpf = @init_lpf;
    util.lpf_update = @lpf_update;
    util.merge_structs = @merge_structs;
    util.normalize_mode = @normalize_mode;
    util.clamp = @clamp;
    util.lpf_alpha = @lpf_alpha;
end

function lpf = init_lpf(sample_time, cutoff_freq)
    lpf.alpha = lpf_alpha(cutoff_freq, sample_time);
    lpf.output = 0.0;
    lpf.initialized = false;
end

function lpf = lpf_update(lpf, input)
    if ~lpf.initialized
        lpf.output = input;
        lpf.initialized = true;
    else
        lpf.output = lpf.alpha * input + (1 - lpf.alpha) * lpf.output;
    end
end

function out = merge_structs(base, override)
    out = base;
    if isempty(override)
        return;
    end

    fields = fieldnames(override);
    for idx = 1:numel(fields)
        name = fields{idx};
        if isfield(out, name) && isstruct(out.(name)) && isstruct(override.(name))
            out.(name) = merge_structs(out.(name), override.(name));
        else
            out.(name) = override.(name);
        end
    end
end

function spec = normalize_mode(mode)
    if isstring(mode) || ischar(mode)
        key = lower(strtrim(char(mode)));
        switch key
            case {'mode1', 'debug'}
                spec = struct('mode_number', 1, 'firmware_mode', 0, 'name', 'debug');
            case {'mode2', 'pid'}
                spec = struct('mode_number', 2, 'firmware_mode', 1, 'name', 'pid');
            case {'mode3', 'lqr'}
                spec = struct('mode_number', 3, 'firmware_mode', 2, 'name', 'lqr');
            case {'mode4', 'mrac'}
                spec = struct('mode_number', 4, 'firmware_mode', 3, 'name', 'mrac');
            case {'mode5', 'mpc'}
                spec = struct('mode_number', 5, 'firmware_mode', 4, 'name', 'mpc');
            otherwise
                error('sim_utils:InvalidMode', 'Unsupported mode: %s', char(mode));
        end
        return;
    end

    if isnumeric(mode) && isscalar(mode)
        switch double(mode)
            case 1
                spec = struct('mode_number', 1, 'firmware_mode', 0, 'name', 'debug');
            case 2
                spec = struct('mode_number', 2, 'firmware_mode', 1, 'name', 'pid');
            case 3
                spec = struct('mode_number', 3, 'firmware_mode', 2, 'name', 'lqr');
            case 4
                spec = struct('mode_number', 4, 'firmware_mode', 3, 'name', 'mrac');
            case 5
                spec = struct('mode_number', 5, 'firmware_mode', 4, 'name', 'mpc');
            otherwise
                error('sim_utils:InvalidMode', ...
                    'Numeric mode must be one of 1, 2, 3, 4, 5.');
        end
        return;
    end

    error('sim_utils:InvalidMode', 'mode must be a string or scalar number.');
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function alpha = lpf_alpha(cutoff_freq, sample_time)
    tau = 1 / (2 * pi * cutoff_freq);
    alpha = sample_time / (tau + sample_time);
end
