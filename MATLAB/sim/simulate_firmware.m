function result = simulate_firmware(p, x0, t_end, mode, ctrl_param, options)
% SIMULATE_FIRMWARE Generic firmware-like simulator for mode1-mode4.
%   mode:
%     1 / 'mode1' / 'debug'
%     2 / 'mode2' / 'pid'
%     3 / 'mode3' / 'lqr'
%     4 / 'mode4' / 'mrac'
%
%   ctrl_param can be omitted to use the default controller design.

    if nargin < 6
        options = struct();
    end
    if nargin < 5
        ctrl_param = [];
    end

    experiment = FirmwareExperiment(p, mode, ctrl_param, options);
    result = experiment.run(x0, t_end);
end
