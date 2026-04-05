function analyze_mode4_failure()
% ANALYZE_MODE4_FAILURE Compare the redesigned augmented mode4 variants.

    p = params();
    c = design_mrac(p);
    x0 = [0; 0; deg2rad(5); 0];

    fw = simulate_firmware_mode4(p, x0, 5.0, c);
    fw_no_quant = simulate_firmware_mode4(p, x0, 5.0, c, struct( ...
        'enable_encoder_quantization', false, ...
        'enable_theta_quantization', false));
    fw_ideal_motor = simulate_firmware_mode4(p, x0, 5.0, c, struct( ...
        'enable_motor_electrical_dynamics', false));

    fprintf('\n=== Mode4 Analysis ===\n');
    print_fw_result('Firmware-like', fw);
    print_fw_result('Firmware-like no quantization', fw_no_quant);
    print_fw_result('Firmware-like ideal motor', fw_ideal_motor);

    figure('Name', 'Mode4 Failure Analysis', 'NumberTitle', 'off', ...
        'Position', [100, 100, 1100, 800]);

    subplot(4,1,1);
    h1 = plot(fw.t, rad2deg(fw.x(:,3)), 'LineWidth', 1.2); hold on;
    h2 = plot(fw_no_quant.t, rad2deg(fw_no_quant.x(:,3)), 'LineWidth', 1.2);
    h3 = plot(fw_ideal_motor.t, rad2deg(fw_ideal_motor.x(:,3)), 'LineWidth', 1.2);
    ylabel('theta [deg]');
    legend([h1 h2 h3], {'fw', 'fw no quant', 'fw ideal motor'});
    grid on;

    subplot(4,1,2);
    h1 = plot(fw.t, fw.x(:,1), 'LineWidth', 1.2); hold on;
    h2 = plot(fw_no_quant.t, fw_no_quant.x(:,1), 'LineWidth', 1.2);
    h3 = plot(fw_ideal_motor.t, fw_ideal_motor.x(:,1), 'LineWidth', 1.2);
    ylabel('x [m]');
    legend([h1 h2 h3], {'fw', 'fw no quant', 'fw ideal motor'});
    grid on;

    subplot(4,1,3);
    h1 = plot(fw.t, fw.target_voltage, 'LineWidth', 1.2); hold on;
    h2 = plot(fw.t, fw.voltage, 'LineWidth', 1.2);
    ylabel('voltage [V]');
    legend([h1 h2], {'target', 'applied'});
    grid on;

    subplot(4,1,4);
    h1 = plot(fw.t, fw.current, 'LineWidth', 1.2); hold on;
    h2 = plot(fw.t, fw.current_est, 'LineWidth', 1.2);
    ylabel('current [A]');
    xlabel('time [s]');
    legend([h1 h2], {'actual', 'estimated'});
    grid on;
end

function print_fw_result(name, r)
    fprintf('\n[%s]\n', name);
    fprintf('  max theta: %.3f deg\n', max(abs(rad2deg(r.x(:,3)))));
    fprintf('  max x:     %.4f m\n', max(abs(r.x(:,1))));
    fprintf('  max volt:  %.4f V\n', max(abs(r.voltage)));
    fprintf('  final th:  %.3f deg\n', rad2deg(r.x(end,3)));
end
