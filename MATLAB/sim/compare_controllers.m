function compare_controllers(t_lqr, x_lqr, u_lqr, t_pid, x_pid, u_pid)
% COMPARE_CONTROLLERS LQR vs PID の性能比較プロット

    figure('Name', 'Controller Comparison', 'NumberTitle', 'off', ...
           'Position', [100, 100, 900, 700]);

    % 振子角度
    subplot(3, 1, 1);
    plot(t_lqr, rad2deg(x_lqr(:,3)), 'b-', 'LineWidth', 1.5); hold on;
    plot(t_pid, rad2deg(x_pid(:,3)), 'r--', 'LineWidth', 1.5);
    ylabel('Angle [deg]');
    title('Pendulum Angle');
    legend('LQR', 'PID');
    grid on;

    % 台車位置
    subplot(3, 1, 2);
    plot(t_lqr, x_lqr(:,1), 'b-', 'LineWidth', 1.5); hold on;
    plot(t_pid, x_pid(:,1), 'r--', 'LineWidth', 1.5);
    ylabel('Position [m]');
    title('Cart Position');
    legend('LQR', 'PID');
    grid on;

    % 制御入力
    subplot(3, 1, 3);
    plot(t_lqr, u_lqr, 'b-', 'LineWidth', 1.5); hold on;
    plot(t_pid, u_pid, 'r--', 'LineWidth', 1.5);
    ylabel('Force [N]');
    xlabel('Time [s]');
    title('Control Input');
    legend('LQR', 'PID');
    grid on;

    % 性能指標を表示
    fprintf('\n=== Performance Comparison ===\n');
    print_metrics('LQR', t_lqr, x_lqr);
    print_metrics('PID', t_pid, x_pid);
end

function print_metrics(name, t, x)
    theta_deg = rad2deg(x(:, 3));

    % 整定時間（角度が±0.5度以内に収まる最初の時刻）
    last_outside_idx = find(abs(theta_deg) > 0.5, 1, 'last');
    if isempty(last_outside_idx)
        ts = t(1);
    elseif last_outside_idx < numel(t)
        ts = t(last_outside_idx + 1);
    else
        ts = NaN;
    end

    % 最大オーバーシュート（初期角度からの逆側への振れ）
    overshoot = max(abs(theta_deg));

    fprintf('\n[%s]\n', name);
    fprintf('  Settling time (±0.5 deg): %.3f s\n', ts);
    fprintf('  Max angle:                %.2f deg\n', overshoot);
    fprintf('  Max cart displacement:    %.3f m\n', max(abs(x(:,1))));
end
