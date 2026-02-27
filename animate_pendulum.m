function animate_pendulum(t, x, p)
% ANIMATE_PENDULUM 台車型倒立振子のアニメーション
%   t: 時間ベクトル
%   x: 状態行列 [N x 4]
%   p: システムパラメータ

    L = 2 * p.l;  % 振子の描画長さ

    % 描画間隔（実時間の20倍速程度に間引く）
    skip = max(1, round(0.02 / (t(2) - t(1))));

    fig = figure('Name', 'Inverted Pendulum Animation', 'NumberTitle', 'off');

    x_range = max(abs(x(:,1))) + L + 0.5;

    for k = 1:skip:length(t)
        if ~isvalid(fig)
            break;
        end

        cart_x = x(k, 1);
        theta  = x(k, 3);

        % 振子先端の位置
        pend_x = cart_x + L * sin(theta);
        pend_y = L * cos(theta);

        clf;
        hold on;

        % 台車（長方形）
        cart_w = 0.4;
        cart_h = 0.2;
        rectangle('Position', [cart_x - cart_w/2, -cart_h/2, cart_w, cart_h], ...
                  'FaceColor', [0.3 0.5 0.8], 'EdgeColor', 'k', 'LineWidth', 1.5);

        % 振子（棒）
        plot([cart_x, pend_x], [0, pend_y], 'r-', 'LineWidth', 3);

        % 振子先端（丸）
        plot(pend_x, pend_y, 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');

        % 地面
        plot([-x_range, x_range], [-cart_h/2, -cart_h/2], 'k-', 'LineWidth', 1);

        % 軸設定
        xlim([-x_range, x_range]);
        ylim([-1, L + 0.5]);
        axis equal;
        grid on;
        title(sprintf('t = %.2f s', t(k)));
        xlabel('x [m]');
        ylabel('y [m]');

        drawnow;
    end
end
