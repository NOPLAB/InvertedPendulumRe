function pid_gains = design_pid()
% DESIGN_PID PID制御器のゲイン設定
%   角度θおよび台車位置xに対するPID制御

    % 角度制御（内側ループ）
    pid_gains.theta.Kp = 50;
    pid_gains.theta.Ki = 5;
    pid_gains.theta.Kd = 15;

    % 位置制御（外側ループ）
    pid_gains.x.Kp = 10.0;
    pid_gains.x.Ki = 0.5;
    pid_gains.x.Kd = 15.0;
end
