function p = params()
% PARAMS 台車型倒立振子のシステムパラメータ
% パラメータ推定値 (InvertedPendulum/MATLAB/estimate_parameters/parameters.mat 由来)
% M, bx は design_feedback_controller.m でのオーバーライド値を採用

    % 機械系
    p.M  = 0.57;          % 台車質量 [kg] (オーバーライド値、元値: 0.699)
    p.m  = 0.0303;        % 振子質量 [kg]
    p.l  = 0.15;          % 振子長さ（回転軸から重心） [m]
    p.r  = 0.0255;        % タイヤ半径 [m]
    p.g  = 9.81;          % 重力加速度 [m/s^2]
    p.Jp = 2.2725e-4;     % 振子慣性モーメント [kg*m^2]
    p.Iw = 0;             % タイヤ慣性モーメント [kg*m^2] (≈0)
    p.bx = 0.0;           % 台車粘性摩擦係数 [N*s/m] (オーバーライド値)
    p.btheta = 1.67926e-3; % 振子粘性摩擦係数 [N*m*s/rad]

    % モーター
    p.Kt = 0.01857;       % トルク定数 [N*m/A]
    p.Ke = 0.01857;       % 逆起電力定数 [V*s/rad]
    p.Ra = 32.4;          % 電機子抵抗 [Ohm]
    p.La = 2.955e-3;      % 電機子インダクタンス [H]
    p.G  = 6.666;         % ギア比
end
