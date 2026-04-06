function [A, B, C, D] = linearize_voltage_input(p)
% LINEARIZE_VOLTAGE_INPUT Linearized plant for direct-voltage control.
%   State: x = [position; velocity; theta; theta_dot; current]
%   Input: u = motor voltage [V]
%
%   The mechanical subsystem is the existing force-input linearization.
%   Force is generated from armature current:
%       F = 2 * G * Kt / r * i
%   Armature current dynamics are:
%       i_dot = (v - Ke * G / r * x_dot - Ra * i) / La

    [A_force, B_force, ~, ~] = linearize_system(p);

    force_per_current = 2.0 * p.G * p.Kt / p.r;
    current_back_emf = p.Ke * p.G / (p.La * p.r);

    A = [A_force, B_force * force_per_current;
         0.0, -current_back_emf, 0.0, 0.0, -p.Ra / p.La];
    B = [0.0; 0.0; 0.0; 0.0; 1.0 / p.La];

    C = eye(5);
    D = zeros(5, 1);
end
