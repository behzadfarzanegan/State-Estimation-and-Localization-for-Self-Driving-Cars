function [x_r, y_r, V_r, psi_r, dotpsi_r] = reference(k, T)

    % Constants
    A = 1.5; % Amplitude of trajectory
    omega = 0.4; % Frequency of oscillation
    phi = 0.2; % Phase shift

    % Reference trajectory equations
    x_r = A * sin(omega * k * T + phi) + 0.7;
    y_r = A * (1 - cos(omega * k * T + phi)) - 1.5;

    % First derivatives
    dotx_r = A * omega * cos(omega * k * T + phi);
    doty_r = A * omega * sin(omega * k * T + phi);

    % Second derivatives
    dot2x_r = -A * omega^2 * sin(omega * k * T + phi);
    dot2y_r = A * omega^2 * cos(omega * k * T + phi);

    % Compute reference speed
    V_r = sqrt(dotx_r^2 + doty_r^2);

    % Compute reference heading (yaw angle)
    psi_r = atan2(doty_r, dotx_r);

    % Compute yaw rate, preventing division by zero
    if V_r == 0
        dotpsi_r = 0;
    else
        dotpsi_r = (dot2y_r * dotx_r - dot2x_r * doty_r) / V_r^2;
    end

end

%Adaptive Trajectory Tracking for Car-Like Vehicles With Input Constraints


% function [x_r, y_r, V_r, psi_r, dotpsi_r] = reference(k, T)
%     % Example: Circular trajectory
%     omega_ref = 0.1;  % Reference angular velocity
%     V_r = 1;          % Constant reference velocity
%     x_r = V_r * k * T * cos(omega_ref * k * T);
%     y_r = V_r * k * T * sin(omega_ref * k * T);
%     psi_r = omega_ref * k * T;
%     dotpsi_r = omega_ref;
% end
