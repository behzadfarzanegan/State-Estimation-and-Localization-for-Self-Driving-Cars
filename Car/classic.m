clear all;
close all;
clc;

% Simulation Parameters
T = 0.01;
tf = 18;
N = tf / T;  % Number of steps

% Initial Reference
[x_r(1), y_r(1), V_r(1), psi_r(1), dotpsi_r(1)] = reference(0, T);
xr(:,1) = [x_r(1), y_r(1), psi_r(1)]';
x(:,1) = [-0.128, 0, 0]';
xe(:,1) = xr(:,1) - x(:,1);

% Controller Gains
k1 = 1.98;
k2 = 1.98;
k3 = 1.98;

% Vehicle Parameters
l = 0.256;
u1_max = 1.5;
u2_max = 0.5;

% Control Inputs
V(1) = 2.8175;
omega(1) = 0;
dotV_r(1) = 0;

for k = 2:N
    % Compute state error using car dynamics
    xe(:,k) = car(T, dotpsi_r(k-1), V_r(k-1), V(k-1), omega(k-1), xe(:,k-1));

    % Update reference trajectory
    [x_r(k), y_r(k), V_r(k), psi_r(k), dotpsi_r(k)] = reference(k, T);
    xr(:,k) = [x_r(k), y_r(k), psi_r(k)]';

    % Compute velocity control
    V(k) = V_r(k) * cos(xe(3,k)) + k1 * xe(1,k);
    V(k) = min(max(V(k), 0), u1_max);  % Ensure V is within bounds
    dotV_r(k) = (V_r(k) - V_r(k-1)) / T;

    % Compute steering control (prevent division by zero)
    tildey = xe(2,k) + k2/V_r(k) * sin(xe(3,k));
    denominator = xe(1,k) + k2/V_r(k) * cos(xe(3,k));
    
    if abs(denominator) < 1e-6
        denominator = 1e-6;
    end

    omega(k) = (k3 * tildey + 2 * V_r(k) * sin(xe(3,k)) + k2/V_r(k) * cos(xe(3,k)) * dotpsi_r(k) - k2/V_r(k) * sin(xe(3,k)) * dotV_r(k)) ...
               / denominator;

    % Compute actual vehicle position
    psi = mod(xr(3,k) - xe(3,k) + pi, 2*pi) - pi;  % Normalize psi

    Rot = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
    x(:,k) = xr(:,k) - Rot' * xe(:,k);  % More stable than inv(Rot)

end

% Plot results
figure(1);
plot(xe(1,:), 'b', 'LineWidth', 1.5);
xlabel('Time Step'); ylabel('X Error'); title('X Position Error'); grid on;

figure(2);
plot(xe(2,:), 'r', 'LineWidth', 1.5);
xlabel('Time Step'); ylabel('Y Error'); title('Y Position Error'); grid on;

figure(3);
plot(x(1,:), x(2,:), 'g', 'LineWidth', 1.5);
hold on;
plot(xr(1,:), xr(2,:), '--r', 'LineWidth', 1.5);
xlabel('X Position'); ylabel('Y Position'); title('Vehicle Trajectory');
legend('Actual Trajectory', 'Reference Trajectory');
grid on;
axis equal;

% Improved Omega Plot with Time Steps
t = (0:N-1) * T;
figure(4);
plot(t, omega, 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Steering Angle Rate (\omega)');
title('Control Input: Omega');
grid on;
