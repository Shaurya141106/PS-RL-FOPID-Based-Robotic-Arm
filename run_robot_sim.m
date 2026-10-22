%% Dynamic Simulation of 3-DOF Assistive Arm (URDF Parameters)
clear; clc; close all;

% 1. Define Model Parameters (Extracted directly from URDF)
params.m1 = 0.25975;
params.m2 = 0.22196;
params.m3_base = 0.57884; 
params.mp = 0.200;        % 200 ml of liquid/food payload
params.m3 = params.m3_base + params.mp;

params.L1 = 0.2947;
params.L2 = 0.2500;
params.L3 = 0.0400;

params.lc1 = 0.1419;
params.lc2 = 0.1268;
params.lc3 = 0.0380;

params.I1 = 2.1385e-3;
params.I2 = 1.3141e-3;
params.I3 = 3.1123e-4;

params.g = 9.81; % Acceleration due to gravity (m/s^2)

% 2. Simulation Settings
% State vector x = [q1; q2; q3; dq1; dq2; dq3]
x0 = [deg2rad(15); deg2rad(30); deg2rad(-45); 0; 0; 0]; 
tspan = [0 4];

% 3. Run Dynamics (Testing uncontrolled free swing under gravity: tau = 0)
[t, x] = ode45(@(t, x) robot_dynamics(t, x, [0; 0; 0], params), tspan, x0);

% 4. Plot Joint Motions
figure('Color', 'w', 'Name', 'Joint Angles vs Time');
subplot(3,1,1);
plot(t, rad2deg(x(:,1)), 'LineWidth', 2, 'Color', 'b');
grid on; ylabel('q_1 (Shoulder) [deg]'); title('3-DOF Free-Fall Gravity Response');

subplot(3,1,2);
plot(t, rad2deg(x(:,2)), 'LineWidth', 2, 'Color', [0.85 0.325 0.098]);
grid on; ylabel('q_2 (Elbow) [deg]');

subplot(3,1,3);
plot(t, rad2deg(x(:,3)), 'LineWidth', 2, 'Color', [0.466 0.674 0.188]);
grid on; xlabel('Time (s)'); ylabel('q_3 (Wrist) [deg]');

%% 3-DOF Nonlinear Dynamic Equations Function
function dxdt = robot_dynamics(~, x, tau, p)
q1 = x(1); q2 = x(2); q3 = x(3);
dq1 = x(4); dq2 = x(5); dq3 = x(6);

% Mass Matrix M(q)
M = zeros(3,3);
M(1,1) = p.I1 + p.I2 + p.I3 + p.m1*p.lc1^2 + ...
    p.m2*(p.L1^2 + p.lc2^2 + 2*p.L1*p.lc2*cos(q2)) + ...
    p.m3*(p.L1^2 + p.L2^2 + p.lc3^2 + 2*p.L1*p.L2*cos(q2) + ...
    2*p.L2*p.lc3*cos(q3) + 2*p.L1*p.lc3*cos(q2 + q3));

M(1,2) = p.I2 + p.I3 + p.m2*(p.lc2^2 + p.L1*p.lc2*cos(q2)) + ...
    p.m3*(p.L2^2 + p.lc3^2 + p.L1*p.L2*cos(q2) + ...
    2*p.L2*p.lc3*cos(q3) + p.L1*p.lc3*cos(q2 + q3));

M(1,3) = p.I3 + p.m3*(p.lc3^2 + p.L2*p.lc3*cos(q3) + p.L1*p.lc3*cos(q2 + q3));

M(2,1) = M(1,2);
M(2,2) = p.I2 + p.I3 + p.m2*p.lc2^2 + ...
    p.m3*(p.L2^2 + p.lc3^2 + 2*p.L2*p.lc3*cos(q3));
M(2,3) = p.I3 + p.m3*(p.lc3^2 + p.L2*p.lc3*cos(q3));

M(3,1) = M(1,3);
M(3,2) = M(2,3);
M(3,3) = p.I3 + p.m3*p.lc3^2;

% Coriolis & Centrifugal Matrix C(q, dq)*dq
% Approximate damping/velocity coupling
B_viscous = 0.05; % Joint friction damping
C_dq = [B_viscous*dq1; B_viscous*dq2; B_viscous*dq3];

% Gravity Vector G(q)
G = zeros(3,1);
g1 = (p.m1*p.lc1 + (p.m2 + p.m3)*p.L1)*p.g*cos(q1) + ...
    (p.m2*p.lc2 + p.m3*p.L2)*p.g*cos(q1 + q2) + ...
    p.m3*p.lc3*p.g*cos(q1 + q2 + q3);

g2 = (p.m2*p.lc2 + p.m3*p.L2)*p.g*cos(q1 + q2) + ...
    p.m3*p.lc3*p.g*cos(q1 + q2 + q3);

g3 = p.m3*p.lc3*p.g*cos(q1 + q2 + q3);

G = [g1; g2; g3];

% Compute Accelerations: ddq = inv(M) * (tau - C*dq - G)
dq = [dq1; dq2; dq3];
ddq = M \ (tau - C_dq - G);

dxdt = [dq; ddq];
end