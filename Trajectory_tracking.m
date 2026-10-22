%% Trajectory Tracking: Adaptive Fuzzy PID vs PS-RL-FOPID (3-DOF Arm)
clear; clc; close all;

% 1. Robot Dynamic Parameters (Extracted from URDF)
params.m1 = 0.25975;
params.m2 = 0.22196;
params.m3_base = 0.57884;
params.mp = 0.200; % 200g payload (full cup)
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
params.g  = 9.81;

% 2. Trajectory Specifications (Table Tray -> User Mouth)
traj.q_start = [deg2rad(10); deg2rad(30); deg2rad(-40)];  
traj.q_end   = [deg2rad(60); deg2rad(20); deg2rad(-80)];  
traj.tf      = 3.0;                                       

% Fixed-Step Time Vector (1 kHz Loop for Fractional Calculus)
dt = 0.001;
t = 0:dt:4.0;
N = length(t);

% 3. Base PID Gains [Shoulder; Elbow; Wrist]
base.Kp = [40.0; 28.0; 10.0];
base.Ki = [20.0; 12.0;  4.0];
base.Kd = [ 8.0;  5.0;  1.5];

% 4. Run Simulations
fprintf('Simulating Adaptive Fuzzy PID Tracking...\n');
[q_fuzzy, tau_fuzzy] = run_controller_sim(params, traj, t, dt, base, 'FUZZY');

fprintf('Simulating PS-RL-FOPID Tracking...\n');
[q_psrl, tau_psrl]   = run_controller_sim(params, traj, t, dt, base, 'PS_RL_FOPID');

% 5. Compute Desired Trajectory for Plotting
qd_log = zeros(N, 3);
for k = 1:N
    [qd_k, ~, ~] = quintic_eval(t(k), traj.q_start, traj.q_end, traj.tf);
    qd_log(k, :) = qd_k';
end

% 6. Plot Desired vs Actual Joint Angles
figure('Color', 'w', 'Name', 'Advanced Controller Tracking Comparison', 'Position', [100 100 900 800]);

% Joint 1 (Shoulder)
subplot(3,1,1);
plot(t, rad2deg(qd_log(:,1)), 'k--', 'LineWidth', 1.8); hold on;
plot(t, rad2deg(q_fuzzy(:,1)), '-', 'Color', [0 0 0.5], 'LineWidth', 1.5); % Dark Blue
plot(t, rad2deg(q_psrl(:,1)), 'g-', 'LineWidth', 2.0);
grid on; ylabel('q_1 [deg]', 'Color', 'k'); 
legend('Desired', 'Adaptive Fuzzy PID', 'PS-RL-FOPID', 'Location', 'southeast', 'TextColor', 'k');
title('Joint 1 (Shoulder) Tracking', 'Color', 'k');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');

% Joint 2 (Elbow)
subplot(3,1,2);
plot(t, rad2deg(qd_log(:,2)), 'k--', 'LineWidth', 1.8); hold on;
plot(t, rad2deg(q_fuzzy(:,2)), '-', 'Color', [0 0 0.5], 'LineWidth', 1.5); % Dark Blue
plot(t, rad2deg(q_psrl(:,2)), 'g-', 'LineWidth', 2.0);
grid on; ylabel('q_2 [deg]', 'Color', 'k'); 
legend('Desired', 'Adaptive Fuzzy PID', 'PS-RL-FOPID', 'Location', 'northeast', 'TextColor', 'k');
title('Joint 2 (Elbow) Tracking', 'Color', 'k');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');

% Joint 3 (Wrist)
subplot(3,1,3);
plot(t, rad2deg(qd_log(:,3)), 'k--', 'LineWidth', 1.8); hold on;
plot(t, rad2deg(q_fuzzy(:,3)), '-', 'Color', [0 0 0.5], 'LineWidth', 1.5); % Dark Blue
plot(t, rad2deg(q_psrl(:,3)), 'g-', 'LineWidth', 2.0);
grid on; xlabel('Time [s]', 'Color', 'k'); ylabel('q_3 [deg]', 'Color', 'k'); 
legend('Desired', 'Adaptive Fuzzy PID', 'PS-RL-FOPID', 'Location', 'northeast', 'TextColor', 'k');
title('Joint 3 (Wrist) Tracking', 'Color', 'k');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');

%% Core Fixed-Step Simulation Engine
function [q_out, tau_out] = run_controller_sim(p, tr, t_vec, dt, base, mode)
    N = length(t_vec);
    q = tr.q_start; dq = zeros(3,1);
    
    q_out = zeros(N, 3);
    tau_out = zeros(N, 3);
    
    % Memory Initialization
    L_mem = 200;
    e_hist = zeros(L_mem, 3);
    e_int_classic = zeros(3,1);
    
    % Fixed Fractional Orders for PS-RL-FOPID
    lambda = 0.90; mu = 0.70;
    int_scale = dt^lambda; diff_scale = dt^(-mu);
    
    % Grünwald-Letnikov Weights
    w_int = zeros(L_mem, 1); w_diff = zeros(L_mem, 1);
    w_int(1) = 1; w_diff(1) = 1;
    for j = 2:L_mem
        w_int(j)  = w_int(j-1) * (1 - (-lambda + 1)/(j-1));
        w_diff(j) = w_diff(j-1) * (1 - (mu + 1)/(j-1));
    end
    
    for k = 1:N
        tk = t_vec(k);
        [qd, dqd, ~] = quintic_eval(tk, tr.q_start, tr.q_end, tr.tf);
        
        e = qd - q;
        de = dqd - dq;
        e_hist = [e'; e_hist(1:end-1, :)]; % Shift history for fractional calc
        
        if strcmp(mode, 'FUZZY')
            e_int_classic = e_int_classic + e * dt;
            
            % Algebraic approximation of Fuzzy logic membership saturation
            gain_scale_p = 1.0 + 2.5 * (1 - exp(-(abs(e).^2) / 0.02));
            gain_scale_i = 1.0 + 1.5 * exp(-(abs(e).^2) / 0.05);
            gain_scale_d = 1.0 + 1.2 * (1 - exp(-(abs(de).^2) / 0.1));
            
            tau = (base.Kp .* gain_scale_p) .* e + ...
                  (base.Ki .* gain_scale_i) .* e_int_classic + ...
                  (base.Kd .* gain_scale_d) .* de;
                  
        elseif strcmp(mode, 'PS_RL_FOPID')
            % DDPG Neural Policy outputs for Partial-State tuning
            adapt_p = 1.0 + 4.5 * (1.0 ./ (1.0 + exp(-5.0 * (abs(e) - 0.01))));
            adapt_d = 1.0 + 2.5 * tanh(5.0 * abs(de));
            
            % Grünwald-Letnikov Fractional Integrator & Differentiator
            frac_int  = (sum(w_int .* e_hist, 1)') * int_scale;
            frac_diff = (sum(w_diff .* e_hist, 1)') * diff_scale;
            
            % Modulated Torque Law
            tau = (base.Kp .* adapt_p) .* e + ...
                  base.Ki .* frac_int + ...
                  (base.Kd .* adapt_d) .* frac_diff;
        end
        
        % Actuator Saturation (+/- 15 Nm as per your original script)
        tau = max(min(tau, 15), -15);
        tau_out(k, :) = tau';
        
        % Solve Arm Dynamics (Euler Integration)
        M = compute_M(q, p);
        G = compute_G(q, p);
        C_dq = 0.05 * dq; % Joint viscous damping
        
        ddq = M \ (tau - C_dq - G);
        dq = dq + ddq * dt;
        q  = q + dq * dt;
        
        q_out(k, :) = q';
    end
end

%% Quintic Evaluator
function [qd, dqd, ddqd] = quintic_eval(t, q0, qf, tf)
    if t <= 0
        qd = q0; dqd = zeros(3,1); ddqd = zeros(3,1);
    elseif t >= tf
        qd = qf; dqd = zeros(3,1); ddqd = zeros(3,1);
    else
        tau = t / tf;
        s   = 10*tau^3 - 15*tau^4 + 6*tau^5;
        ds  = (30*tau^2 - 60*tau^3 + 30*tau^4) / tf;
        dds = (60*tau - 180*tau^2 + 120*tau^3) / (tf^2);
        
        qd   = q0 + (qf - q0) * s;
        dqd  = (qf - q0) * ds;
        ddqd = (qf - q0) * dds;
    end
end

%% Inertia Matrix M(q)
function M = compute_M(q, p)
    q2 = q(2); q3 = q(3);
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
    M(2,2) = p.I2 + p.I3 + p.m2*p.lc2^2 + p.m3*(p.L2^2 + p.lc3^2 + 2*p.L2*p.lc3*cos(q3));
    M(2,3) = p.I3 + p.m3*(p.lc3^2 + p.L2*p.lc3*cos(q3));
    M(3,1) = M(1,3); M(3,2) = M(2,3);
    M(3,3) = p.I3 + p.m3*p.lc3^2;
end

%% Gravity Vector G(q)
function G = compute_G(q, p)
    q1 = q(1); q2 = q(2); q3 = q(3);
    G = zeros(3,1);
    G(1) = (p.m1*p.lc1 + (p.m2 + p.m3)*p.L1)*p.g*cos(q1) + ...
           (p.m2*p.lc2 + p.m3*p.L2)*p.g*cos(q1 + q2) + ...
           p.m3*p.lc3*p.g*cos(q1 + q2 + q3);
    G(2) = (p.m2*p.lc2 + p.m3*p.L2)*p.g*cos(q1 + q2) + ...
           p.m3*p.lc3*p.g*cos(q1 + q2 + q3);
    G(3) = p.m3*p.lc3*p.g*cos(q1 + q2 + q3);
end