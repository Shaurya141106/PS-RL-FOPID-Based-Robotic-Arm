%% Step 2: Actuator Energy, Torque RMS, Chatter, and Energy Profiles
clear; clc; close all;

% 1. Physical Parameters (URDF)
params.m1 = 0.25975;
params.m2 = 0.22196;
params.m3_base = 0.57884;
params.mp_full  = 0.300; % 300g unknown liquid load
params.mp_empty = 0.010; % 10g after dropping
params.t_drop   = 1.5;
params.L1 = 0.2947; params.L2 = 0.2500; params.L3 = 0.0400;
params.lc1 = 0.1419; params.lc2 = 0.1268; params.lc3 = 0.0380;
params.I1 = 2.1385e-3; params.I2 = 1.3141e-3; params.I3 = 3.1123e-4;
params.g  = 9.81;

% 2. Trajectory (2.0s fast motion)
traj.q_start = [deg2rad(10); deg2rad(30); deg2rad(-40)];
traj.q_end   = [deg2rad(60); deg2rad(20); deg2rad(-80)];
traj.tf      = 2.0;
dt = 0.001;
t = 0:dt:3.5;
N = length(t);

% 3. Equal Base Gains (FIXED: Kd tuned to realistic critical damping for low-inertia links)
base_gains.Kp = [50.0; 35.0; 12.0];
base_gains.Ki = [30.0; 20.0;  6.0];
base_gains.Kd = [2.5;   1.5;  0.5]; 

% 4. Run Energy Benchmarking Simulations
fprintf('Calculating Adaptive Fuzzy PID energy and torques...\n');
[q_fuzzy, tau_fuzzy, e_fuzzy, rms_fuzzy, chatter_fuzzy, e_prof_fuzzy] = run_energy_sim(params, traj, t, dt, 'FUZZY_PID', base_gains);

fprintf('Calculating PS-RL-FOPID energy and torques...\n');
[q_psrl, tau_psrl, e_psrl, rms_psrl, chatter_psrl, e_prof_psrl] = run_energy_sim(params, traj, t, dt, 'PS_RL_FOPID', base_gains);

% 5. Print Formatted Energy & Effort Journal Table
fprintf('\n======================== ACTUATOR ENERGY & CONTROL EFFORT ========================\n');
fprintf('%-18s | %-16s | %-16s | %-16s\n', 'Controller', 'Total Energy [J]', 'Avg RMS Torque [Nm]', 'Torque Chatter [Nm]');
fprintf('----------------------------------------------------------------------------------\n');
fprintf('%-18s | %-16.2f | %-16.3f | %-16.2f\n', 'Adaptive Fuzzy PID', e_fuzzy, mean(rms_fuzzy), chatter_fuzzy);
fprintf('%-18s | %-16.2f | %-16.3f | %-16.2f\n', 'PS-RL-FOPID', e_psrl, mean(rms_psrl), chatter_psrl);
fprintf('==================================================================================\n\n');

% 6. Plot 1: Motor Torques
figure('Color', [1 1 1], 'Name', 'Actuator Torque Profiles', 'Position', [100 100 950 750]);

% Shoulder Joint Torque
subplot(3,1,1);
plot(t, tau_fuzzy(:,1), 'Color', [0 0 0.5], 'LineStyle', '-', 'LineWidth', 1.5); hold on;
plot(t, tau_psrl(:,1), 'g-', 'LineWidth', 2.0);
xline(params.t_drop, 'y:', 'LineWidth', 1.5, 'Label', '300g Mass Drop', 'Color', 'k');
grid on; 
ylabel('\tau_1 (Shoulder) [Nm]', 'Color', 'k');
legend('Adaptive Fuzzy PID', 'PS-RL-FOPID', 'Location', 'northeast', 'TextColor', 'k');
title('Actuator Torque Profiles & Control Effort Under 300g Disturbance', 'Color', 'k');
set(gca, 'Color', [1 1 1], 'XColor', 'k', 'YColor', 'k');

% Elbow Joint Torque
subplot(3,1,2);
plot(t, tau_fuzzy(:,2), 'Color', [0 0 0.5], 'LineStyle', '-', 'LineWidth', 1.5); hold on;
plot(t, tau_psrl(:,2), 'g-', 'LineWidth', 2.0);
xline(params.t_drop, 'y:', 'LineWidth', 1.5, 'Color', 'k');
grid on; 
ylabel('\tau_2 (Elbow) [Nm]', 'Color', 'k');
legend('Adaptive Fuzzy PID', 'PS-RL-FOPID', 'Location', 'northeast', 'TextColor', 'k');
set(gca, 'Color', [1 1 1], 'XColor', 'k', 'YColor', 'k');

% Wrist Joint Torque
subplot(3,1,3);
plot(t, tau_fuzzy(:,3), 'Color', [0 0 0.5], 'LineStyle', '-', 'LineWidth', 1.5); hold on;
plot(t, tau_psrl(:,3), 'g-', 'LineWidth', 2.0);
xline(params.t_drop, 'y:', 'LineWidth', 1.5, 'Color', 'k');
grid on; 
xlabel('Time [s]', 'Color', 'k'); ylabel('\tau_3 (Wrist) [Nm]', 'Color', 'k');
legend('Adaptive Fuzzy PID', 'PS-RL-FOPID', 'Location', 'northeast', 'TextColor', 'k');
set(gca, 'Color', [1 1 1], 'XColor', 'k', 'YColor', 'k');

% 7. Plot 2: Cumulative Energy Profile
figure('Color', [1 1 1], 'Name', 'Energy Profiles', 'Position', [150 150 800 400]);
plot(t, e_prof_fuzzy, 'Color', [0 0 0.5], 'LineStyle', '-', 'LineWidth', 2.0); hold on;
plot(t, e_prof_psrl, 'g-', 'LineWidth', 2.0);
xline(params.t_drop, 'y:', 'LineWidth', 1.5, 'Label', '300g Mass Drop', 'Color', 'k');
grid on; 
xlabel('Time [s]', 'Color', 'k'); ylabel('Cumulative Energy [J]', 'Color', 'k');
legend('Adaptive Fuzzy PID', 'PS-RL-FOPID', 'Location', 'northwest', 'TextColor', 'k');
title('Cumulative System Actuator Energy Consumption', 'Color', 'k');
set(gca, 'Color', [1 1 1], 'XColor', 'k', 'YColor', 'k');

%% Simulation Engine with Energy Computation
function [q_out, tau_out, total_energy, rms_torque, chatter_idx, energy_prof] = run_energy_sim(p, tr, t_vec, dt, mode, ctrl)
    N = length(t_vec);
    q = tr.q_start; dq = zeros(3,1);
    q_out = zeros(N, 3);
    tau_out = zeros(N, 3);
    
    L_mem = 200;
    e_hist = zeros(L_mem, 3);
    
    % Fixed Fractional Orders
    lambda = 0.90; mu = 0.70;
    int_scale = dt^lambda; diff_scale = dt^(-mu);
    
    % Integer Integrator Memory
    e_int_classic = zeros(3,1);
    
    w_int = zeros(L_mem, 1); w_diff = zeros(L_mem, 1);
    w_int(1) = 1; w_diff(1) = 1;
    for j = 2:L_mem
        w_int(j)  = w_int(j-1) * (1 - (-lambda + 1)/(j-1));
        w_diff(j) = w_diff(j-1) * (1 - (mu + 1)/(j-1));
    end
    
    % Nominal model assumption (assumes ONLY 50g baseline)
    p_nom = p;
    p_nom.m3 = p.m3_base + 0.050; 
    
    for k = 1:N
        tk = t_vec(k);
        [qd, dqd, ~] = quintic_eval(tk, tr.q_start, tr.q_end, tr.tf);
        
        e = qd - q;
        de = dqd - dq;
        e_hist = [e'; e_hist(1:end-1, :)];
        
        tau_ff = compute_G(q, p_nom);
        
        if strcmp(mode, 'FUZZY_PID')
            % Structural anti-windup
            e_int_classic = max(min(e_int_classic + e * dt, 0.5), -0.5);
            
            % Stable Fuzzy Rules scaling ONLY P and D
            gain_scale_p = 1.0 + 0.8 * (1 - exp(-(abs(e).^2) / 0.05));
            gain_scale_d = 1.0 + 0.5 * (1 - exp(-(abs(de).^2) / 0.1));
            
            tau_fb = (ctrl.Kp .* gain_scale_p) .* e + ...
                     ctrl.Ki .* e_int_classic + ...
                     (ctrl.Kd .* gain_scale_d) .* de;
                     
        elseif strcmp(mode, 'PS_RL_FOPID')
            % Neural DDPG Surrogate mapping for Proportional and Derivative
            sig_e = 1.0 ./ (1.0 + exp(-5.0 * (abs(e) - 0.01)));
            sig_0 = 1.0 / (1.0 + exp(0.05)); 
            adapt_p = 1.0 + 4.5 * ((sig_e - sig_0) ./ (1.0 - sig_0));
            adapt_d = 1.0 + 2.5 * tanh(5.0 * abs(de));
            
            % Fixed Fractional Integrator and Differentiator
            frac_int  = (sum(w_int .* e_hist, 1)') * int_scale;
            frac_diff = (sum(w_diff .* e_hist, 1)') * diff_scale;
            
            tau_fb = (ctrl.Kp .* adapt_p) .* e + ...
                     ctrl.Ki .* frac_int + ...
                     (ctrl.Kd .* adapt_d) .* frac_diff;
        end
        
        % Saturation limits
        tau = max(min(tau_ff + tau_fb, 35), -35);
        tau_out(k, :) = tau';
        
        if tk < p.t_drop
            p.m3 = p.m3_base + p.mp_full;
        else
            p.m3 = p.m3_base + p.mp_empty;
        end
        
        M = compute_M(q, p);
        G = compute_G(q, p);
        ddq = M \ (tau - 0.05*dq - G);
        
        dq = dq + ddq * dt;
        q  = q + dq * dt;
        q_out(k, :) = q';
    end
    
    % Metrics Calculations
    energy_prof = cumsum(sum(tau_out.^2, 2)) * dt;
    total_energy = energy_prof(end);
    
    rms_torque = sqrt(mean(tau_out.^2, 1))';
    
    d_tau = diff(tau_out, 1, 1);
    chatter_idx = sum(sum(abs(d_tau)));
end

%% Helpers
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