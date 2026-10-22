%% 3-Way Benchmark: Integer PID vs Adaptive Fuzzy PID vs PS-RL-FOPID
clear; clc; close all;

% 1. Physical Model Parameters (URDF)
params.m1 = 0.25975;
params.m2 = 0.22196;
params.m3_base = 0.57884;
params.mp_full  = 0.220; % 220g liquid (True initial state)
params.mp_empty = 0.015; % 15g left after sip
params.t_drop   = 1.8;   % Unannounced sip time
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

% 2. Feeding Trajectory
traj.q_start = [deg2rad(10); deg2rad(30); deg2rad(-40)];
traj.q_end   = [deg2rad(60); deg2rad(20); deg2rad(-80)];
traj.tf      = 3.0;

% 3. Time Discretization
dt = 0.001;
t = 0:dt:4.5;
N = length(t);

% 4. Controllers Configuration
% All controllers now strictly use the same base gains to prevent actuator saturation
base_gains.Kp = [50.0; 35.0; 12.0];
base_gains.Ki = [30.0; 20.0;  6.0];
base_gains.Kd = [10.0;  7.0;  2.0];

% 5. Run Simulations
fprintf('1. Simulating Integer PID...\n');
[q_pid, y_pid, ita_pid, iae_pid] = run_benchmark_sim(params, traj, t, dt, 'PID', base_gains);

fprintf('2. Simulating Adaptive Fuzzy PID...\n');
[q_fuzzy, y_fuzzy, ita_fuzzy, iae_fuzzy] = run_benchmark_sim(params, traj, t, dt, 'FUZZY_PID', base_gains);

fprintf('3. Simulating PS-RL-FOPID (Proposed)...\n');
[q_psrl, y_psrl, ita_psrl, iae_psrl] = run_benchmark_sim(params, traj, t, dt, 'PS_RL_FOPID', base_gains);

% 6. Compute Desired Trajectory
qd_all = zeros(N, 3);
y_des = zeros(N, 1);
for k = 1:N
    [qd_k, ~, ~] = quintic_eval(t(k), traj.q_start, traj.q_end, traj.tf);
    qd_all(k,:) = qd_k';
    y_des(k) = params.L1*sin(qd_k(1)) + params.L2*sin(qd_k(1)+qd_k(2)) + params.L3*sin(sum(qd_k));
end

% 7. Print Performance Comparison Table
fprintf('\n======================== BENCHMARK PERFORMANCE METRICS ========================\n');
fprintf('%-25s | %-12s | %-12s | %-15s\n', 'Controller', 'ITAE', 'IAE', 'Peak Overshoot');
fprintf('-------------------------------------------------------------------------------\n');
fprintf('%-25s | %-12.4f | %-12.4f | %-15.2f cm\n', 'Integer PID', ita_pid, iae_pid, (max(y_pid) - y_des(end))*100);
fprintf('%-25s | %-12.4f | %-12.4f | %-15.2f cm\n', 'Adaptive Fuzzy PID', ita_fuzzy, iae_fuzzy, (max(y_fuzzy) - y_des(end))*100);
fprintf('%-25s | %-12.4f | %-12.4f | %-15.2f cm\n', 'PS-RL-FOPID (Proposed)', ita_psrl, iae_psrl, (max(y_psrl) - y_des(end))*100);
fprintf('===============================================================================\n\n');

% 8. Plot Tracking
figure('Color', 'w', 'Name', '3-Way Controller Comparison', 'Position', [100 100 900 800]);

% Shoulder
subplot(3,1,1);
plot(t, rad2deg(qd_all(:,1)), 'k--', 'LineWidth', 1.8); hold on;
plot(t, rad2deg(q_pid(:,1)), 'r-', 'LineWidth', 1.6);
plot(t, rad2deg(q_fuzzy(:,1)), '-', 'Color', [0 0 0.5], 'LineWidth', 1.6); 
plot(t, rad2deg(q_psrl(:,1)), 'g-', 'LineWidth', 2.0);
xline(params.t_drop, 'k:', 'LineWidth', 1.5, 'Label', 'Payload Sip (1.8s)', 'Color', 'k');
grid on; ylabel('q_1 [deg]', 'Color', 'k');
legend('Desired', 'Integer PID', 'Adaptive Fuzzy PID', 'PS-RL-FOPID', 'Location', 'southeast', 'TextColor', 'k');
title('Joint 1 (Shoulder) Tracking Under Unmodeled Payload Shift', 'Color', 'k');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');

% Elbow
subplot(3,1,2);
plot(t, rad2deg(qd_all(:,2)), 'k--', 'LineWidth', 1.8); hold on;
plot(t, rad2deg(q_pid(:,2)), 'r-', 'LineWidth', 1.6);
plot(t, rad2deg(q_fuzzy(:,2)), '-', 'Color', [0 0 0.5], 'LineWidth', 1.6); 
plot(t, rad2deg(q_psrl(:,2)), 'g-', 'LineWidth', 2.0);
xline(params.t_drop, 'k:', 'LineWidth', 1.5, 'Color', 'k');
grid on; ylabel('q_2 [deg]', 'Color', 'k');
legend('Desired', 'Integer PID', 'Adaptive Fuzzy PID', 'PS-RL-FOPID', 'Location', 'northeast', 'TextColor', 'k');
title('Joint 2 (Elbow) Tracking', 'Color', 'k');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');

% Vertical Safe-Zone Height
subplot(3,1,3);
plot(t, y_des*100, 'k--', 'LineWidth', 1.8); hold on;
plot(t, y_pid*100, 'r-', 'LineWidth', 1.6);
plot(t, y_fuzzy*100, '-', 'Color', [0 0 0.5], 'LineWidth', 1.6); 
plot(t, y_psrl*100, 'g-', 'LineWidth', 2.0);
xline(params.t_drop, 'k:', 'LineWidth', 1.5, 'Color', 'k');
grid on; xlabel('Time [s]', 'Color', 'k'); ylabel('Cup Height Y [cm]', 'Color', 'k');
legend('Desired Path', 'Integer PID', 'Adaptive Fuzzy PID', 'PS-RL-FOPID', 'Location', 'southeast', 'TextColor', 'k');
title('End-Effector Safe-Zone Height', 'Color', 'k');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');

%% Benchmark Simulation Function
function [q_out, tip_y, itae, iae] = run_benchmark_sim(p, tr, t_vec, dt, mode, ctrl)
    N = length(t_vec);
    
    q = tr.q_start;         
    dq = zeros(3,1);        
    q_out = zeros(N, 3);
    tip_y = zeros(N, 1);
    
    L_mem = 200;
    e_hist = zeros(L_mem, 3); 
    e_int_classic = zeros(3,1); 
    itae = 0;
    iae = 0;
    
    lambda = 0.90; 
    mu     = 0.70;
    
    int_scale = dt^lambda;
    diff_scale = dt^(-mu);
    
    w_int = zeros(L_mem, 1);
    w_diff = zeros(L_mem, 1);
    w_int(1) = 1; w_diff(1) = 1;
    for j = 2:L_mem
        w_int(j)  = w_int(j-1) * (1 - (-lambda + 1)/(j-1));
        w_diff(j) = w_diff(j-1) * (1 - (mu + 1)/(j-1));
    end
    
    p_nom = p;
    p_nom.m3 = p.m3_base + 0.100; 
    
    for k = 1:N
        tk = t_vec(k);
        [qd, dqd, ~] = quintic_eval(tk, tr.q_start, tr.q_end, tr.tf);
        
        e = qd - q;
        de = dqd - dq;
        e_hist = [e'; e_hist(1:end-1, :)];
        
        iae = iae + sum(abs(e)) * dt;
        itae = itae + tk * sum(abs(e)) * dt;
        
        tau_ff = compute_G(q, p_nom);
        
        if strcmp(mode, 'PID')
            % Added Anti-Windup to make the baseline competent
            e_int_classic = max(min(e_int_classic + e * dt, 1.0), -1.0);
            tau_fb = ctrl.Kp .* e + ctrl.Ki .* e_int_classic + ctrl.Kd .* de;
            
        elseif strcmp(mode, 'FUZZY_PID')
            e_int_classic = max(min(e_int_classic + e * dt, 1.0), -1.0);
            
            gain_scale_p = 1.0 + 0.8 * (1 - exp(-(abs(e).^2) / 0.05));
            gain_scale_i = exp(-(abs(e).^2) / 0.05); 
            gain_scale_d = 1.0 + 0.5 * (1 - exp(-(abs(de).^2) / 0.1));
            
            tau_fb = (ctrl.Kp .* gain_scale_p) .* e + ...
                     (ctrl.Ki .* gain_scale_i) .* e_int_classic + ...
                     (ctrl.Kd .* gain_scale_d) .* de;
                     
        elseif strcmp(mode, 'PS_RL_FOPID') 
            % FIXED MATH: Now normalizes precisely to 1.0 when error is zero
            sig_e = 1.0 ./ (1.0 + exp(-5.0 * (abs(e) - 0.01)));
            sig_0 = 1.0 / (1.0 + exp(0.05)); % Value at e=0 is ~0.4875
            adapt_p = 1.0 + 4.5 * ((sig_e - sig_0) ./ (1.0 - sig_0));
            
            adapt_d = 1.0 + 2.5 * tanh(5.0 * abs(de)); % Already natural at 1.0
            
            frac_int  = (sum(w_int .* e_hist, 1)') * int_scale;
            frac_diff = (sum(w_diff .* e_hist, 1)') * diff_scale;
            
            tau_fb = (ctrl.Kp .* adapt_p) .* e + ...
                     ctrl.Ki .* frac_int + ...
                     (ctrl.Kd .* adapt_d) .* frac_diff;
        end
        
        tau = tau_ff + tau_fb;
        tau = max(min(tau, 35), -35); 
        
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
        tip_y(k) = p.L1*sin(q(1)) + p.L2*sin(q(1)+q(2)) + p.L3*sin(sum(q));
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
% Add this loop to your main script to generate standard deviations for Table II
num_trials = 10;
itae_results = zeros(num_trials, 1);
overshoot_results = zeros(num_trials, 1);

for trial = 1:num_trials
    % Randomize the sip time between 1.5s and 2.2s
    params.t_drop = 1.5 + rand() * 0.7; 

    % Randomize the unmodeled initial liquid mass between 180g and 250g
    params.mp_full = 0.180 + rand() * 0.070; 

    [~, y_psrl, itae_psrl, ~] = run_benchmark_sim(params, traj, t, dt, 'PS_RL_FOPID', base_gains);

    itae_results(trial) = itae_psrl;
    overshoot_results(trial) = (max(y_psrl) - y_des(end))*100;
end

fprintf('PS-RL-FOPID ITAE: %.4f ± %.4f\n', mean(itae_results), std(itae_results));
fprintf('PS-RL-FOPID Overshoot: %.2f ± %.2f cm\n', mean(overshoot_results), std(overshoot_results));

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