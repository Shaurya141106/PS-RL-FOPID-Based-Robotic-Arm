%% Step 1: Dynamic Anti-Slosh Trajectory Tracking: Adaptive Fuzzy PID vs PS-RL-FOPID
clear; clc; close all;

% 1. Physical Parameters (URDF)
params.m1 = 0.25975;
params.m2 = 0.22196;
params.m3_base = 0.57884;
params.mp_full  = 0.250; % 250g liquid payload
params.mp_empty = 0.020; % 20g after drinking/spillage
params.t_drop   = 1.6;
params.L1 = 0.2947; params.L2 = 0.2500; params.L3 = 0.0400;
params.lc1 = 0.1419; params.lc2 = 0.1268; params.lc3 = 0.0380;
params.I1 = 2.1385e-3; params.I2 = 1.3141e-3; params.I3 = 3.1123e-4;
params.g  = 9.81;

% 2. Fast Feeding Path Specifications (tf = 2.0 s)
traj.q_start = [deg2rad(10); deg2rad(30); deg2rad(-40)];
traj.q_end   = [deg2rad(60); deg2rad(20); deg2rad(-80)];
traj.tf      = 2.0;
dt = 0.001;
t = 0:dt:3.5;
N = length(t);

% 3. Generate Anti-Slosh Compensated Trajectory for Wrist (q3)
[qd_nominal, dqd_nominal, ddqd_nominal] = generate_nominal_traj(t, traj);
[qd_slosh, dqd_slosh, ddqd_slosh, theta_tilt] = compute_antislosh_traj(t, traj, params);

% 4. Controller Setup (Equal Base Gains)
ctrl.Kp = [65.0; 45.0; 18.0];
ctrl.Ki = [35.0; 22.0;  9.0];
ctrl.Kd = [12.0;  8.5;  3.0];
lambda = 0.90; 
mu     = 0.70;

% 5. Run Two Simulations (Both attempting the Anti-Slosh path)
fprintf('Running Adaptive Fuzzy PID on Anti-Slosh Path...\n');
[q_fuzzy, a_eff_fuzzy, slosh_angle_fuzzy] = run_antislosh_sim(params, t, dt, qd_slosh, dqd_slosh, 'FUZZY_PID', ctrl, lambda, mu);

fprintf('Running PS-RL-FOPID on Anti-Slosh Path...\n');
[q_psrl, a_eff_psrl, slosh_angle_psrl] = run_antislosh_sim(params, t, dt, qd_slosh, dqd_slosh, 'PS_RL_FOPID', ctrl, lambda, mu);

% 6. Plot Slosh Angle and Liquid Behavior Comparison
figure('Color', 'w', 'Name', 'Anti-Slosh Tracking Performance', 'Position', [100 100 950 750]);

% Subplot 1: Joint 3 (Wrist) Trajectory Modification
subplot(3,1,1);
plot(t, rad2deg(qd_slosh(:,3)), 'k--', 'LineWidth', 2.0); hold on;
plot(t, rad2deg(q_fuzzy(:,3)), '-', 'Color', [0 0 0.5], 'LineWidth', 1.5); 
plot(t, rad2deg(q_psrl(:,3)), 'g-', 'LineWidth', 2.0);
xline(params.t_drop, 'k:', 'LineWidth', 1.5, 'Label', 'Sip Action / Drop', 'Color', 'k');
grid on; ylabel('q_3 (Wrist) [deg]', 'Color', 'k');
legend('Anti-Slosh Reference', 'Adaptive Fuzzy PID', 'PS-RL-FOPID', 'Location', 'southeast', 'TextColor', 'k');
title('Wrist Trajectory Adaptation Tracking for Liquid Stabilization', 'Color', 'k');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');

% Subplot 2: Instantaneous Apparent Tilt / Slosh Deflection Angle
subplot(3,1,2);
plot(t, rad2deg(slosh_angle_fuzzy), '-', 'Color', [0 0 0.5], 'LineWidth', 1.5); hold on; 
plot(t, rad2deg(slosh_angle_psrl), 'g-', 'LineWidth', 2.0);
yline(10, 'r--', 'LineWidth', 1.2, 'Label', 'Spill Threshold (+10 deg)', 'Color', 'r', 'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'bottom');
yline(-10, 'r--', 'LineWidth', 1.2, 'Label', 'Spill Threshold (-10 deg)', 'Color', 'r', 'LabelHorizontalAlignment', 'right', 'LabelVerticalAlignment', 'top');
xline(params.t_drop, 'k:', 'LineWidth', 1.5, 'Color', 'k');
grid on; ylabel('Slosh Angle [deg]', 'Color', 'k');
legend('Adaptive Fuzzy PID', 'PS-RL-FOPID', 'Location', 'northeast', 'TextColor', 'k');
title('Relative Liquid Free-Surface Slosh Deflection (\theta_{slosh})', 'Color', 'k');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');

% Subplot 3: Cup Height Safe-Zone Tracking
y_des = params.L1*sin(qd_nominal(:,1)) + params.L2*sin(qd_nominal(:,1)+qd_nominal(:,2));
y_fuzzy = params.L1*sin(q_fuzzy(:,1)) + params.L2*sin(q_fuzzy(:,1)+q_fuzzy(:,2));
y_psrl = params.L1*sin(q_psrl(:,1)) + params.L2*sin(q_psrl(:,1)+q_psrl(:,2));

subplot(3,1,3);
plot(t, y_des*100, 'k--', 'LineWidth', 1.8); hold on;
plot(t, y_fuzzy*100, '-', 'Color', [0 0 0.5], 'LineWidth', 1.5); 
plot(t, y_psrl*100, 'g-', 'LineWidth', 2.0);
xline(params.t_drop, 'k:', 'LineWidth', 1.5, 'Color', 'k');
grid on; xlabel('Time [s]', 'Color', 'k'); ylabel('Height Y [cm]', 'Color', 'k');
legend('Target Trajectory', 'Adaptive Fuzzy PID', 'PS-RL-FOPID', 'Location', 'southeast', 'TextColor', 'k');
title('Vertical End-Effector Profile', 'Color', 'k');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');

% Print summary metrics
fprintf('\n================== SLOSH REDUCTION RESULTS ==================\n');
fprintf('Peak Slosh Angle (Adaptive Fuzzy) : %.2f degrees\n', rad2deg(max(abs(slosh_angle_fuzzy))));
fprintf('Peak Slosh Angle (PS-RL-FOPID)    : %.2f degrees\n', rad2deg(max(abs(slosh_angle_psrl))));
fprintf('=============================================================\n\n');

%% --- Trajectory & Dynamic Calculation Functions ---
function [qd, dqd, ddqd] = generate_nominal_traj(t_vec, tr)
    N = length(t_vec);
    qd = zeros(N, 3); dqd = zeros(N, 3); ddqd = zeros(N, 3);
    for k = 1:N
        [q, dq, ddq] = quintic_eval(t_vec(k), tr.q_start, tr.q_end, tr.tf);
        qd(k,:) = q'; dqd(k,:) = dq'; ddqd(k,:) = ddq';
    end
end

function [qd_out, dqd_out, ddqd_out, theta_cup] = compute_antislosh_traj(t_vec, tr, p)
    N = length(t_vec);
    qd_out = zeros(N, 3); dqd_out = zeros(N, 3); ddqd_out = zeros(N, 3);
    theta_cup = zeros(N, 1);
    
    for k = 1:N
        [q, dq, ddq] = quintic_eval(t_vec(k), tr.q_start, tr.q_end, tr.tf);
        
        q1 = q(1); q2 = q(2);
        dq1 = dq(1); dq2 = dq(2);
        ddq1 = ddq(1); ddq2 = ddq(2);
        
        ax_tip = -p.L1*sin(q1)*ddq1 - p.L1*cos(q1)*dq1^2 ...
                 -p.L2*sin(q1+q2)*(ddq1+ddq2) - p.L2*cos(q1+q2)*(dq1+dq2)^2;
             
        ay_tip =  p.L1*cos(q1)*ddq1 - p.L1*sin(q1)*dq1^2 ...
                 +p.L2*cos(q1+q2)*(ddq1+ddq2) - p.L2*sin(q1+q2)*(dq1+dq2)^2;
        
        theta_optimal = atan2(-ax_tip, p.g + ay_tip);
        theta_cup(k) = theta_optimal;
        
        q3_antislosh = (theta_optimal - pi/2) - (q1 + q2);
        qd_out(k,:) = [q1, q2, q3_antislosh];
    end
    
    dt = t_vec(2) - t_vec(1);
    dqd_out = gradient(qd_out, dt);
    ddqd_out = gradient(dqd_out, dt);
end

function [q_log, a_eff_log, slosh_log] = run_antislosh_sim(p, t_vec, dt, qd_mat, dqd_mat, mode, ctrl, lambda, mu)
    N = length(t_vec);
    q = qd_mat(1,:)';
    dq = zeros(3,1);
    
    q_log = zeros(N, 3);
    a_eff_log = zeros(N, 2);
    slosh_log = zeros(N, 1);
    
    L_mem = 200;
    e_hist = zeros(L_mem, 3);
    e_int_classic = zeros(3,1);
    
    int_scale = dt^lambda;
    diff_scale = dt^(-mu);
    
    w_int = zeros(L_mem, 1); w_diff = zeros(L_mem, 1);
    w_int(1) = 1; w_diff(1) = 1;
    for j = 2:L_mem
        w_int(j)  = w_int(j-1) * (1 - (-lambda + 1)/(j-1));
        w_diff(j) = w_diff(j-1) * (1 - (mu + 1)/(j-1));
    end
    
    p_nom = p;
    p_nom.m3 = p.m3_base + 0.100;
    
    for k = 1:N
        tk = t_vec(k);
        qd = qd_mat(k,:)';
        dqd = dqd_mat(k,:)';
        
        e = qd - q;
        de = dqd - dq;
        e_hist = [e'; e_hist(1:end-1, :)];
        
        tau_ff = compute_G(q, p_nom);
        
        if strcmp(mode, 'FUZZY_PID')
            % FIXED: Anti-windup and correct fuzzy rules restored
            e_int_classic = max(min(e_int_classic + e * dt, 1.0), -1.0);
            
            gain_scale_p = 1.0 + 0.8 * (1 - exp(-(abs(e).^2) / 0.05));
            gain_scale_i = exp(-(abs(e).^2) / 0.05); 
            gain_scale_d = 1.0 + 0.5 * (1 - exp(-(abs(de).^2) / 0.1));
            
            tau_fb = (ctrl.Kp .* gain_scale_p) .* e + ...
                     (ctrl.Ki .* gain_scale_i) .* e_int_classic + ...
                     (ctrl.Kd .* gain_scale_d) .* de;
                     
        elseif strcmp(mode, 'PS_RL_FOPID')
            % FIXED: Mathematical normalization restored
            sig_e = 1.0 ./ (1.0 + exp(-5.0 * (abs(e) - 0.01)));
            sig_0 = 1.0 / (1.0 + exp(0.05)); 
            adapt_p = 1.0 + 4.5 * ((sig_e - sig_0) ./ (1.0 - sig_0));
            
            adapt_d = 1.0 + 2.5 * tanh(5.0 * abs(de));
            
            frac_int  = (sum(w_int .* e_hist, 1)') * int_scale;
            frac_diff = (sum(w_diff .* e_hist, 1)') * diff_scale;
            
            tau_fb = (ctrl.Kp .* adapt_p) .* e + ctrl.Ki .* frac_int + (ctrl.Kd .* adapt_d) .* frac_diff;
        end
        
        tau = max(min(tau_ff + tau_fb, 35), -35);
        
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
        q_log(k, :) = q';
        
        % Measure physical tip acceleration
        q1 = q(1); q2 = q(2); q3 = q(3);
        dq1 = dq(1); dq2 = dq(2);
        ddq1 = ddq(1); ddq2 = ddq(2);
        
        ax_tip = -p.L1*sin(q1)*ddq1 - p.L1*cos(q1)*dq1^2 ...
                 -p.L2*sin(q1+q2)*(ddq1+ddq2) - p.L2*cos(q1+q2)*(dq1+dq2)^2;
        ay_tip =  p.L1*cos(q1)*ddq1 - p.L1*sin(q1)*dq1^2 ...
                 +p.L2*cos(q1+q2)*(ddq1+ddq2) - p.L2*sin(q1+q2)*(dq1+dq2)^2;
             
        a_eff = [-ax_tip; -(p.g + ay_tip)];
        a_eff_log(k, :) = a_eff';
        
        % FIXED: Mathematical wrapping of the circular phase difference 
        cup_world_angle = q1 + q2 + q3 + pi/2;
        app_grav_angle = atan2(a_eff(1), -a_eff(2));
        
        raw_slosh = app_grav_angle - cup_world_angle;
        slosh_log(k) = atan2(sin(raw_slosh), cos(raw_slosh)); 
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