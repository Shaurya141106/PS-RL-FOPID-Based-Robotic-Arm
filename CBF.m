%% Step 3: Smooth Control Barrier Function: Adaptive Fuzzy PID vs PS-RL-FOPID
clear; clc; close all;

% 1. Physical Parameters (URDF)
params.m1 = 0.25975;
params.m2 = 0.22196;
params.m3_base = 0.57884;
params.mp_full  = 0.300; 
params.mp_empty = 0.010;
params.t_drop   = 1.5;
params.L1 = 0.2947; params.L2 = 0.2500; params.L3 = 0.0400;
params.lc1 = 0.1419; params.lc2 = 0.1268; params.lc3 = 0.0380;
params.I1 = 2.1385e-3; params.I2 = 1.3141e-3; params.I3 = 3.1123e-4;
params.g  = 9.81;

% 2. Safe Start -> Over-Reaching Fault Trajectory
% Commands the robot to push forward to X = 0.54m (Violating the 0.46m safety limit)
traj.q_start = [deg2rad(55); deg2rad(25); deg2rad(-80)]; 
traj.q_end   = [deg2rad(15); deg2rad(15); deg2rad(-30)]; 
traj.tf      = 1.8;
dt = 0.001;
t = 0:dt:3.0;
N = length(t);

% 3. Virtual Barrier Setup
cbf.x_limit = 0.46; % Absolute forward limit (46 cm)

% 4. Controller Base Gains
base_gains.Kp = [50.0; 35.0; 12.0];
base_gains.Ki = [30.0; 20.0;  6.0];
base_gains.Kd = [10.0;  7.0;  2.0];

% 5. Run Comparisons
fprintf('Simulating Unconstrained Fault Scenario (Red)...\n');
[q_raw, tip_raw, tau_raw] = run_smooth_cbf_sim(params, traj, t, dt, base_gains, cbf, 'PS_RL_FOPID', false);

fprintf('Simulating Adaptive Fuzzy PID with Smooth CBF (Dark Blue)...\n');
[q_fuzzy, tip_fuzzy, tau_fuzzy] = run_smooth_cbf_sim(params, traj, t, dt, base_gains, cbf, 'FUZZY_PID', true);

fprintf('Simulating PS-RL-FOPID with Smooth CBF (Green)...\n');
[q_psrl, tip_psrl, tau_psrl] = run_smooth_cbf_sim(params, traj, t, dt, base_gains, cbf, 'PS_RL_FOPID', true);

% 6. Plotting
% Changed Figure background to white [1 1 1]
figure('Color', [1 1 1], 'Name', 'Smooth CBF Safety Barrier Integration', 'Position', [100 100 950 750]);

% Displacement X (Safety Zone Breach Check)
subplot(2,1,1);
plot(t, tip_raw(:,1)*100, 'r--', 'LineWidth', 1.8); hold on;
% Changed cyan to dark blue [0 0 0.5]
plot(t, tip_fuzzy(:,1)*100, 'Color', [0 0 0.5], 'LineStyle', '-', 'LineWidth', 1.5);
plot(t, tip_psrl(:,1)*100, 'g-', 'LineWidth', 2.2);
yline(cbf.x_limit*100, 'm-', 'LineWidth', 2.0, 'Label', 'Virtual Safety Barrier (46.0 cm)', 'Color', 'm');
xline(params.t_drop, 'y:', 'LineWidth', 1.5, 'Label', 'Payload Sip', 'Color', 'y');
grid on; 
% Changed text color to black ('k')
ylabel('X Position [cm]', 'Color', 'k');
legend('Unconstrained Fault (Dangerous Breach)', 'Fuzzy PID + CBF', 'PS-RL-FOPID + CBF', 'Safety Barrier Limit', ...
       'Location', 'northwest', 'TextColor', 'k');
title('Smooth Control Barrier Function: Forward Over-Reach Interception', 'Color', 'k');
% Changed axes background to white and tick colors to black
set(gca, 'Color', [1 1 1], 'XColor', 'k', 'YColor', 'k');

% Actuator Torque (Chatter and Braking Check)
subplot(2,1,2);
plot(t, tau_raw(:,1), 'r--', 'LineWidth', 1.5); hold on;
% Changed cyan to dark blue [0 0 0.5]
plot(t, tau_fuzzy(:,1), 'Color', [0 0 0.5], 'LineStyle', '-', 'LineWidth', 1.5);
plot(t, tau_psrl(:,1), 'g-', 'LineWidth', 2.0);
xline(params.t_drop, 'y:', 'LineWidth', 1.5, 'Color', 'y');
grid on; 
% Changed text color to black ('k')
xlabel('Time [s]', 'Color', 'k'); ylabel('\tau_1 (Shoulder) [Nm]', 'Color', 'k');
legend('Nominal Fault Torque', 'Fuzzy PID + CBF Torque', 'PS-RL-FOPID + CBF Torque', 'Location', 'northeast', 'TextColor', 'k');
title('Actuator Torque Response: Continuous Chatter-Free Intervention', 'Color', 'k');
% Changed axes background to white and tick colors to black
set(gca, 'Color', [1 1 1], 'XColor', 'k', 'YColor', 'k');

%% Smooth CBF Simulation Engine
function [q_out, tip_pos, tau_out] = run_smooth_cbf_sim(p, tr, t_vec, dt, ctrl, cbf, mode, enable_cbf)
    N = length(t_vec);
    q = tr.q_start; dq = zeros(3,1);
    q_out = zeros(N, 3);
    tip_pos = zeros(N, 2);
    tau_out = zeros(N, 3);
    
    L_mem = 200;
    e_hist = zeros(L_mem, 3);
    e_int_classic = zeros(3,1);
    
    lambda = 0.90; mu = 0.70;
    int_scale = dt^lambda; diff_scale = dt^(-mu);
    
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
        [qd, dqd, ~] = quintic_eval(tk, tr.q_start, tr.q_end, tr.tf);
        
        if tk < p.t_drop
            p.m3 = p.m3_base + p.mp_full;
        else
            p.m3 = p.m3_base + p.mp_empty;
        end
        
        e = qd - q;
        de = dqd - dq;
        e_hist = [e'; e_hist(1:end-1, :)];
        
        tau_ff = compute_G(q, p_nom);
        
        % Generate Nominal Tracking Torque
        if strcmp(mode, 'FUZZY_PID')
            e_int_classic = e_int_classic + e * dt;
            gain_scale_p = 1.0 + 2.5 * (1 - exp(-(abs(e).^2) / 0.02));
            gain_scale_i = 1.0 + 1.5 * exp(-(abs(e).^2) / 0.05);
            gain_scale_d = 1.0 + 1.2 * (1 - exp(-(abs(de).^2) / 0.1));
            
            tau_fb = (ctrl.Kp .* gain_scale_p) .* e + ...
                     (ctrl.Ki .* gain_scale_i) .* e_int_classic + ...
                     (ctrl.Kd .* gain_scale_d) .* de;
                     
        elseif strcmp(mode, 'PS_RL_FOPID')
            adapt_p = 1.0 + 4.5 * (1.0 ./ (1.0 + exp(-5.0 * (abs(e) - 0.01))));
            adapt_d = 1.0 + 2.5 * tanh(5.0 * abs(de));
            
            frac_int  = (sum(w_int .* e_hist, 1)') * int_scale;
            frac_diff = (sum(w_diff .* e_hist, 1)') * diff_scale;
            
            tau_fb = (ctrl.Kp .* adapt_p) .* e + ctrl.Ki .* frac_int + (ctrl.Kd .* adapt_d) .* frac_diff;
        end
        
        tau_nom = tau_ff + tau_fb;
        
        % Smooth CBF Safety Filter Intervention
        if enable_cbf
            q1 = q(1); q2 = q(2); q3 = q(3);
            x_tip = p.L1*cos(q1) + p.L2*cos(q1+q2) + p.L3*cos(q1+q2+q3);
            
            Jx = [-p.L1*sin(q1) - p.L2*sin(q1+q2) - p.L3*sin(q1+q2+q3), ...
                  -p.L2*sin(q1+q2) - p.L3*sin(q1+q2+q3), ...
                  -p.L3*sin(q1+q2+q3)];
              
            h = cbf.x_limit - x_tip;
            x_dot = Jx * dq;
            
            % Smooth barrier correction using a continuous boundary penalty
            margin = 0.04; % 4 cm safety activation zone
            if h < margin
                % Soft boundary scaling factor (0 to 1) - Equivalent to QP slack relaxation
                alpha_barrier = 0.5 * (1 + cos(pi * max(h, 0) / margin)); 
                
                % Damped braking torque projected into joint space
                f_brake = -80.0 * (margin - h) - 15.0 * max(x_dot, 0);
                tau_cbf_adjust = Jx' * f_brake;
                
                % Seamless torque blending
                tau = (1 - alpha_barrier) * tau_nom + alpha_barrier * (tau_ff + tau_cbf_adjust);
            else
                tau = tau_nom; % Free operation
            end
        else
            tau = tau_nom; % Safety constraint disabled
        end
        
        tau = max(min(tau, 35), -35);
        tau_out(k, :) = tau';
        
        M = compute_M(q, p);
        G = compute_G(q, p);
        ddq = M \ (tau - 0.05*dq - G);
        
        dq = dq + ddq * dt;
        q  = q + dq * dt;
        
        q_out(k, :) = q';
        tip_pos(k, 1) = p.L1*cos(q(1)) + p.L2*cos(q(1)+q(2)) + p.L3*cos(sum(q));
        tip_pos(k, 2) = p.L1*sin(q(1)) + p.L2*sin(q(1)+q(2)) + p.L3*sin(sum(q));
    end
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