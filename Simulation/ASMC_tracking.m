%  ASMC - CubeSat Simulation

clear; clc; close all;

% Simulation Parameters
dt      = 0.01;
t_final = 160;
time    = 0:dt:t_final;
N       = length(time);

%  Fault injection
fault_time  = [20, 80];    % time of wheel failure
fault_wheel = [3, 4];     % which wheel fails
fault_level = [0.0, 0.1];   % effectiveness after fault  (0 = dead, 1 = healthy)

% Passive fault detection threshold (display only)
s_fault_thresh = 0.01;   % |s| above this flags a likely fault

% CubeSat Parameters
J             = diag([0.002167, 0.002167, 0.002167]);
Js = J * 1;
A             = [-0.5,   -0.5,   0.5,  0.5;
                  0.5,   -0.5,  -0.5,  0.5;
                  0.707,  0.707, 0.707, 0.707];
A_pinv        = pinv(A);
tau_wheel_max = 0.005;   % Max torque per wheel Nm

% Controller gains 
lambda = 1.5;    % Sliding surface slope
Kd     = 0.08;   % Velocity damping
Kp     = 0.04;   % Attitude error proportional gain
phi    = 0.05;    % Boundary layer thickness

% Adaptive gain parameters
K_adapt_init = 0.01;   % Initial adaptive gain
Gamma        = 2.0;    % Adaptation rate
sigma        = 0.015;  % Leakage coefficient
K_min        = 0.01;   % Minimum switching gain (nominal stability floor)
K_max        = 2.0;    % Maximum switching gain (actuator saturation ceiling)

%  Initial Conditions
ax0    = [1;1;1] / norm([1;1;1]);
ang0   = 30 * pi/180;
q0     = [cos(ang0/2); sin(ang0/2)*ax0];
omega0 = [0.02; -0.01; 0.01];

% (Desired orientation)
ang_d   = 60 * pi/180;
q_d     = [cos(ang_d/2); sin(ang_d/2); 0; 0];
omega_d = [0; 0; 0];

% Define waypoints as [angle_deg, axis]
waypoints = [
    0,  1, 0, 0;   % 0° about X
    130,  1, 1, 0;   % 130° about XY  
    90,  0, 0, 1;   % 90° about Z
   -15,  1, 1, 0;   % -15° about XY
];
waypoint_times = [0, 18, 35, 55];  

%  Data storage (needed for visualisation)
q_hist        = zeros(4, N);
omega_hist    = zeros(3, N);
tau_hist      = zeros(3, N);
wheel_hist    = zeros(4, N);
s_hist        = zeros(3, N);
err_hist      = zeros(1, N);
Kadapt_hist   = zeros(1, N);   % adaptive gain history
effect_hist   = ones(4,  N);   % wheel effectiveness history
fault_flag    = zeros(1, N);   % passive fault indicator (display only)

%  update loop
q            = q0;
omega        = omega0;
K_adapt      = K_adapt_init;   % Scalar adaptive switching gain
wheel_effect = ones(4, 1);

% Simple sliding-surface spike detector for passive fault display
s_window     = zeros(1, 10);   % short window to detect sustained |s| spike

fprintf('CubeSat ASMC (Fixed Adaptive) Simulation\n');
fprintf('==========================================\n');
fprintf('Initial attitude error : %.2f deg\n', quat_angle_error(q, q_d));
fprintf('Fault                  : Wheel %d fails at t = %.1f s (effectiveness %.0f%%)\n', ...
    fault_wheel, fault_time, fault_level*100);
fprintf('Running...\n\n');

% --------------------------------------------------------------------------------------------------------------
% Simulation loop
for i = 1:N
    t = time(i);

    % Fault injection
    for f = 1:length(fault_time)
        if t >= fault_time(f)
            wheel_effect(fault_wheel(f)) = fault_level(f);
        end
    end
    
    % Update waypoint
    ang_d = (90 * sin(0.1 * t) + 45) * pi/180;   % oscillate 15°–75°
    ax_d  = [1; 0; 0];                             % about X axis
    q_d   = [cos(ang_d/2); sin(ang_d/2)*ax_d];
    % Also update omega_d to the reference angular velocity
    % (otherwise the sliding surface has a phantom error term)
    ang_d_dot = 30 * 0.1 * cos(0.1 * t) * pi/180;
    omega_d   = ang_d_dot * ax_d;

    % Quaternion and angular velocity error
    q_e_full = quat_mult(quat_inv(q_d), q);
    if q_e_full(1) < 0
        q_e_full = -q_e_full;
    end
    e_q     = q_e_full(2:4);
    e_omega = omega - omega_d;



    % Sliding surface
    s = e_omega + lambda * e_q;
    s_norm = norm(s);

    % Gyroscopic compensation
    tau_gyro = -skew(omega) * (J * omega);

    % PD feedback
    tau_pd = Js * (-Kp * e_q - Kd * e_omega);

    % Adaptive switching term
    tau_sw = -Js * K_adapt * sat_vec(s, phi);

    % Total torque
    tau = tau_pd + tau_sw + tau_gyro;

    % Torque allocation + analysis
    tau_wheels_cmd    = A_pinv * tau;
    tau_wheels_cmd    = max(min(tau_wheels_cmd, tau_wheel_max), -tau_wheel_max);
    tau_wheels_actual = wheel_effect .* tau_wheels_cmd;
    tau_actual        = A * tau_wheels_actual;

    % Adaptive term
    K_adapt_dot = Gamma * s_norm - sigma * (K_adapt - K_min);
    K_adapt     = K_adapt + K_adapt_dot * dt;
    K_adapt     = max(K_min, min(K_max, K_adapt));

    % Fault indicator for display
    s_window = [s_window(2:end), s_norm];
    if mean(s_window) > s_fault_thresh && t > 5   % ignore startup transient
        fault_flag(i) = 1;
    end

    % Log data to workspace
    q_hist(:,    i) = q;
    omega_hist(:,i) = omega;
    tau_hist(:,  i) = tau_actual;
    wheel_hist(:,i) = tau_wheels_actual;
    s_hist(:,    i) = s;
    err_hist(i)     = quat_angle_error(q, q_d);
    Kadapt_hist(i)  = K_adapt;
    effect_hist(:,i) = wheel_effect;

    % Dynamics calculations
    omega_dot = Js \ (tau_actual - skew(omega) * (Js * omega));
    q_dot     = 0.5 * Xi(q) * omega;

    omega = omega + omega_dot * dt;
    q     = q     + q_dot     * dt;
    q     = q / norm(q);

    % test
    if mod(i, round(N/10)) == 0
        fprintf('t = %5.1f s | err = %7.3f deg | |omega| = %.4f rad/s | K_adapt = %.4f\n', ...
            t, err_hist(i), norm(omega), K_adapt);
    end
end

fprintf('\n');

%-------------------------------------------------------------------------------------------------------------

% performance report
thresh      = 5.0;   % degrees
settled_idx = find(err_hist < thresh, 1, 'first');

pre_idx  = time <  fault_time(1); 
post_idx = time >= fault_time(1);

fprintf('Performance Metrics: \n');
if ~isempty(settled_idx)
    if time(settled_idx) < fault_time
        fprintf('Settling time (<%g deg): %.2f s  [before fault]\n', thresh, time(settled_idx));
    else
        fprintf('Settling time (<%g deg): %.2f s  [after fault injection]\n', thresh, time(settled_idx));
    end
else
    fprintf('Settling time: Not achieved within %.0f s\n', t_final);
end
fprintf('Final attitude error    : %.4f deg\n', err_hist(end));
fprintf('Final angular velocity  : [%.4f, %.4f, %.4f] rad/s\n', omega);
fprintf('Max torque magnitude    : %.5f N·m\n',  max(vecnorm(tau_hist)));
fprintf('Total control effort    : %.4f N·m·s\n', sum(vecnorm(tau_hist)) * dt);
fprintf('K_adapt final           : %.4f  (started at %.4f)\n', K_adapt, K_adapt_init);

% Passive fault detection report
fault_detected_idx = find(fault_flag, 1, 'first');
if ~isempty(fault_detected_idx)
    fprintf('Passive fault indicator : triggered at t = %.2f s\n', time(fault_detected_idx));
else
    fprintf('Passive fault indicator : no fault detected\n');
end

if fault_time(1) < t_final
    post_err  = err_hist(time >= fault_time(1));
    post_time = time(time >= fault_time(1));
    recover_idx = find(post_err < thresh, 1, 'first');
    if ~isempty(recover_idx)
        fprintf('Recovery time after first fault: %.2f s\n', post_time(recover_idx) - fault_time(1));
    end
end

%-------------------------------------------------------------------------------------------
% Graphs
C = [0.8500 0.3250 0.0980;
     0.4660 0.6740 0.1880;
     0.0000 0.4470 0.7410;
     0.4940 0.1840 0.5560];
fault_color = [0.9 0.3 0.3];
flag_color  = [1.0 0.6 0.0];   % orange for passive fault indicator
main_colour = [0.04 0.77 0.596];

show_fault  = fault_time < t_final;
fault_label = sprintf('RW%d fails (injected)', fault_wheel);
flag_label  = 'Fault detected (passive)';

%% Graphs / plots
% Create figure, white background
fig = figure('Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8], 'Color', 'w');

% 1. Adaptive Gain
subplot(2,2,1);
yyaxis left
plot(time, err_hist, 'Color', [0.04 0.77 0.596], 'LineWidth', 2);
ylabel('Attitude Error [deg]', 'FontWeight', 'bold');
grid on; hold on;
ax = gca;
ax.YAxis(1).Color = 'k'; % Labels are black

yyaxis right
plot(time, Kadapt_hist, 'Color', [0.85 0.32 0.1], 'LineWidth', 2.5);
xlabel('Time [s]', 'FontWeight', 'bold');
ylabel('Adaptive Gain K(t)', 'FontWeight', 'bold');
ax.YAxis(2).Color = 'k'; 
ax.XAxis(1).Color = 'k';

xline(fault_time(1), '--', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.5);
text(fault_time(1), max(err_hist)*0.3, 'RW3 Failure', 'Color', 'k', 'FontWeight', 'bold');
xline(fault_time(2), '--', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.5);
text(fault_time(2), max(err_hist)*0.9, 'RW4 Failure', 'Color', 'k', 'FontWeight', 'bold');
title('Figure 2A. Adaptative Gain Response', 'Color', 'k');
legend('Attitude Error', 'Adaptive Gain', 'Location', 'best');

% 2. Phase plane (ERROR VS RATE)
subplot(2,2,2);
e_q_x = q_hist(2,:) * 180/pi; 
e_w_x = omega_hist(1,:);      
plot(e_q_x, e_w_x, 'LineWidth', 1.5, 'Color', [0.2 0.2 0.2]);
hold on; grid on;

% Sliding Surface Line (s=0)
e_limit = max(abs(e_q_x));
e_axis = linspace(-e_limit, e_limit, 100);
s_line = -lambda * (e_axis * pi/180); 
plot(e_axis, s_line, 'r--', 'LineWidth', 2);
plot(0,0,'ko','MarkerFaceColor','k'); 

ax = gca;
ax.XColor = 'k';
ax.YColor = 'k';
xlabel('Attitude Error [deg]', 'FontWeight', 'bold');
ylabel('Angular Rate [rad/s]', 'FontWeight', 'bold');
title('Figure 2B. Phase Plane Trajectory', 'Color', 'k');
legend('System Path', 'Sliding Surface (s=0)', 'Location', 'best');

% 3. Sliding sruface stability
subplot(2,2,3:4); 
semilogy(time, vecnorm(s_hist), 'LineWidth', 2, 'Color', [0 0.44 0.74]);
hold on; grid on;

% Sliding Surface Boundary
yline(s_fault_thresh, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5);
text(time(round(end/4)), s_fault_thresh*1.4, 'Sliding Surface Boundary', 'FontWeight', 'bold');

xline(fault_time, '--k', 'LineWidth', 1.5);
ax = gca;
ax.XColor = 'k';
ax.YColor = 'k';
ylabel('||s|| (Log Scale)', 'FontWeight', 'bold');
xlabel('Time [s]', 'FontWeight', 'bold');
title('Figure 2C. Sliding Surface Stability (Convergence Proof)', 'Color', 'k');

%{

figure('Position', [50 50 1500 900], 'Color', 'w');
sgtitle('CubeSat ASMC (Fixed Adaptive + Passive FTC) — Fault Tolerance Simulation', ...
    'FontSize', 13, 'FontWeight', 'bold');

% Helper: add fault/detection lines to current axes
add_fault_lines = @(ax) deal( ...
    show_fault && xline(ax, fault_time, '--', 'Color', fault_color, ...
        'LineWidth', 1.5, 'Label', fault_label, 'LabelVerticalAlignment','bottom'), ...
    ~isempty(fault_detected_idx) && xline(ax, time(fault_detected_idx), '--', ...
        'Color', flag_color, 'LineWidth', 1.2, 'Label', flag_label, ...
        'LabelVerticalAlignment','top') );

% 1. Attitude error
ax1 = subplot(3,3,1);
plot(time, err_hist, 'Color', main_colour, 'LineWidth', 2);
hold on; grid on; box on;
yline(thresh, 'r--', sprintf('%g° threshold', thresh), 'LineWidth', 1.2);
if ~isempty(settled_idx)
    xline(time(settled_idx), 'b--', 'Settled', 'LineWidth', 1.2);
end
if show_fault; xline(ax1, fault_time, '--', 'Color', fault_color, 'LineWidth', 1.5, 'Label', fault_label, 'LabelVerticalAlignment','bottom'); end
if ~isempty(fault_detected_idx); xline(ax1, time(fault_detected_idx), '--', 'Color', flag_color, 'LineWidth', 1.2, 'Label', flag_label, 'LabelVerticalAlignment','top'); end
ylabel('Error [deg]');
title('Attitude Error');
set(gca, 'FontSize', 10);

% 2. Attitude error (log)
ax2 = subplot(3,3,2);
semilogy(time, max(err_hist, 1e-6), 'Color', main_colour, 'LineWidth', 2);
hold on; grid on; box on;
yline(thresh, 'r--', 'LineWidth', 1.2);
if show_fault; xline(ax2, fault_time, '--', 'Color', fault_color, 'LineWidth', 1.5, 'Label', fault_label, 'LabelVerticalAlignment','bottom'); end
if ~isempty(fault_detected_idx); xline(ax2, time(fault_detected_idx), '--', 'Color', flag_color, 'LineWidth', 1.2, 'Label', flag_label, 'LabelVerticalAlignment','top'); end
ylabel('Error [deg] (log)');
title('Convergence (Log Scale)');
set(gca, 'FontSize', 10);

% 3. Angular velocity
ax3 = subplot(3,3,3);
hold on; grid on; box on;
for k = 1:3
    plot(time, omega_hist(k,:), 'Color', C(k,:), 'LineWidth', 1.5);
end
yline(0, 'k:', 'LineWidth', 0.8);
if show_fault; xline(ax3, fault_time, '--', 'Color', fault_color, 'LineWidth', 1.5); end
legend('\omega_x','\omega_y','\omega_z', 'Location','best');
ylabel('\omega [rad/s]');
title('Angular Velocity');
set(gca, 'FontSize', 10);

% 4. Body torque
ax4 = subplot(3,3,4);
hold on; grid on; box on;
for k = 1:3
    plot(time, tau_hist(k,:)*1e3, 'Color', C(k,:), 'LineWidth', 1.5);
end
if show_fault; xline(ax4, fault_time, '--', 'Color', fault_color, 'LineWidth', 1.5, 'Label', fault_label, 'LabelVerticalAlignment','bottom'); end
legend('\tau_x','\tau_y','\tau_z', 'Location','best');
ylabel('\tau [mN·m]');
title('Body Frame Torque');
set(gca, 'FontSize', 10);

% 5. Wheel torques
ax5 = subplot(3,3,5);
hold on; grid on; box on;
ls = {'-','-','-','-'};
for k = 1:4
    lbl = sprintf('RW%d', k);
    if k == fault_wheel; lbl = sprintf('RW%d (FAILED)', k); ls{k} = '--'; end
    plot(time, wheel_hist(k,:)*1e3, 'Color', C(k,:), ...
        'LineStyle', ls{k}, 'LineWidth', 1.5, 'DisplayName', lbl);
end
yline( tau_wheel_max*1e3, 'k--', 'LineWidth', 1.0, 'HandleVisibility','off');
yline(-tau_wheel_max*1e3, 'k--', 'LineWidth', 1.0, 'HandleVisibility','off');
if show_fault; xline(ax5, fault_time, '--', 'Color', fault_color, 'LineWidth', 1.5, 'Label', fault_label, 'LabelVerticalAlignment','bottom'); end
legend('Location','best');
ylabel('\tau_{wheel} [mN·m]');
title('Reaction Wheel Torques');
set(gca, 'FontSize', 10);

% 6. Adaptive gain K_adapt
ax6 = subplot(3,3,6);
plot(time, Kadapt_hist, 'Color', main_colour, 'LineWidth', 2);
hold on; grid on; box on;
yline(K_min, 'b--', 'K_{min}', 'LineWidth', 1.0);
yline(K_max, 'r--', 'K_{max}', 'LineWidth', 1.0);
if show_fault; xline(ax6, fault_time, '--', 'Color', fault_color, 'LineWidth', 1.5, 'Label', fault_label, 'LabelVerticalAlignment','bottom'); end
if ~isempty(fault_detected_idx); xline(ax6, time(fault_detected_idx), '--', 'Color', flag_color, 'LineWidth', 1.2, 'Label', flag_label, 'LabelVerticalAlignment','top'); end
ylabel('K_{adapt}');
title('Adaptive Switching Gain');
set(gca, 'FontSize', 10);

% 7. Sliding surface
ax7 = subplot(3,3,7);
hold on; grid on; box on;
for k = 1:3
    plot(time, s_hist(k,:), 'Color', C(k,:), 'LineWidth', 1.5);
end
yline(0,'k:','LineWidth',0.8);
if show_fault; xline(ax7, fault_time, '--', 'Color', fault_color, 'LineWidth', 1.5, 'Label', fault_label, 'LabelVerticalAlignment','bottom'); end
legend('s_1','s_2','s_3', 'Location','best');
ylabel('s');
xlabel('Time [s]');
title('Sliding Surface');
set(gca, 'FontSize', 10);

% 8. Sliding surface norm + passive fault flag
ax8 = subplot(3,3,8);
yyaxis left
semilogy(time, max(vecnorm(s_hist), 1e-9), 'Color', main_colour, 'LineWidth', 2);
hold on; grid on; box on;
yline(s_fault_thresh, 'r--', 'Detection threshold', 'LineWidth', 1.0);
ylabel('||s||  (log)');
yyaxis right
area(time, fault_flag * 0.1, 'FaceColor', flag_color, 'FaceAlpha', 0.4, ...
    'EdgeColor', 'none', 'DisplayName', 'Fault flag');
ylabel('Fault flag');
if show_fault; xline(ax8, fault_time, '--', 'Color', fault_color, 'LineWidth', 1.5); end
xlabel('Time [s]');
title('Sliding Surface Norm + Passive Fault Flag');
set(gca, 'FontSize', 10);

% 9. Quaternion components
ax9 = subplot(3,3,9);
hold on; grid on; box on;
labels_q = {'q_0','q_1','q_2','q_3'};
for k = 1:4
    plot(time, q_hist(k,:), 'Color', C(min(k,4),:), 'LineWidth', 1.5, ...
        'DisplayName', labels_q{k});
end
yline(q_d(1), 'k--', 'LineWidth', 0.8, 'HandleVisibility','off');
if show_fault; xline(ax9, fault_time, '--', 'Color', fault_color, 'LineWidth', 1.5, 'Label', fault_label, 'LabelVerticalAlignment','bottom'); end
legend('Location','best');
ylabel('q [-]');
xlabel('Time [s]');
title('Quaternion Components');
set(gca, 'FontSize', 10);


%}

% Helper functions for matrices and quaternions

% Quaternion multiplication
function q = quat_mult(q1, q2)
    w1 = q1(1); v1 = q1(2:4);
    w2 = q2(1); v2 = q2(2:4);
    q  = [w1*w2 - dot(v1,v2);
          w1*v2 + w2*v1 + cross(v1,v2)];
end
% Quaternion inversiion
function q_inv = quat_inv(q)
    q_inv = [q(1); -q(2:4)];
end
% Angle offset based on error
function angle_deg = quat_angle_error(q, q_d)
    q_e       = quat_mult(quat_inv(q_d), q);
    w_e       = min(max(abs(q_e(1)), -1.0), 1.0);
    angle_deg = 2 * acos(w_e) * 180/pi;
end
% Skew function
function S = skew(v)
    S = [  0,    -v(3),  v(2);
          v(3),   0,    -v(1);
         -v(2),  v(1),   0  ];
end
% Quaternion kinematic matrix
function X = Xi(q)
    w = q(1); v = q(2:4);
    X = [-v';
          w*eye(3) + skew(v)];
end
% Saturation boundary layer
function y = sat_vec(x, phi)
    y = min(max(x / phi, -1), 1);
end
