%  CUBESAT 3D VISUALISATION
%  Must be run after simulation file

%  Required in workspace:
%    q_hist    [4 x N]   quaternion history
%    omega_hist [3 x N]  angular velocity history
%    err_hist  [1 x N]   attitude error in degrees
%    s_hist    [3 x N]   sliding surface history
%    q_d       [4 x 1]   desired quaternion

%  Get timing from the data
N_vis        = size(q_hist, 2);
dt_vis_sim   = 0.01;    % must match sim dt
t_final_vis  = (N_vis - 1) * dt_vis_sim;
time_vis     = linspace(0, t_final_vis, N_vis);
fault_t_vis  = t_final_vis + 1;  % set lower to show fault label

% Setup render view
vis_use_stl        = true;
vis_stl_folder     = 'meshes';
vis_playback_speed = 3;          % 1 = realtime playback
vis_show_trail     = true;       % orange trail on body X axis
vis_show_ghost    = true;       % ghost cube at desired attitude
vis_frame_skip     = 5;          % render every Nth frame (higher = faster)
vis_save_gif       = false;      % true to export GIF
vis_gif_name       = 'cubesat_asmc.gif';

% Load STL files
stl_ok = false;

if vis_use_stl
    f_body = fullfile(vis_stl_folder, 'frame.stl');
    f_w1   = fullfile(vis_stl_folder, 'wheel1.stl');
    f_w2   = fullfile(vis_stl_folder, 'wheel2.stl');
    f_w3   = fullfile(vis_stl_folder, 'wheel3.stl');
    f_w4   = fullfile(vis_stl_folder, 'wheel4.stl');

    if exist(f_body, 'file')
        try
            m_body = stlread(f_body);
            m_w1   = stlread(f_w1);
            m_w2   = stlread(f_w2);
            m_w3   = stlread(f_w3);
            m_w4   = stlread(f_w4);

            sc = 0.001;   % mm to metres
            bv = m_body.Points * sc;   bf = m_body.ConnectivityList;
            v1 = m_w1.Points   * sc;   f1 = m_w1.ConnectivityList;
            v2 = m_w2.Points   * sc;   f2 = m_w2.ConnectivityList;
            v3 = m_w3.Points   * sc;   f3 = m_w3.ConnectivityList;
            v4 = m_w4.Points   * sc;   f4 = m_w4.ConnectivityList;

            % Centre each mesh at its own geometric centre
            bv = bv - mean(bv);
            v1 = v1 - mean(v1);
            v2 = v2 - mean(v2);
            v3 = v3 - mean(v3);
            v4 = v4 - mean(v4);

            all_wv = {v1, v2, v3, v4};
            all_wf = {f1, f2, f3, f4};

            stl_ok = true;
            fprintf('STL files loaded.\n');
        catch err
            fprintf('STL load failed (%s)', err.message);
        end
    else
        fprintf('Meshes folder not found \n');
    end
end

if ~stl_ok
    return
end

% Sliding surface norm
s_norm_vis = vecnorm(s_hist);

% Figure and axes
fig = figure('Position', [50 50 1300 700], ...
             'Color',    [0.06 0.06 0.10], ...
             'Name',     'CubeSat ASMC 3D Simulation');

% 3D Render window
ax3d = subplot('Position', [0.02 0.05 0.55 0.90]);
set(ax3d, 'Color',     [0.06 0.06 0.10], ...
          'XColor',    [0.65 0.65 0.65], ...
          'YColor',    [0.65 0.65 0.65], ...
          'ZColor',    [0.65 0.65 0.65], ...
          'GridColor', [0.20 0.20 0.20], ...
          'GridAlpha', 0.7);
hold(ax3d, 'on');
grid(ax3d, 'on');
axis(ax3d, 'equal');
box(ax3d,  'on');
xlabel(ax3d, 'X [m]', 'Color', [0.65 0.65 0.65]);
ylabel(ax3d, 'Y [m]', 'Color', [0.65 0.65 0.65]);
zlabel(ax3d, 'Z [m]', 'Color', [0.65 0.65 0.65]);
title(ax3d, 'CubeSat Attitude Simulation', ...
    'Color', 'w', 'FontSize', 13, 'FontWeight', 'bold');
lim = 0.13;
xlim(ax3d, [-lim lim]);
ylim(ax3d, [-lim lim]);
zlim(ax3d, [-lim lim]);
view(ax3d, 40, 22);
lighting(ax3d, 'gouraud');
light(ax3d, 'Position', [1 2 3], 'Style', 'infinite');

% World frame reference arrows
quiver3(ax3d,0,0,0, 0.10,0,0,   'Color',[1.0 0.3 0.3],'LineWidth',1.5,'AutoScale','off');
quiver3(ax3d,0,0,0, 0,0.10,0,   'Color',[0.3 1.0 0.3],'LineWidth',1.5,'AutoScale','off');
quiver3(ax3d,0,0,0, 0,0,0.10,   'Color',[0.4 0.6 1.0],'LineWidth',1.5,'AutoScale','off');
text(ax3d, 0.113, 0,     0,     'X_W','Color',[1.0 0.3 0.3],'FontSize',8,'FontWeight','bold');
text(ax3d, 0,     0.113, 0,     'Y_W','Color',[0.3 1.0 0.3],'FontSize',8,'FontWeight','bold');
text(ax3d, 0,     0,     0.113, 'Z_W','Color',[0.4 0.6 1.0],'FontSize',8,'FontWeight','bold');

% Attitude error plot
ax_err = subplot('Position', [0.62 0.55 0.36 0.38]);
set(ax_err, 'Color',     [0.08 0.08 0.12], ...
            'XColor',    'w', ...
            'YColor',    'w', ...
            'GridColor', [0.28 0.28 0.28], ...
            'GridAlpha', 0.5);
hold(ax_err, 'on');
grid(ax_err, 'on');
box(ax_err,  'on');
ylabel(ax_err, 'Error [deg]',  'Color', 'w', 'FontSize', 10);
title(ax_err,  'Attitude Error','Color', 'w', 'FontSize', 11);
xlim(ax_err, [0 t_final_vis]);
ylim(ax_err, [0 max(err_hist) * 1.15 + 1]);
% Full run background trace
plot(ax_err, time_vis, err_hist, 'Color', [0.32 0.32 0.32], 'LineWidth', 1);
yline(ax_err, 2.0, '--', 'Color', [1.0 0.60 0.10], 'LineWidth', 1.2, ...
    'Label', '2 deg threshold', 'LabelHorizontalAlignment', 'left');
% Live trace and current-point dot
err_live = plot(ax_err, NaN, NaN, 'Color', [0.20 0.90 0.60], 'LineWidth', 2.5);
err_dot  = plot(ax_err, NaN, NaN, 'o', 'Color', 'w', ...
    'MarkerFaceColor', [0.20 0.90 0.60], 'MarkerSize', 8);

% Sliding surface plot
ax_s = subplot('Position', [0.62 0.08 0.36 0.38]);
set(ax_s, 'Color',     [0.08 0.08 0.12], ...
          'XColor',    'w', ...
          'YColor',    'w', ...
          'GridColor', [0.28 0.28 0.28], ...
          'GridAlpha', 0.5);
hold(ax_s, 'on');
grid(ax_s, 'on');
box(ax_s,  'on');
ylabel(ax_s, '||s||',      'Color', 'w', 'FontSize', 10);
xlabel(ax_s, 'Time [s]',   'Color', 'w', 'FontSize', 10);
title(ax_s,  'Sliding Surface Norm', 'Color', 'w', 'FontSize', 11);
xlim(ax_s, [0 t_final_vis]);
plot(ax_s, time_vis, s_norm_vis, 'Color', [0.32 0.32 0.32], 'LineWidth', 1);
s_live = plot(ax_s, NaN, NaN, 'Color', [1.0 0.40 0.40], 'LineWidth', 2.5);
s_dot  = plot(ax_s, NaN, NaN, 'o', 'Color', 'w', ...
    'MarkerFaceColor', [1.0 0.40 0.40], 'MarkerSize', 8);

% Initial geometry
R0 = quat2rotm(q_hist(:,1)');

if stl_ok
    % Main body
    sat_body = patch(ax3d, ...
        'Vertices',  (R0 * bv')', ...
        'Faces',     bf, ...
        'FaceColor', [0.72 0.74 0.78], ...
        'EdgeColor', [0.50 0.50 0.55], ...
        'EdgeAlpha', 0.20, ...
        'FaceAlpha', 0.93, ...
        'LineWidth', 0.5);

    % Reaction wheels
    sat_wheels = gobjects(4, 1);
    for wi = 1:4
        sat_wheels(wi) = patch(ax3d, ...
            'Vertices',  (R0 * all_wv{wi}')', ...
            'Faces',     all_wf{wi}, ...
            'FaceColor', [0.18 0.18 0.22], ...
            'EdgeColor', [0.40 0.40 0.50], ...
            'EdgeAlpha', 0.12, ...
            'FaceAlpha', 0.96, ...
            'LineWidth', 0.5);
    end
end

% Target ghost (desired attitude)
if vis_show_ghost
    R_d = quat2rotm(q_d');
    if stl_ok
        patch(ax3d, ...
            'Vertices',  (R_d * bv')', ...
            'Faces',     bf, ...
            'FaceColor', [0.20 0.85 0.35], ...
            'EdgeColor', [0.30 0.95 0.45], ...
            'FaceAlpha', 0.06, ...
            'EdgeAlpha', 0.18, ...
            'LineStyle', '--', ...
            'LineWidth', 0.8);
    end
end

% Body frame axis arrows
al = 0.085;
h_bx = quiver3(ax3d,0,0,0, al,0,0, 'r','LineWidth',3,'AutoScale','off');
h_by = quiver3(ax3d,0,0,0, 0,al,0, 'g','LineWidth',3,'AutoScale','off');
h_bz = quiver3(ax3d,0,0,0, 0,0,al, 'b','LineWidth',3,'AutoScale','off');
t_bx = text(ax3d, al+0.012, 0,        0,        'X_B','Color','r',         'FontSize',9,'FontWeight','bold');
t_by = text(ax3d, 0,        al+0.012, 0,        'Y_B','Color','g',         'FontSize',9,'FontWeight','bold');
t_bz = text(ax3d, 0,        0,        al+0.012, 'Z_B','Color',[0.5 0.7 1], 'FontSize',9,'FontWeight','bold');

% Trajectory trail
trl   = 50;
tr_x  = nan(1, trl);
tr_y  = nan(1, trl);
tr_z  = nan(1, trl);
trail = plot3(ax3d, tr_x, tr_y, tr_z, ...
    'Color', [1.0 0.55 0.10 0.75], 'LineWidth', 2);

% HUD text
hud_time   = text(ax3d, -lim+0.004, -lim+0.004,  lim-0.004, 't = 0.00 s', ...
    'Color','w','FontSize',12,'FontWeight','bold');
hud_status = text(ax3d, -lim+0.004, -lim+0.004,  lim-0.018, 'Manoeuvring...', ...
    'Color',[1.0 0.85 0.20],'FontSize',10);
hud_error  = text(ax3d, -lim+0.004, -lim+0.004,  lim-0.030, 'error = 0.00 deg', ...
    'Color',[0.20 0.90 0.60],'FontSize',10);

% Animation Loop
frame_idx = 1:vis_frame_skip:N_vis;
n_frames  = length(frame_idx);
dt_pause  = dt_vis_sim * vis_frame_skip / vis_playback_speed;

fprintf('Animating %d frames at %dx speed...\n', n_frames, vis_playback_speed);

for fi = 1:n_frames
    ii = frame_idx(fi);
    t  = time_vis(ii);
    R  = quat2rotm(q_hist(:,ii)');

    % Rotate satellite
    if stl_ok
        set(sat_body, 'Vertices', (R * bv')');
        for wi = 1:4
            set(sat_wheels(wi), 'Vertices', (R * all_wv{wi}')');
        end
    else
        set(sat_body, 'Vertices', (R * cube_v')');
    end

    % Body frame axes
    xb = R * [al; 0; 0];
    yb = R * [0; al; 0];
    zb = R * [0; 0; al];
    set(h_bx, 'UData',xb(1), 'VData',xb(2), 'WData',xb(3));
    set(h_by, 'UData',yb(1), 'VData',yb(2), 'WData',yb(3));
    set(h_bz, 'UData',zb(1), 'VData',zb(2), 'WData',zb(3));
    set(t_bx, 'Position', xb' + [0.012 0     0    ]);
    set(t_by, 'Position', yb' + [0     0.012 0    ]);
    set(t_bz, 'Position', zb' + [0     0     0.012]);

    % Trail
    if vis_show_trail
        tr_x = [tr_x(2:end), xb(1)];
        tr_y = [tr_y(2:end), xb(2)];
        tr_z = [tr_z(2:end), xb(3)];
        set(trail, 'XData',tr_x, 'YData',tr_y, 'ZData',tr_z);
    end

    % HUD
    set(hud_time,  'String', sprintf('t = %.2f s',    t));
    set(hud_error, 'String', sprintf('error = %.2f deg', err_hist(ii)));
    if err_hist(ii) < 2.0
        set(hud_status, 'String', 'Settled  v', 'Color', [0.20 1.00 0.50]);
    elseif t >= fault_t_vis
        set(hud_status, 'String', 'FAULT ACTIVE', 'Color', [1.00 0.30 0.30]);
    else
        set(hud_status, 'String', 'Manoeuvring...', 'Color', [1.00 0.85 0.20]);
    end

    % Live plots
    set(err_live, 'XData', time_vis(1:ii), 'YData', err_hist(1:ii));
    set(err_dot,  'XData', t,              'YData', err_hist(ii));
    set(s_live,   'XData', time_vis(1:ii), 'YData', s_norm_vis(1:ii));
    set(s_dot,    'XData', t,              'YData', s_norm_vis(ii));

    % GIF export
    if vis_save_gif
        frm = getframe(fig);
        [imind, cm] = rgb2ind(frame2im(frm), 256);
        if fi == 1
            imwrite(imind, cm, vis_gif_name, 'gif', ...
                'Loopcount', inf, 'DelayTime', dt_pause);
        else
            imwrite(imind, cm, vis_gif_name, 'gif', ...
                'WriteMode', 'append', 'DelayTime', dt_pause);
        end
    end

    drawnow limitrate;
    pause(dt_pause);
end

fprintf('Animation complete.\n');
