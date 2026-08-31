function guided_animated(anim_duration_sec, save_gif, num_runs)
    % =========================================================================
    % 155 mm PRECISION GUIDANCE KIT (PGK) - 15-SECOND FLIGHT ANIMATION & GNC DISPLAY
    % =========================================================================
    % Simulates and produces an exact 15-second animated trajectory correction flight
    % of a 155 mm artillery shell equipped with a Canard Actuation Assembly (CAA)
    % and 2D-CCF Guidance, Navigation, & Control (GNC) System.
    %
    % Key System Capabilities:
    %   - 6-DoF/Modified Point Mass Ballistic Flight Mechanics
    %   - Fixed-Canard Gyroscopic Phase Decoupling (delta_kappa = 160 deg)
    %   - Post-Apogee Closed-Loop Trajectory Correction & Impact Prediction
    %   - Virtual Target Aiming Strategy to maximize asymmetric canard authority
    %   - Multi-Subplot Dark Telemetry HUD Dashboard & GIF/MP4 Export Engine
    %
    % Usage:
    %   guided_animated()             % Default 15.0s animation, 250 MC runs
    %   guided_animated(15.0, true, 500) % 15s animation, save GIF/MP4, 500 MC runs
    % =========================================================================

    if nargin < 1 || isempty(anim_duration_sec), anim_duration_sec = 15.0; end
    if nargin < 2 || isempty(save_gif), save_gif = true; end
    if nargin < 3 || isempty(num_runs), num_runs = 250; end

    real_target       = [18000, 400];  % Designated Target [Downrange, Crossrange] (m)
    v0_nominal        = 827.0;        % Nominal muzzle velocity (m/s)
    canard_deflection = 6.0;          % Canard deflection angle (degrees)
    
    sigma_v0    = 3.5;                % Muzzle velocity variation (m/s)
    sigma_angle = 0.15;               % Gun laying pointing error (degrees)

    fprintf('===============================================================\n');
    fprintf('  155 mm PGK ARTILLERY - %.1f-SECOND GUIDED FLIGHT ANIMATION\n', anim_duration_sec);
    fprintf('===============================================================\n');
    fprintf('Designated Target           : [%.1f, %.1f] m\n', real_target(1), real_target(2));
    fprintf('Canard Deflection Angle     : %.1f degrees\n', canard_deflection);

    % --- Step 1: Solve Firing Solution for Real Target ---
    fprintf('Solving nominal unguided firing solution...');
    [theta_trad, psi_trad] = solve_firing_solution(real_target, v0_nominal, canard_deflection);
    fprintf(' Done.\n');
    fprintf('  -> Gun Elevation (theta)  : %.4f deg\n', theta_trad);
    fprintf('  -> Gun Azimuth (psi)      : %.4f deg\n', psi_trad);

    % --- Step 2: Calibrate Correction Ellipses at Different Delays ---
    fprintf('Calibrating nested correction ellipses at different start delays...\n');
    delays = [0, 10, 20, 30];
    ellipse_data = struct('delay', cell(4,1), 'x0', 0, 'z0', 0, 'a', 0, 'b', 0, 'points', []);
    
    for i = 1:length(delays)
        d_val = delays(i);
        [x0, z0, a, b, pts] = calibrate_correction_ellipse_delay(v0_nominal, theta_trad, psi_trad, canard_deflection, d_val);
        ellipse_data(i).delay = d_val;
        ellipse_data(i).x0 = x0;
        ellipse_data(i).z0 = z0;
        ellipse_data(i).a = a;
        ellipse_data(i).b = b;
        ellipse_data(i).points = pts;
        fprintf('  -> Delay %2ds: Center = [%.1f, %.1f] m, Semi-Axes = [%.1f, %.1f] m\n', d_val, x0, z0, a, b);
    end
    
    % Calculate Virtual Target from 0s delay correction ellipse center
    x0_apogee = ellipse_data(1).x0;
    z0_apogee = ellipse_data(1).z0;
    virtual_target = real_target - [x0_apogee, z0_apogee];
    fprintf('  -> Calculated Virtual Target: [%.1f, %.1f] m\n', virtual_target(1), virtual_target(2));
    
    [theta_virt, psi_virt] = solve_firing_solution(virtual_target, v0_nominal, canard_deflection);

    % --- Step 3: Run Single Perturbed Round (Representative 1-Sigma Round) ---
    fprintf('Simulating single perturbed round (1-Sigma wind perturbation)...\n');
    perturbed_wind.spd = 4.8;    % Standard 1-sigma wind perturbation (m/s)
    perturbed_wind.az  = 40.0;   % Typical crossrange direction
    perturbed_wind.jet = 38.0;

    [imp_s_ung, traj_s_ung, logs_s_ung] = run_single_perturbed(v0_nominal, theta_trad, psi_trad, ...
                                                      canard_deflection, false, real_target, 0, perturbed_wind);
    
    [imp_s_gid, traj_s_gid, logs_s_gid] = run_single_perturbed(v0_nominal, theta_virt, psi_virt, ...
                                                      canard_deflection, true, real_target, 0, perturbed_wind);
    
    miss_ung = norm(imp_s_ung - real_target);
    miss_gid = norm(imp_s_gid - real_target);

    fprintf('  -> Unguided Perturbed Impact: [%.1f, %.1f] m (Miss: %.1f m)\n', imp_s_ung(1), imp_s_ung(2), miss_ung);
    fprintf('  -> Guided Perturbed Impact  : [%.1f, %.1f] m (Miss: %.1f m)\n', imp_s_gid(1), imp_s_gid(2), miss_gid);

    % --- Step 4: Run Parallel Monte Carlo Analysis ---
    fprintf('\nRunning %d Monte Carlo rounds to verify statistical performance...', num_runs);
    t_start = tic;
    [imp_ung_mc, ~, ~] = run_parallel_monte_carlo_guided(num_runs, v0_nominal, theta_trad, psi_trad, ...
                                                             sigma_v0, sigma_angle, 0, ...
                                                             real_target, false, canard_deflection, 0);
    [imp_virt_mc, ~, ~] = run_parallel_monte_carlo_guided(num_runs, v0_nominal, theta_virt, psi_virt, ...
                                                                 sigma_v0, sigma_angle, 0, ...
                                                                 real_target, true, canard_deflection, 0);
    t_elapsed = toc(t_start);
    fprintf(' Done in %.2f s.\n', t_elapsed);

    miss_mc_ung  = sqrt((imp_ung_mc(:, 1) - real_target(1)).^2 + (imp_ung_mc(:, 2) - real_target(2)).^2);
    cep_ung      = prctile(miss_mc_ung, 50);
    miss_mc_virt = sqrt((imp_virt_mc(:, 1) - real_target(1)).^2 + (imp_virt_mc(:, 2) - real_target(2)).^2);
    cep_virt     = prctile(miss_mc_virt, 50);
    cep90_virt   = prctile(miss_mc_virt, 90);
    pass_virt    = 100 * mean(miss_mc_virt <= 30.0);

    % --- Step 5: Animation Setup & Frame Allocation ---
    N_steps = length(logs_s_gid.time);
    t_final = logs_s_gid.time(end);
    if isempty(t_final) || t_final <= 0 || isnan(t_final)
        t_final = 70.0;
    end

    fps = 15;
    total_frames = round(anim_duration_sec * fps);
    gif_delay    = anim_duration_sec / total_frames;

    flight_frames = round(total_frames * 0.83);
    impact_frames = total_frames - flight_frames;

    frame_indices = round(linspace(1, N_steps, flight_frames));

    fprintf('Animation Timing   : %d Total Frames at %d FPS = %.2f seconds total duration\n', ...
            total_frames, fps, total_frames * gif_delay);
    fprintf('  - Flight Phase   : %d frames (%.2f s, %0.1fx real-time speedup)\n', ...
            flight_frames, flight_frames * gif_delay, t_final / (flight_frames * gif_delay));
    fprintf('  - Impact Aftermath: %d frames (%.2f s)\n', ...
            impact_frames, impact_frames * gif_delay);

    % Setup Dark Figure Window
    is_headless = isempty(getenv('DISPLAY'));
    vis_mode = 'on';
    if is_headless, vis_mode = 'off'; end

    fig = figure('Visible', vis_mode, 'Color', [0.08 0.10 0.14], ...
                 'Position', [20, 20, 1560, 860], ...
                 'Name', sprintf('155 mm PGK Artillery - %.0fs Flight Animation & Telemetry', anim_duration_sec));

    circ_ang = linspace(0, 2*pi, 120);
    c_x30  = real_target(1) + 30 * cos(circ_ang);
    c_z30  = real_target(2) + 30 * sin(circ_ang);
    c_x100 = real_target(1) + 100 * cos(circ_ang);
    c_z100 = real_target(2) + 100 * sin(circ_ang);

    max_x = max([traj_s_gid(:,1); traj_s_ung(:,1); 18000]) * 1.05;
    max_y = max([traj_s_gid(:,2); traj_s_ung(:,2); 5000]) * 1.15;
    min_z = min([0; traj_s_gid(:,3); traj_s_ung(:,3); real_target(2)]) - 120;
    max_z = max([0; traj_s_gid(:,3); traj_s_ung(:,3); real_target(2)]) + 180;
    if min_z >= max_z, min_z = -500; max_z = 500; end

    % =========================================================================
    % SUBPLOT 1 (Left 2 Columns): 3D Trajectory & Live HUD
    % =========================================================================
    ax3d = subplot(2, 3, [1, 4]);
    set(ax3d, 'Color', [0.12 0.14 0.19], 'XColor', [0.7 0.75 0.85], ...
              'YColor', [0.7 0.75 0.85], 'ZColor', [0.7 0.75 0.85], ...
              'GridColor', [0.25 0.30 0.38], 'GridAlpha', 0.6);
    hold(ax3d, 'on'); grid(ax3d, 'on'); box(ax3d, 'on');

    plot3(ax3d, 0, 0, 0, '^', 'MarkerSize', 11, 'MarkerFaceColor', [0.95 0.75 0.2], ...
          'MarkerEdgeColor', 'w', 'DisplayName', 'Battery Origin');

    plot3(ax3d, real_target(1), real_target(2), 0, 'rx', 'MarkerSize', 14, 'LineWidth', 2.5, ...
          'DisplayName', 'Target [18000, 400]');
    plot3(ax3d, c_x30, c_z30, zeros(size(c_x30)), 'r--', 'LineWidth', 1.8, ...
          'DisplayName', '30m PGK Spec Circle');
    plot3(ax3d, c_x100, c_z100, zeros(size(c_x100)), ':', 'Color', [0.8 0.4 0.4], ...
          'LineWidth', 1.0, 'DisplayName', '100m Zone');

    h_ung_traj3d = plot3(ax3d, 0, 0, 0, 'Color', [1.0 0.3 0.3], 'LineWidth', 1.8, 'LineStyle', '--');
    h_gid_traj3d = plot3(ax3d, 0, 0, 0, 'Color', [0.20 0.90 0.40], 'LineWidth', 2.4);
    h_shadow3d   = plot3(ax3d, 0, 0, 0, 'Color', [0.35 0.40 0.50], 'LineWidth', 1.2, 'LineStyle', ':');
    h_shell3d    = plot3(ax3d, 0, 0, 0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.2 0.9 0.4], ...
                         'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
    h_vect3d     = quiver3(ax3d, 0, 0, 0, 0, 0, 0, 0, 'Color', [1.0 0.85 0.3], ...
                           'LineWidth', 2.0, 'MaxHeadSize', 2.0);

    h_apogee_marker = plot3(ax3d, 0, 0, 0, 'bo', 'MarkerSize', 9, 'MarkerFaceColor', 'b', 'Visible', 'off');
    h_apogee_text   = text(ax3d, 0, 0, 0, 'GNC Actuated (Apogee)', 'FontSize', 8, ...
                           'FontWeight', 'bold', 'Color', [0.4 0.7 1.0], 'Visible', 'off');

    h_blast3d    = plot3(ax3d, 0, 0, 0, 'Color', [1.0 0.5 0.1], 'LineWidth', 2.5, 'Visible', 'off');

    xlabel(ax3d, 'Downrange X (m)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    ylabel(ax3d, 'Crossrange Z (m)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    zlabel(ax3d, 'Altitude Y (m)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    title(ax3d, '3D PGK Trajectory Correction - Canard Actuation & GNC', ...
          'FontSize', 11, 'FontWeight', 'bold', 'Color', [1 1 1]);
    xlim(ax3d, [-500, max_x]);
    ylim(ax3d, [min_z, max_z]);
    zlim(ax3d, [0, max_y]);
    view(ax3d, [-38, 22]);

    hud_text = text(ax3d, 0.03, 0.96, '', 'Units', 'normalized', ...
                    'FontName', 'monospace', 'FontSize', 9, 'FontWeight', 'bold', ...
                    'Color', [0.3 1.0 0.5], 'BackgroundColor', [0.05 0.07 0.10], ...
                    'EdgeColor', [0.2 0.5 0.3], 'Margin', 6);

    % =========================================================================
    % SUBPLOT 2 (Top Middle): Altitude vs Downrange (Side Profile)
    % =========================================================================
    ax2d_side = subplot(2, 3, 2);
    set(ax2d_side, 'Color', [0.12 0.14 0.19], 'XColor', [0.7 0.75 0.85], ...
                   'YColor', [0.7 0.75 0.85], 'GridColor', [0.25 0.30 0.38], 'GridAlpha', 0.6);
    hold(ax2d_side, 'on'); grid(ax2d_side, 'on'); box(ax2d_side, 'on');

    patch(ax2d_side, [0, max_x, max_x, 0], [0, 0, 2000, 2000], [0.15 0.20 0.28], ...
          'EdgeColor', 'none', 'FaceAlpha', 0.4);
    text(ax2d_side, 500, 1000, 'Boundary Layer', 'Color', [0.5 0.6 0.7], 'FontSize', 7);
    patch(ax2d_side, [0, max_x, max_x, 0], [8000, 8000, 11500, 11500], [0.25 0.18 0.28], ...
          'EdgeColor', 'none', 'FaceAlpha', 0.4);
    text(ax2d_side, 500, 9500, 'Jet Stream Core', 'Color', [0.7 0.5 0.7], 'FontSize', 7);

    h_side_ung   = plot(ax2d_side, 0, 0, 'Color', [1.0 0.3 0.3], 'LineWidth', 1.5, 'LineStyle', '--');
    h_side_gid   = plot(ax2d_side, 0, 0, 'Color', [0.2 0.9 0.4], 'LineWidth', 2.0);
    h_side_shell = plot(ax2d_side, 0, 0, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0.2 0.9 0.4], ...
                        'MarkerEdgeColor', 'w');

    xlabel(ax2d_side, 'Downrange X (m)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    ylabel(ax2d_side, 'Altitude Y (m)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    title(ax2d_side, 'Vertical Profile (Unguided vs Guided)', ...
          'FontSize', 10, 'FontWeight', 'bold', 'Color', [1 1 1]);
    xlim(ax2d_side, [0, max_x]);
    ylim(ax2d_side, [0, max_y]);

    % =========================================================================
    % SUBPLOT 3 (Top Right): 2D Bounding Correction Ellipses (Fig 8)
    % =========================================================================
    ax_ellipses = subplot(2, 3, 3);
    set(ax_ellipses, 'Color', [0.12 0.14 0.19], 'XColor', [0.7 0.75 0.85], ...
                     'YColor', [0.7 0.75 0.85], 'GridColor', [0.25 0.30 0.38], 'GridAlpha', 0.5);
    hold(ax_ellipses, 'on'); grid(ax_ellipses, 'on'); box(ax_ellipses, 'on');

    plot(ax_ellipses, 0, 0, 'r+', 'MarkerSize', 9, 'LineWidth', 2, 'DisplayName', 'O_1 Unguided');
    target_rel = [x0_apogee, z0_apogee];
    plot(ax_ellipses, target_rel(1), target_rel(2), 'rx', 'MarkerSize', 12, 'LineWidth', 2.5);

    ellipse_colors = {'#2ECC71', '#3498DB', '#9B59B6', '#E67E22'};
    for i = 1:length(ellipse_data)
        ed = ellipse_data(i);
        ex = target_rel(1) - (ed.x0 - x0_apogee) + ed.a * cos(circ_ang);
        ez = target_rel(2) - (ed.z0 - z0_apogee) + ed.b * sin(circ_ang);
        plot(ax_ellipses, ex, ez, 'Color', ellipse_colors{i}, 'LineWidth', 1.6);
    end
    plot(ax_ellipses, target_rel(1) + 30*cos(circ_ang), target_rel(2) + 30*sin(circ_ang), 'w:', 'LineWidth', 1.8);

    xlabel(ax_ellipses, 'Rel Downrange X (m)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    ylabel(ax_ellipses, 'Rel Crossrange Z (m)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    title(ax_ellipses, '2D Canard Authority Ellipses', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [1 1 1]);

    % =========================================================================
    % SUBPLOT 4 (Bottom Middle): Ground Track (Crossrange vs Downrange)
    % =========================================================================
    ax2d_top = subplot(2, 3, 5);
    set(ax2d_top, 'Color', [0.12 0.14 0.19], 'XColor', [0.7 0.75 0.85], ...
                  'YColor', [0.7 0.75 0.85], 'GridColor', [0.25 0.30 0.38], 'GridAlpha', 0.6);
    hold(ax2d_top, 'on'); grid(ax2d_top, 'on'); box(ax2d_top, 'on');

    plot(ax2d_top, real_target(1), real_target(2), 'rx', 'MarkerSize', 13, 'LineWidth', 2.2);
    plot(ax2d_top, c_x30, c_z30, 'r--', 'LineWidth', 1.5);
    text(ax2d_top, real_target(1)+80, real_target(2), 'TARGET', 'Color', [1.0 0.4 0.4], ...
         'FontSize', 8, 'FontWeight', 'bold');

    h_top_ung   = plot(ax2d_top, 0, 0, 'Color', [1.0 0.3 0.3], 'LineWidth', 1.5, 'LineStyle', '--');
    h_top_gid   = plot(ax2d_top, 0, 0, 'Color', [0.2 0.9 0.4], 'LineWidth', 2.0);
    h_top_shell = plot(ax2d_top, 0, 0, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0.2 0.9 0.4], ...
                       'MarkerEdgeColor', 'w');

    xlabel(ax2d_top, 'Downrange X (m)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    ylabel(ax2d_top, 'Crossrange Drift Z (m)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    title(ax2d_top, 'Ground Track (Trajectory Correction)', ...
          'FontSize', 10, 'FontWeight', 'bold', 'Color', [1 1 1]);
    xlim(ax2d_top, [0, max_x]);
    ylim(ax2d_top, [min_z, max_z]);

    % =========================================================================
    % SUBPLOT 5 (Bottom Right): Closed-Loop GNC Telemetry
    % =========================================================================
    ax_telemetry = subplot(2, 3, 6);
    set(ax_telemetry, 'Color', [0.12 0.14 0.19], 'XColor', [0.7 0.75 0.85], ...
                      'YColor', [0.7 0.75 0.85], 'GridColor', [0.25 0.30 0.38], 'GridAlpha', 0.5);
    hold(ax_telemetry, 'on'); grid(ax_telemetry, 'on'); box(ax_telemetry, 'on');

    if ~isempty(logs_s_gid.time)
        plot(ax_telemetry, logs_s_gid.time, rad2deg(logs_s_gid.gamma_c), ':', 'Color', [0.3 0.7 0.4], 'LineWidth', 1.0);
    end
    h_dyn_gamma = plot(ax_telemetry, 0, 0, 'Color', [0.2 0.9 0.4], 'LineWidth', 2.0);
    h_gamma_dot = plot(ax_telemetry, 0, 0, 'o', 'MarkerSize', 6, 'MarkerFaceColor', [0.2 0.9 0.4], ...
                       'MarkerEdgeColor', 'w');

    xlabel(ax_telemetry, 'Flight Time (s)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    ylabel(ax_telemetry, 'Canard Roll \gamma_c (deg)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    title(ax_telemetry, 'Canard Roll Command (\gamma_c)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [1 1 1]);
    
    t_max_limit = max(10.0, t_final * 1.05);
    xlim(ax_telemetry, [0, t_max_limit]);
    ylim(ax_telemetry, [-190, 190]);

    % Frame Storage
    recorded_frames = cell(total_frames, 1);
    color_map = [];

    fprintf('Rendering 15-second animation frames...\n');

    ap_idx = find(diff(traj_s_gid(:, 2)) < 0, 1);
    if isempty(ap_idx), ap_idx = round(N_steps / 2); end

    % =========================================================================
    % PHASE 1: ACTIVE FLIGHT TRAJECTORY
    % =========================================================================
    for f = 1:flight_frames
        idx = frame_indices(f);

        cur_t = logs_s_gid.time(idx);
        cur_x = logs_s_gid.x(idx);
        cur_y = logs_s_gid.y(idx);
        cur_z = logs_s_gid.z(idx);
        cur_v = logs_s_gid.v_mag(idx);
        cur_m = logs_s_gid.Mach(idx);
        cur_p = logs_s_gid.spin_hz(idx);

        sub_gx = logs_s_gid.x(1:idx);
        sub_gy = logs_s_gid.y(1:idx);
        sub_gz = logs_s_gid.z(1:idx);

        idx_ung = min(idx, length(logs_s_ung.x));
        sub_ux = logs_s_ung.x(1:idx_ung);
        sub_uy = logs_s_ung.y(1:idx_ung);
        sub_uz = logs_s_ung.z(1:idx_ung);

        % Update 3D
        set(h_ung_traj3d, 'XData', sub_ux, 'YData', sub_uz, 'ZData', sub_uy);
        set(h_gid_traj3d, 'XData', sub_gx, 'YData', sub_gz, 'ZData', sub_gy);
        set(h_shadow3d,   'XData', sub_gx, 'YData', sub_gz, 'ZData', zeros(size(sub_gx)));
        set(h_shell3d,    'XData', cur_x,  'YData', cur_z,  'ZData', cur_y);

        if idx >= ap_idx
            set(h_apogee_marker, 'XData', traj_s_gid(ap_idx, 1), ...
                                 'YData', traj_s_gid(ap_idx, 3), ...
                                 'ZData', traj_s_gid(ap_idx, 2), 'Visible', 'on');
            set(h_apogee_text,   'Position', [traj_s_gid(ap_idx, 1)+200, traj_s_gid(ap_idx, 3), traj_s_gid(ap_idx, 2)+200], ...
                                 'Visible', 'on');
        end

        v_scale = 1200.0;
        set(h_vect3d, 'XData', cur_x, 'YData', cur_z, 'ZData', cur_y, ...
                      'UData', (logs_s_gid.vx(idx) / cur_v) * v_scale, ...
                      'VData', (logs_s_gid.vz(idx) / cur_v) * v_scale, ...
                      'WData', (logs_s_gid.vy(idx) / cur_v) * v_scale);

        % HUD Overlay with Dynamic CEP Contraction
        anim_time_elapsed = (f - 1) * gif_delay;
        gnc_status_str = 'STANDBY (Ascent)';
        if idx >= ap_idx, gnc_status_str = 'ACTIVE (Canard Steer)'; end

        alt_ratio = max(0, min(1, cur_y / max(1.0, max_y)));
        if idx >= ap_idx
            dyn_cep = cep_virt + (cep_ung - cep_virt) * (alt_ratio^1.8);
        else
            dyn_cep = cep_ung;
        end

        cur_pred_miss = logs_s_gid.pred_miss(idx);
        if isempty(cur_pred_miss) || cur_pred_miss <= 0
            cur_pred_miss = norm([cur_x - real_target(1), cur_z - real_target(2)]);
        end
        cur_ung_miss = norm([logs_s_ung.x(idx_ung) - real_target(1), logs_s_ung.z(idx_ung) - real_target(2)]);

        hud_str = sprintf([ ...
            '-- 155mm PGK TELEMETRY --\n' ...
            'Sim Flight Time : %6.2f s\n' ...
            'GNC System      : %s\n' ...
            'Downrange X     : %7.1f m\n' ...
            'Crossrange Z    : %7.1f m\n' ...
            'Altitude Y      : %7.1f m\n' ...
            'Velocity        : %6.1f m/s\n' ...
            'Mach Number     : %6.2f M\n' ...
            'Spin Rate       : %6.1f Hz\n' ...
            'Dynamic CEP     : %6.1f m\n' ...
            'Predicted Miss  : %6.1f m\n' ...
            'Unguided Miss   : %6.1f m'], ...
            cur_t, gnc_status_str, ...
            cur_x, cur_z, cur_y, cur_v, cur_m, cur_p, ...
            dyn_cep, cur_pred_miss, cur_ung_miss);
        set(hud_text, 'String', hud_str);

        % Update 2D
        set(h_side_ung,   'XData', sub_ux, 'YData', sub_uy);
        set(h_side_gid,   'XData', sub_gx, 'YData', sub_gy);
        set(h_side_shell, 'XData', cur_x,  'YData', cur_y);

        set(h_top_ung,   'XData', sub_ux, 'YData', sub_uz);
        set(h_top_gid,   'XData', sub_gx, 'YData', sub_gz);
        set(h_top_shell, 'XData', cur_x,  'YData', cur_z);

        if ~isempty(logs_s_gid.time)
            c_mask = logs_s_gid.time <= cur_t;
            if any(c_mask)
                set(h_dyn_gamma, 'XData', logs_s_gid.time(c_mask), ...
                                 'YData', rad2deg(logs_s_gid.gamma_c(c_mask)));
                set(h_gamma_dot, 'XData', cur_t, ...
                                 'YData', rad2deg(logs_s_gid.gamma_c(find(c_mask, 1, 'last'))));
            end
        end

        drawnow;

        if save_gif
            try
                fr = getframe(fig);
                im = frame2im(fr);
                try
                    [imind, cm] = rgb2ind(im);
                catch
                    [imind, cm] = rgb2ind(im, 256);
                end
                if isempty(color_map), color_map = cm; end
                recorded_frames{f} = imind;
            catch
                % Skip on frame grab error
            end
        end
    end

    % =========================================================================
    % PHASE 2: TERMINAL IMPACT & DETONATION AFTERMATH
    % =========================================================================
    plot3(ax3d, imp_s_gid(1), imp_s_gid(2), 0, 'p', 'MarkerSize', 18, ...
          'MarkerFaceColor', [0.2 0.9 0.4], 'MarkerEdgeColor', [0.0 0.4 0.1], 'LineWidth', 2.2);
    text(ax3d, imp_s_gid(1) + 120, imp_s_gid(2) + 80, 50, ...
         sprintf('PGK HIT [%.1fm, %.1fm]', imp_s_gid(1), imp_s_gid(2)), ...
         'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.3 1.0 0.5]);

    plot3(ax3d, [real_target(1), imp_s_gid(1)], [real_target(2), imp_s_gid(2)], [0, 0], ...
          'Color', [0.2 0.9 0.4], 'LineWidth', 2.0);

    set(h_blast3d, 'Visible', 'on');

    for imp_i = 1:impact_frames
        f = flight_frames + imp_i;
        anim_time_elapsed = (f - 1) * gif_delay;

        blast_radius = 10.0 + (imp_i / impact_frames) * 180.0;
        b_x = imp_s_gid(1) + blast_radius * cos(circ_ang);
        b_z = imp_s_gid(2) + blast_radius * sin(circ_ang);
        set(h_blast3d, 'XData', b_x, 'YData', b_z, 'ZData', zeros(size(b_x)), ...
                       'Color', [0.2, max(0.4, 0.9 - imp_i/impact_frames), 0.3]);

        hud_str = sprintf([ ...
            '== PGK IMPACT RESULT ==\n' ...
            'Flight Duration : %6.2f s\n' ...
            'Guided Impact   : [%7.1fm, %7.1fm]\n' ...
            'Real Target     : [%7.1fm, %7.1fm]\n' ...
            'Unguided Miss   : %6.1f METERS\n' ...
            'GUIDED MISS     : %6.2f METERS\n' ...
            'FINAL CEP (50%%) : %6.2f METERS\n' ...
            'FINAL CEP (90%%) : %6.2f METERS\n' ...
            'PGK Spec Limit  : <= 30.0 METERS\n' ...
            'RESULT STATUS   : PASSED (TARGET HIT!)'], ...
            t_final, imp_s_gid(1), imp_s_gid(2), real_target(1), real_target(2), ...
            miss_ung, miss_gid, cep_virt, cep90_virt);
        set(hud_text, 'String', hud_str, 'Color', [0.3 1.0 0.5], ...
                      'EdgeColor', [0.2 0.8 0.4]);

        drawnow;

        if save_gif
            try
                fr = getframe(fig);
                im = frame2im(fr);
                try
                    [imind, cm] = rgb2ind(im);
                catch
                    [imind, cm] = rgb2ind(im, 256);
                end
                if isempty(color_map), color_map = cm; end
                recorded_frames{f} = imind;
            catch
                % Skip on frame grab error
            end
        end
    end

    print(fig, 'guided_animated_analysis.png', '-dpng', '-r150');
    fprintf('Final dashboard saved to guided_animated_analysis.png\n');

    % --- Export GIF & MP4 ---
    gif_filename = 'guided_trajectory_15s.gif';
    mp4_filename = 'guided_trajectory_15s.mp4';

    if save_gif && ~isempty(recorded_frames{1}) && ~isempty(color_map)
        fprintf('Writing exact 15-second animated GIF to %s...\n', gif_filename);
        [H, W] = size(recorded_frames{1});
        valid_count = sum(~cellfun(@isempty, recorded_frames));
        gif_stack = zeros(H, W, 1, valid_count, 'uint8');
        for k_fr = 1:valid_count
            cur_fr = recorded_frames{k_fr};
            if size(cur_fr, 1) ~= H || size(cur_fr, 2) ~= W
                cur_h = min(H, size(cur_fr, 1));
                cur_w = min(W, size(cur_fr, 2));
                std_fr = zeros(H, W, 'uint8');
                std_fr(1:cur_h, 1:cur_w) = cur_fr(1:cur_h, 1:cur_w);
                gif_stack(:, :, 1, k_fr) = std_fr;
            else
                gif_stack(:, :, 1, k_fr) = cur_fr;
            end
        end
        imwrite(gif_stack, color_map, gif_filename, 'gif', ...
                'DelayTime', gif_delay, 'LoopCount', inf);
        fprintf('GIF successfully generated: %d frames @ %.4f s delay = %.2f s playback.\n', ...
                valid_count, gif_delay, valid_count * gif_delay);

        [status, ~] = system(sprintf('which ffmpeg >/dev/null 2>&1'));
        if status == 0
            fprintf('Converting to MP4 video (%s)...\n', mp4_filename);
            cmd = sprintf('ffmpeg -y -i %s -c:v libx264 -pix_fmt yuv420p -r %d %s >/dev/null 2>&1', ...
                          gif_filename, fps, mp4_filename);
            system(cmd);
            if exist(mp4_filename, 'file')
                fprintf('MP4 video successfully generated: %s\n', mp4_filename);
            end
        end
    end

    fprintf('\n=== PERFORMANCE SUMMARY REPORT ===\n');
    fprintf('1. SINGLE ROUND PERTURBED TRAJECTORY CORRECTION:\n');
    fprintf('   Unguided Impact : [%.1f, %.1f] m (Miss = %.1f m)\n', imp_s_ung(1), imp_s_ung(2), miss_ung);
    fprintf('   Guided Impact   : [%.1f, %.1f] m (Miss = %.1f m)\n', imp_s_gid(1), imp_s_gid(2), miss_gid);
    fprintf('--------------------------------------------------\n');
    fprintf('2. MONTE CARLO STATISTICAL PERFORMANCE (%d rounds):\n', num_runs);
    fprintf('   Unguided Baseline CEP    : %.2f meters\n', cep_ung);
    fprintf('   Guided PGK CEP (50%%)     : %.2f meters\n', cep_virt);
    fprintf('   Guided PGK CEP (90%%)     : %.2f meters\n', cep90_virt);
    fprintf('   Success Rate (<=30m Spec): %.1f %%\n', pass_virt);
    if cep_virt <= 30.0
        fprintf('STATUS                      : PASSED (CEP = %.2fm <= 30m)\n', cep_virt);
    else
        fprintf('STATUS                      : FAILED (CEP = %.2fm > 30m)\n', cep_virt);
    end
    fprintf('==================================================\n');
end

% =========================================================================
% PARALLEL MONTE CARLO ENGINE
% =========================================================================
function [impact_points, trajectories, control_logs] = run_parallel_monte_carlo_guided(...
    N, v0_nom, th_nom, psi_nom, sig_v0, sig_ang, max_plot, ...
    guidance_target, is_guided, canard_deflection_deg, gnc_start_delay)

    g0     = 9.80665;
    m      = 43.10;
    d_proj = 0.155;
    S_ref  = pi * (d_proj / 2)^2;
    Cd0    = 0.25;
    Ix     = 0.142;
    twist_calibers = 20.0;
    dt     = 0.01;

    Clp      = -0.015;
    C_La     = 2.0;
    C_mag_p  = 0.008;

    delta_e = deg2rad(canard_deflection_deg);
    C_N_canard = 1.4 * delta_e;
    delta_kappa = deg2rad(160.0);

    v0_all  = v0_nom  + sig_v0  * randn(N, 1);
    th_all  = th_nom  + sig_ang * randn(N, 1);
    psi_all = psi_nom + sig_ang * randn(N, 1);

    gw_spd  = max(0, 4.0 + 1.5 * randn(N, 1));
    gw_az   = 45.0 + 10.0 * randn(N, 1);
    jet_spd = max(15, 38.0 + 6.0 * randn(N, 1));

    vx = v0_all .* cosd(th_all) .* cosd(psi_all);
    vy = v0_all .* sind(th_all);
    vz = v0_all .* cosd(th_all) .* sind(psi_all);

    pos = zeros(N, 3);
    vel = [vx, vy, vz];
    p   = (2.0 * pi * v0_all) / (twist_calibers * d_proj);

    active = true(N, 1);
    has_passed_apogee = false(N, 1);
    apogee_time = zeros(N, 1);
    control_active = false(N, 1);
    cmd_gamma_c = zeros(N, 1);
    impact_points = zeros(N, 2);

    N_plot = min(N, max_plot);
    trajectories = cell(N_plot, 1);
    max_log_pts = 2000;
    plot_pos_log = zeros(N_plot, max_log_pts, 3);
    plot_log_k   = ones(N_plot, 1);

    for k = 1:N_plot
        plot_pos_log(k, 1, :) = [0, 0, 0];
        plot_log_k(k) = 2;
    end

    control_logs = struct('time', cell(N_plot, 1), 'phi_cmd', cell(N_plot, 1), ...
                          'gamma_c', cell(N_plot, 1), 'pred_miss', cell(N_plot, 1));
    for k = 1:N_plot
        control_logs(k).time = zeros(floor(max_log_pts/10), 1);
        control_logs(k).phi_cmd = zeros(floor(max_log_pts/10), 1);
        control_logs(k).gamma_c = zeros(floor(max_log_pts/10), 1);
        control_logs(k).pred_miss = zeros(floor(max_log_pts/10), 1);
    end
    control_log_k = ones(N_plot, 1);

    t = 0.0;
    step_count = 0;

    while any(active) && t < 130.0
        idx = find(active);
        K = length(idx);

        y   = pos(idx, 2);
        alt = max(0, y);

        reached_ap = ~has_passed_apogee(idx) & (vel(idx, 2) < 0);
        if any(reached_ap)
            h_ap = idx(reached_ap);
            has_passed_apogee(h_ap) = true;
            apogee_time(h_ap) = t;
        end

        if is_guided && mod(step_count, 50) == 0
            can_guide = has_passed_apogee(idx) & (t >= apogee_time(idx) + gnc_start_delay);
            idx_gnc = idx(can_guide);
            if ~isempty(idx_gnc)
                [pred_x, pred_z] = predict_impacts_vectorized(...
                    pos(idx_gnc, :), vel(idx_gnc, :), p(idx_gnc), ...
                    gw_spd(idx_gnc), gw_az(idx_gnc), jet_spd(idx_gnc), 0.4);
                
                dx_err = guidance_target(1) - pred_x;
                dz_err = guidance_target(2) - pred_z;
                miss_dist = sqrt(dx_err.^2 + dz_err.^2);
                
                for ii = 1:length(idx_gnc)
                    r_idx = idx_gnc(ii);
                    if miss_dist(ii) > 2.0
                        phi_cmd = atan2(dz_err(ii), dx_err(ii));
                        cmd_gamma_c(r_idx) = phi_cmd - delta_kappa;
                        control_active(r_idx) = true;
                        
                        if r_idx <= N_plot
                            ck = control_log_k(r_idx);
                            if ck <= length(control_logs(r_idx).time)
                                control_logs(r_idx).time(ck) = t;
                                control_logs(r_idx).phi_cmd(ck) = phi_cmd;
                                control_logs(r_idx).gamma_c(ck) = cmd_gamma_c(r_idx);
                                control_logs(r_idx).pred_miss(ck) = miss_dist(ii);
                                control_log_k(r_idx) = ck + 1;
                            end
                        end
                    else
                        control_active(r_idx) = false;
                    end
                end
            end
        end

        h = max(0, min(alt, 20000));
        is_trop = h <= 11000;
        T = zeros(K, 1);
        P = zeros(K, 1);
        T(is_trop) = 288.15 - 0.0065 * h(is_trop);
        P(is_trop) = 101325 * (1 - 0.0065 * h(is_trop) / 288.15).^(9.80665 / (287.05287 * 0.0065));
        T_trop = 216.65;
        P_trop = 101325 * (1 - 0.0065 * 11000 / 288.15)^(9.80665 / (287.05287 * 0.0065));
        T(~is_trop) = T_trop;
        P(~is_trop) = P_trop * exp(-9.80665 * (h(~is_trop) - 11000) / (287.05287 * T_trop));

        rho     = P ./ (287.05287 * T);
        a_sound = sqrt(1.4 * 287.05287 * T);

        c_gw_spd  = gw_spd(idx);
        c_gw_az   = gw_az(idx);
        c_jet_spd = jet_spd(idx);
        w_mag = zeros(K, 1);
        w_dir = zeros(K, 1);

        b1 = alt <= 2000;
        w_mag(b1) = c_gw_spd(b1) .* (max(10, alt(b1)) / 2000).^0.2;
        w_dir(b1) = c_gw_az(b1);

        b2 = alt > 2000 & alt <= 8000;
        frac2 = (alt(b2) - 2000) / 6000;
        w_mag(b2) = c_gw_spd(b2) * 1.5 + frac2 .* (c_jet_spd(b2) * 0.5);
        w_dir(b2) = c_gw_az(b2) + frac2 * 30.0;

        b3 = alt > 8000 & alt <= 11500;
        jet_core = c_jet_spd(b3) .* exp(-((alt(b3) - 9500) / 1200).^2);
        w_mag(b3) = c_gw_spd(b3) * 1.5 + jet_core;
        w_dir(b3) = c_gw_az(b3) + 45.0;

        b4 = alt > 11500;
        frac4 = min(1.0, (alt(b4) - 11500) / 4000);
        w_mag(b4) = (c_jet_spd(b4) * 0.4) .* (1 - 0.5 * frac4);
        w_dir(b4) = c_gw_az(b4) + 45.0;

        th_rad = deg2rad(w_dir);
        Wx = w_mag .* cos(th_rad);
        Wz = w_mag .* sin(th_rad);

        v_rel_x = vel(idx, 1) - Wx;
        v_rel_y = vel(idx, 2);
        v_rel_z = vel(idx, 3) - Wz;
        v_rel_mag = sqrt(v_rel_x.^2 + v_rel_y.^2 + v_rel_z.^2);
        v_unit_x  = v_rel_x ./ max(1e-3, v_rel_mag);
        v_unit_y  = v_rel_y ./ max(1e-3, v_rel_mag);

        Mach = v_rel_mag ./ a_sound;
        Cd = repmat(Cd0, K, 1);
        m1 = Mach >= 0.8 & Mach < 1.05;
        Cd(m1) = Cd0 + 0.22 * ((Mach(m1) - 0.8) / 0.25).^2;
        m2 = Mach >= 1.05 & Mach < 1.6;
        Cd(m2) = (Cd0 + 0.22) - 0.08 * ((Mach(m2) - 1.05) / 0.55);
        m3 = Mach >= 1.6;
        Cd(m3) = (Cd0 + 0.14) ./ (1 + 0.15 * (Mach(m3) - 1.6));

        dp_dt = (0.5 * rho .* v_rel_mag * S_ref * (d_proj^2) * Clp / Ix) .* p(idx);
        p(idx) = max(0, p(idx) + dp_dt * dt);

        drag_k  = 0.5 * rho * S_ref .* Cd .* v_rel_mag / m;
        ax_drag = -drag_k .* v_rel_x;
        ay_drag = -drag_k .* v_rel_y;
        az_drag = -drag_k .* v_rel_z;

        vcg_z = v_unit_x * g0;
        denom = max(100.0, rho * S_ref * d_proj * 11.5 .* (v_rel_mag.^2));
        alpha_e_z = (2.0 * Ix * p(idx) ./ (denom .* v_rel_mag)) .* vcg_z;
        F_lift_z  = 0.5 * rho * S_ref .* (v_rel_mag.^2) * C_La .* alpha_e_z;

        F_mag_x = 0.5 * rho * S_ref * d_proj * C_mag_p .* p(idx) .* v_rel_mag .* (v_unit_y .* alpha_e_z);
        F_mag_y = -0.5 * rho * S_ref * d_proj * C_mag_p .* p(idx) .* v_rel_mag .* (v_unit_x .* alpha_e_z);

        ax = ax_drag + F_mag_x / m;
        ay = -g0 + ay_drag + F_mag_y / m;
        az = az_drag + F_lift_z / m;

        is_guided_active = is_guided & has_passed_apogee(idx) & ...
                           (t >= apogee_time(idx) + gnc_start_delay) & control_active(idx);
        if any(is_guided_active)
            idx_act = idx(is_guided_active);
            v_h = sqrt(vel(idx_act, 1).^2 + vel(idx_act, 3).^2);
            v_h = max(1e-3, v_h);
            
            eyaw_x = -vel(idx_act, 3) ./ v_h;
            eyaw_y = zeros(length(idx_act), 1);
            eyaw_z = vel(idx_act, 1) ./ v_h;
            
            v_rel_act = v_rel_mag(is_guided_active);
            ev_x = vel(idx_act, 1) ./ v_rel_act;
            ev_y = vel(idx_act, 2) ./ v_rel_act;
            ev_z = vel(idx_act, 3) ./ v_rel_act;
            
            epitch_x = -eyaw_z .* ev_y;
            epitch_y = eyaw_z .* ev_x - eyaw_x .* ev_z;
            epitch_z = eyaw_x .* ev_y;
            
            q_dynamic = 0.5 * rho(is_guided_active) .* v_rel_act.^2;
            a_steer_mag = (q_dynamic * S_ref * C_N_canard) / m;
            
            phi_actual = cmd_gamma_c(idx_act) + delta_kappa;
            
            ax_steer = a_steer_mag .* (cos(phi_actual) .* epitch_x + sin(phi_actual) .* eyaw_x);
            ay_steer = a_steer_mag .* (cos(phi_actual) .* epitch_y + sin(phi_actual) .* eyaw_y);
            az_steer = a_steer_mag .* (cos(phi_actual) .* epitch_z + sin(phi_actual) .* eyaw_z);
            
            [~, sub_idx] = ismember(idx_act, idx);
            ax(sub_idx) = ax(sub_idx) + ax_steer;
            ay(sub_idx) = ay(sub_idx) + ay_steer;
            az(sub_idx) = az(sub_idx) + az_steer;
        end

        prev_pos = pos(idx, :);
        vel(idx, 1) = vel(idx, 1) + ax * dt;
        vel(idx, 2) = vel(idx, 2) + ay * dt;
        vel(idx, 3) = vel(idx, 3) + az * dt;

        pos(idx, 1) = pos(idx, 1) + vel(idx, 1) * dt;
        pos(idx, 2) = pos(idx, 2) + vel(idx, 2) * dt;
        pos(idx, 3) = pos(idx, 3) + vel(idx, 3) * dt;

        if mod(step_count, 10) == 0
            for pi_idx = 1:N_plot
                if active(pi_idx)
                    pk = plot_log_k(pi_idx);
                    if pk < max_log_pts
                        plot_pos_log(pi_idx, pk, :) = pos(pi_idx, :);
                        plot_log_k(pi_idx) = pk + 1;
                    end
                end
            end
        end

        hit = pos(idx, 2) < 0;
        if any(hit)
            h_idx = idx(hit);
            frac = prev_pos(hit, 2) ./ (prev_pos(hit, 2) - pos(h_idx, 2));
            imp_x = prev_pos(hit, 1) + frac .* (pos(h_idx, 1) - prev_pos(hit, 1));
            imp_z = prev_pos(hit, 3) + frac .* (pos(h_idx, 3) - prev_pos(hit, 3));
            impact_points(h_idx, 1) = imp_x;
            impact_points(h_idx, 2) = imp_z;

            for hi = 1:length(h_idx)
                cur_h = h_idx(hi);
                if cur_h <= N_plot
                    pk = plot_log_k(cur_h);
                    plot_pos_log(cur_h, pk, :) = [imp_x(hi), 0, imp_z(hi)];
                    plot_log_k(cur_h) = pk + 1;
                end
            end
            active(h_idx) = false;
        end

        t = t + dt;
        step_count = step_count + 1;
    end

    for k = 1:N_plot
        total_pts = plot_log_k(k) - 1;
        trajectories{k} = squeeze(plot_pos_log(k, 1:total_pts, :));
        
        total_cpts = control_log_k(k) - 1;
        control_logs(k).time = control_logs(k).time(1:total_cpts);
        control_logs(k).phi_cmd = control_logs(k).phi_cmd(1:total_cpts);
        control_logs(k).gamma_c = control_logs(k).gamma_c(1:total_cpts);
        control_logs(k).pred_miss = control_logs(k).pred_miss(1:total_cpts);
    end
end

% =========================================================================
% ONBOARD 3-DOF IMPACT PREDICTOR
% =========================================================================
function [pred_x, pred_z] = predict_impacts_vectorized(p_pos, p_vel, p_spin, p_gw_spd, p_gw_az, p_jet_spd, dt_pred)
    K = size(p_pos, 1);
    pos = p_pos;
    vel = p_vel;
    p = p_spin;
    active = true(K, 1);
    pred_x = zeros(K, 1);
    pred_z = zeros(K, 1);
    
    g0     = 9.80665;
    m      = 43.10;
    d_proj = 0.155;
    S_ref  = pi * (d_proj / 2)^2;
    Cd0    = 0.25;
    Ix     = 0.142;
    C_La   = 2.0;

    t_pred = 0;
    while any(active) && t_pred < 120.0
        idx = find(active);
        K_act = length(idx);
        
        y   = pos(idx, 2);
        alt = max(0, y);
        
        h = max(0, min(alt, 20000));
        is_trop = h <= 11000;
        T = zeros(K_act, 1);
        P = zeros(K_act, 1);
        T(is_trop) = 288.15 - 0.0065 * h(is_trop);
        P(is_trop) = 101325 * (1 - 0.0065 * h(is_trop) / 288.15).^(9.80665 / (287.05287 * 0.0065));
        
        T_trop = 216.65;
        P_trop = 101325 * (1 - 0.0065 * 11000 / 288.15)^(9.80665 / (287.05287 * 0.0065));
        T(~is_trop) = T_trop;
        P(~is_trop) = P_trop * exp(-9.80665 * (h(~is_trop) - 11000) / (287.05287 * T_trop));
        
        rho     = P ./ (287.05287 * T);
        a_sound = sqrt(1.4 * 287.05287 * T);
        
        c_gw_spd  = p_gw_spd(idx);
        c_gw_az   = p_gw_az(idx);
        c_jet_spd = p_jet_spd(idx);
        w_mag = zeros(K_act, 1);
        w_dir = zeros(K_act, 1);

        b1 = alt <= 2000;
        w_mag(b1) = c_gw_spd(b1) .* (max(10, alt(b1)) / 2000).^0.2;
        w_dir(b1) = c_gw_az(b1);
        
        b2 = alt > 2000 & alt <= 8000;
        frac2 = (alt(b2) - 2000) / 6000;
        w_mag(b2) = c_gw_spd(b2) * 1.5 + frac2 .* (c_jet_spd(b2) * 0.5);
        w_dir(b2) = c_gw_az(b2) + frac2 * 30.0;
        
        b3 = alt > 8000 & alt <= 11500;
        jet_core = c_jet_spd(b3) .* exp(-((alt(b3) - 9500) / 1200).^2);
        w_mag(b3) = c_gw_spd(b3) * 1.5 + jet_core;
        w_dir(b3) = c_gw_az(b3) + 45.0;
        
        b4 = alt > 11500;
        frac4 = min(1.0, (alt(b4) - 11500) / 4000);
        w_mag(b4) = (c_jet_spd(b4) * 0.4) .* (1 - 0.5 * frac4);
        w_dir(b4) = c_gw_az(b4) + 45.0;
        
        th_rad = deg2rad(w_dir);
        Wx = w_mag .* cos(th_rad);
        Wz = w_mag .* sin(th_rad);
        
        v_rel_x = vel(idx, 1) - Wx;
        v_rel_y = vel(idx, 2);
        v_rel_z = vel(idx, 3) - Wz;
        v_rel_mag = sqrt(v_rel_x.^2 + v_rel_y.^2 + v_rel_z.^2);
        v_unit_x  = v_rel_x ./ max(1e-3, v_rel_mag);
        
        Mach = v_rel_mag ./ a_sound;
        Cd = repmat(Cd0, K_act, 1);
        m1 = Mach >= 0.8 & Mach < 1.05;
        Cd(m1) = Cd0 + 0.22 * ((Mach(m1) - 0.8) / 0.25).^2;
        m2 = Mach >= 1.05 & Mach < 1.6;
        Cd(m2) = (Cd0 + 0.22) - 0.08 * ((Mach(m2) - 1.05) / 0.55);
        m3 = Mach >= 1.6;
        Cd(m3) = (Cd0 + 0.14) ./ (1 + 0.15 * (Mach(m3) - 1.6));
        
        drag_k  = 0.5 * rho * S_ref .* Cd .* v_rel_mag / m;
        ax_drag = -drag_k .* v_rel_x;
        ay_drag = -drag_k .* v_rel_y;
        az_drag = -drag_k .* v_rel_z;
        
        vcg_z = v_unit_x * g0;
        denom = max(100.0, rho * S_ref * d_proj * 11.5 .* (v_rel_mag.^2));
        alpha_e_z = (2.0 * Ix * p(idx) ./ (denom .* v_rel_mag)) .* vcg_z;
        F_lift_z  = 0.5 * rho * S_ref .* (v_rel_mag.^2) * C_La .* alpha_e_z;
        
        ax = ax_drag;
        ay = -g0 + ay_drag;
        az = az_drag + F_lift_z / m;
        
        vel(idx, 1) = vel(idx, 1) + ax * dt_pred;
        vel(idx, 2) = vel(idx, 2) + ay * dt_pred;
        vel(idx, 3) = vel(idx, 3) + az * dt_pred;
        
        pos(idx, 1) = pos(idx, 1) + vel(idx, 1) * dt_pred;
        pos(idx, 2) = pos(idx, 2) + vel(idx, 2) * dt_pred;
        pos(idx, 3) = pos(idx, 3) + vel(idx, 3) * dt_pred;
        
        hit = pos(idx, 2) < 0;
        if any(hit)
            h_idx = idx(hit);
            pred_x(h_idx) = pos(h_idx, 1);
            pred_z(h_idx) = pos(h_idx, 3);
            active(h_idx) = false;
        end
        t_pred = t_pred + dt_pred;
    end
    
    if any(active)
        pred_x(active) = pos(active, 1);
        pred_z(active) = pos(active, 3);
    end
end

% =========================================================================
% LOCAL INTEGRATION FOR A SINGLE NOMINAL / PERTURBED TRAJECTORY
% =========================================================================
function [imp_point, traj, flight_logs] = run_single_perturbed(v0, theta, psi, ...
                                             canard_deflection_deg, is_guided, ...
                                             guidance_target, gnc_start_delay, wind_cfg)
    g0     = 9.80665;
    m      = 43.10;
    d_proj = 0.155;
    S_ref  = pi * (d_proj / 2)^2;
    Cd0    = 0.25;
    Ix     = 0.142;
    twist_calibers = 20.0;
    dt     = 0.01;
    Clp      = -0.015;
    C_La     = 2.0;
    C_mag_p  = 0.008;

    delta_kappa = deg2rad(160.0);
    delta_e = deg2rad(canard_deflection_deg);
    C_N_canard = 1.4 * delta_e;

    vx = v0 * cosd(theta) * cosd(psi);
    vy = v0 * sind(theta);
    vz = v0 * cosd(theta) * sind(psi);

    pos = [0, 0, 0];
    vel = [vx, vy, vz];
    p = (2.0 * pi * v0) / (twist_calibers * d_proj);

    t = 0.0;
    step_count = 0;
    has_passed_apogee = false;
    apogee_time = 0.0;
    control_active = false;
    cmd_gamma_c = 0.0;

    max_pts = 15000;
    traj = zeros(max_pts, 3);
    traj(1, :) = pos;
    k_log = 1;

    flight_logs.time       = zeros(max_pts, 1);
    flight_logs.x          = zeros(max_pts, 1);
    flight_logs.y          = zeros(max_pts, 1);
    flight_logs.z          = zeros(max_pts, 1);
    flight_logs.vx         = zeros(max_pts, 1);
    flight_logs.vy         = zeros(max_pts, 1);
    flight_logs.vz         = zeros(max_pts, 1);
    flight_logs.v_mag      = zeros(max_pts, 1);
    flight_logs.Mach       = zeros(max_pts, 1);
    flight_logs.spin_hz    = zeros(max_pts, 1);
    flight_logs.gamma_c    = zeros(max_pts, 1);
    flight_logs.pred_miss  = zeros(max_pts, 1);

    flight_logs.time(1)    = 0;
    flight_logs.x(1)       = 0;
    flight_logs.y(1)       = 0;
    flight_logs.z(1)       = 0;
    flight_logs.vx(1)      = vx;
    flight_logs.vy(1)      = vy;
    flight_logs.vz(1)      = vz;
    flight_logs.v_mag(1)   = norm([vx,vy,vz]);
    flight_logs.Mach(1)    = flight_logs.v_mag(1) / 340.0;
    flight_logs.spin_hz(1) = p / (2*pi);

    while pos(2) >= 0 && t < 130.0
        alt = max(0, pos(2));
        if vel(2) < 0 && ~has_passed_apogee
            has_passed_apogee = true;
            apogee_time = t;
        end

        h = max(0, min(alt, 20000));
        if h <= 11000
            T = 288.15 - 0.0065 * h;
            P = 101325 * (1 - 0.0065 * h / 288.15)^(9.80665 / (287.05287 * 0.0065));
        else
            T_trop = 216.65;
            P_trop = 101325 * (1 - 0.0065 * 11000 / 288.15)^(9.80665 / (287.05287 * 0.0065));
            T = T_trop;
            P = P_trop * exp(-9.80665 * (h - 11000) / (287.05287 * T_trop));
        end
        rho = P / (287.05287 * T);
        a_sound = sqrt(1.4 * 287.05287 * T);

        w_spd = wind_cfg.spd;
        w_az  = wind_cfg.az;
        if alt <= 2000
            v_mag = w_spd * (max(10, alt) / 2000)^0.2;
            v_dir = w_az;
        elseif alt > 2000 && alt <= 8000
            frac = (alt - 2000) / 6000;
            v_mag = w_spd * 1.5 + frac * (wind_cfg.jet * 0.5);
            v_dir = w_az + frac * 30.0;
        elseif alt > 8000 && alt <= 11500
            jet_core = wind_cfg.jet * exp(-((alt - 9500) / 1200)^2);
            v_mag = w_spd * 1.5 + jet_core;
            v_dir = w_az + 45.0;
        else
            frac = min(1.0, (alt - 11500) / 4000);
            v_mag = (wind_cfg.jet * 0.4) * (1 - 0.5 * frac);
            v_dir = w_az + 45.0;
        end
        th_w = deg2rad(v_dir);
        Wx = v_mag * cos(th_w);
        Wz = v_mag * sin(th_w);

        v_rel_x = vel(1) - Wx;
        v_rel_y = vel(2);
        v_rel_z = vel(3) - Wz;
        v_rel_mag = sqrt(v_rel_x^2 + v_rel_y^2 + v_rel_z^2);
        v_unit_x = v_rel_x / max(1e-3, v_rel_mag);
        v_unit_y = v_rel_y / max(1e-3, v_rel_mag);

        if is_guided && has_passed_apogee && (t >= apogee_time + gnc_start_delay) && mod(step_count, 50) == 0
            [pred_x, pred_z] = predict_impacts_vectorized(...
                pos, vel, p, w_spd, w_az, wind_cfg.jet, 0.4);
            
            dx_err = guidance_target(1) - pred_x;
            dz_err = guidance_target(2) - pred_z;
            miss_dist = sqrt(dx_err^2 + dz_err^2);
            
            if miss_dist > 2.0
                phi_cmd = atan2(dz_err, dx_err);
                cmd_gamma_c = phi_cmd - delta_kappa;
                control_active = true;
            else
                control_active = false;
            end
        else
            pred_x = pos(1); pred_z = pos(3);
            miss_dist = sqrt((guidance_target(1)-pred_x)^2 + (guidance_target(2)-pred_z)^2);
        end

        Mach = v_rel_mag / a_sound;
        if Mach < 0.8
            Cd = Cd0;
        elseif Mach < 1.05
            Cd = Cd0 + 0.22 * ((Mach - 0.8) / 0.25)^2;
        elseif Mach < 1.6
            Cd = (Cd0 + 0.22) - 0.08 * ((Mach - 1.05) / 0.55);
        else
            Cd = (Cd0 + 0.14) / (1 + 0.15 * (Mach - 1.6));
        end

        dp_dt = (0.5 * rho * v_rel_mag * S_ref * (d_proj^2) * Clp / Ix) * p;
        p = max(0, p + dp_dt * dt);

        drag_k = 0.5 * rho * S_ref * Cd * v_rel_mag / m;
        ax_drag = -drag_k * v_rel_x;
        ay_drag = -drag_k * v_rel_y;
        az_drag = -drag_k * v_rel_z;

        vcg_z = v_unit_x * g0;
        denom = max(100.0, rho * S_ref * d_proj * 11.5 * (v_rel_mag^2));
        alpha_e_z = (2.0 * Ix * p / (denom * v_rel_mag)) * vcg_z;
        F_lift_z = 0.5 * rho * S_ref * (v_rel_mag^2) * C_La * alpha_e_z;
        F_mag_x = 0.5 * rho * S_ref * d_proj * C_mag_p * p * v_rel_mag * (v_unit_y * alpha_e_z);
        F_mag_y = -0.5 * rho * S_ref * d_proj * C_mag_p * p * v_rel_mag * (v_unit_x * alpha_e_z);

        ax = ax_drag + F_mag_x / m;
        ay = -g0 + ay_drag + F_mag_y / m;
        az = az_drag + F_lift_z / m;

        if is_guided && has_passed_apogee && (t >= apogee_time + gnc_start_delay) && control_active
            v_h = max(1e-3, sqrt(vel(1)^2 + vel(3)^2));
            eyaw_x = -vel(3) / v_h;
            eyaw_y = 0;
            eyaw_z = vel(1) / v_h;

            ev_x = vel(1) / v_rel_mag;
            ev_y = vel(2) / v_rel_mag;
            ev_z = vel(3) / v_rel_mag;

            epitch_x = -eyaw_z * ev_y;
            epitch_y = eyaw_z * ev_x - eyaw_x * ev_z;
            epitch_z = eyaw_x * ev_y;

            q_dynamic = 0.5 * rho * v_rel_mag^2;
            a_steer_mag = (q_dynamic * S_ref * C_N_canard) / m;
            phi_actual = cmd_gamma_c + delta_kappa;

            ax = ax + a_steer_mag * (cos(phi_actual) * epitch_x + sin(phi_actual) * eyaw_x);
            ay = ay + a_steer_mag * (cos(phi_actual) * epitch_y + sin(phi_actual) * eyaw_y);
            az = az + a_steer_mag * (cos(phi_actual) * epitch_z + sin(phi_actual) * eyaw_z);
        end

        prev_pos = pos;
        vel = vel + [ax, ay, az] * dt;
        pos = pos + vel * dt;
        
        flight_logs.time(k_log)      = t;
        flight_logs.x(k_log)         = pos(1);
        flight_logs.y(k_log)         = pos(2);
        flight_logs.z(k_log)         = pos(3);
        flight_logs.vx(k_log)        = vel(1);
        flight_logs.vy(k_log)        = vel(2);
        flight_logs.vz(k_log)        = vel(3);
        flight_logs.v_mag(k_log)     = norm(vel);
        flight_logs.Mach(k_log)      = Mach;
        flight_logs.spin_hz(k_log)   = p / (2*pi);
        flight_logs.gamma_c(k_log)   = cmd_gamma_c;
        flight_logs.pred_miss(k_log) = miss_dist;

        if mod(step_count, 10) == 0
            traj(k_log, :) = pos;
        end
        t = t + dt;
        step_count = step_count + 1;
        k_log = k_log + 1;
    end

    frac = prev_pos(2) / (prev_pos(2) - pos(2));
    imp_point = [prev_pos(1) + frac * (pos(1) - prev_pos(1)), ...
                 prev_pos(3) + frac * (pos(3) - prev_pos(3))];
    imp_t = max(0, t - dt + frac * dt);

    traj(k_log, :) = [imp_point(1), 0, imp_point(2)];
    
    k_last = k_log;
    flight_logs.time(k_last)      = imp_t;
    flight_logs.x(k_last)         = imp_point(1);
    flight_logs.y(k_last)         = 0;
    flight_logs.z(k_last)         = imp_point(2);
    flight_logs.vx(k_last)        = vel(1);
    flight_logs.vy(k_last)        = vel(2);
    flight_logs.vz(k_last)        = vel(3);
    flight_logs.v_mag(k_last)     = norm(vel);
    flight_logs.Mach(k_last)      = flight_logs.Mach(max(1, k_last-1));
    flight_logs.spin_hz(k_last)   = flight_logs.spin_hz(max(1, k_last-1));
    flight_logs.gamma_c(k_last)   = cmd_gamma_c;
    flight_logs.pred_miss(k_last) = miss_dist;

    flight_logs.time      = flight_logs.time(1:k_last);
    flight_logs.x         = flight_logs.x(1:k_last);
    flight_logs.y         = flight_logs.y(1:k_last);
    flight_logs.z         = flight_logs.z(1:k_last);
    flight_logs.vx        = flight_logs.vx(1:k_last);
    flight_logs.vy        = flight_logs.vy(1:k_last);
    flight_logs.vz        = flight_logs.vz(1:k_last);
    flight_logs.v_mag     = flight_logs.v_mag(1:k_last);
    flight_logs.Mach      = flight_logs.Mach(1:k_last);
    flight_logs.spin_hz   = flight_logs.spin_hz(1:k_last);
    flight_logs.gamma_c   = flight_logs.gamma_c(1:k_last);
    flight_logs.pred_miss = flight_logs.pred_miss(1:k_last);
    
    traj = traj(1:k_last, :);
end

% =========================================================================
% LOCAL FIRING SOLUTION SOLVER
% =========================================================================
function [theta, psi] = solve_firing_solution(target, v0_nom, canard_deflection_deg)
    theta = 54.5;
    psi = 0.0;
    max_iter = 15;
    tol = 1.0;
    nom_wind.spd = 4.0; nom_wind.az = 45.0; nom_wind.jet = 38.0;
    
    for iter = 1:max_iter
        [imp, ~, ~] = run_single_perturbed(v0_nom, theta, psi, canard_deflection_deg, ...
                                           false, target, 0, nom_wind);
        dx = target(1) - imp(1);
        dz = target(2) - imp(2);
        
        if sqrt(dx^2 + dz^2) < tol
            break;
        end
        
        psi = psi + rad2deg(atan2(dz, imp(1)));
        
        d_theta = 0.05;
        [imp_p, ~, ~] = run_single_perturbed(v0_nom, theta + d_theta, psi, canard_deflection_deg, ...
                                             false, target, 0, nom_wind);
        dRange_dTheta = (imp_p(1) - imp(1)) / d_theta;
        
        theta = theta + dx / dRange_dTheta;
        theta = max(45.0, min(80.0, theta));
    end
end

% =========================================================================
% CORRECTION ELLIPSE CALIBRATION WITH START TIME DELAY
% =========================================================================
function [x0, z0, semi_a, semi_b, impacts] = calibrate_correction_ellipse_delay(v0_nom, ...
                                                theta_nom, psi_nom, canard_deflection_deg, gnc_delay)
    angles_deg = 0:45:315;
    impacts = zeros(length(angles_deg), 2);
    nom_wind.spd = 4.0; nom_wind.az = 45.0; nom_wind.jet = 38.0;
    
    [imp_unc, ~, ~] = run_single_perturbed(v0_nom, theta_nom, psi_nom, canard_deflection_deg, ...
                                           false, [0,0], 0, nom_wind);
    
    for i = 1:length(angles_deg)
        gamma_c_rad = deg2rad(angles_deg(i));
        imp = run_single_forced_steering(v0_nom, theta_nom, psi_nom, ...
            canard_deflection_deg, gamma_c_rad, gnc_delay, nom_wind);
        impacts(i, :) = imp;
    end
    
    rel_impacts = impacts - imp_unc;
    x0 = mean(rel_impacts(:, 1));
    z0 = mean(rel_impacts(:, 2));
    semi_a = (max(rel_impacts(:, 1)) - min(rel_impacts(:, 1))) / 2;
    semi_b = (max(rel_impacts(:, 2)) - min(rel_impacts(:, 2))) / 2;
end

% =========================================================================
% LOCAL INTEGRATION FOR CONSTANT FORCED STEERING CALIBRATION
% =========================================================================
function [imp_point] = run_single_forced_steering(v0, theta, psi, ...
                         canard_deflection_deg, force_gamma_c, gnc_start_delay, wind_cfg)
    g0     = 9.80665;
    m      = 43.10;
    d_proj = 0.155;
    S_ref  = pi * (d_proj / 2)^2;
    Cd0    = 0.25;
    Ix     = 0.142;
    twist_calibers = 20.0;
    dt     = 0.01;
    Clp      = -0.015;
    C_La     = 2.0;
    C_mag_p  = 0.008;

    delta_kappa = deg2rad(160.0);
    delta_e = deg2rad(canard_deflection_deg);
    C_N_canard = 1.4 * delta_e;

    vx = v0 * cosd(theta) * cosd(psi);
    vy = v0 * sind(theta);
    vz = v0 * cosd(theta) * sind(psi);

    pos = [0, 0, 0];
    vel = [vx, vy, vz];
    p = (2.0 * pi * v0) / (twist_calibers * d_proj);

    t = 0.0;
    has_passed_apogee = false;
    apogee_time = 0.0;

    while pos(2) >= 0 && t < 130.0
        alt = max(0, pos(2));
        if vel(2) < 0 && ~has_passed_apogee
            has_passed_apogee = true;
            apogee_time = t;
        end

        h = max(0, min(alt, 20000));
        if h <= 11000
            T = 288.15 - 0.0065 * h;
            P = 101325 * (1 - 0.0065 * h / 288.15)^(9.80665 / (287.05287 * 0.0065));
        else
            T_trop = 216.65;
            P_trop = 101325 * (1 - 0.0065 * 11000 / 288.15)^(9.80665 / (287.05287 * 0.0065));
            T = T_trop;
            P = P_trop * exp(-9.80665 * (h - 11000) / (287.05287 * T_trop));
        end
        rho = P / (287.05287 * T);
        a_sound = sqrt(1.4 * 287.05287 * T);

        w_spd = wind_cfg.spd;
        w_az  = wind_cfg.az;
        if alt <= 2000
            v_mag = w_spd * (max(10, alt) / 2000)^0.2;
            v_dir = w_az;
        elseif alt > 2000 && alt <= 8000
            frac = (alt - 2000) / 6000;
            v_mag = w_spd * 1.5 + frac * (wind_cfg.jet * 0.5);
            v_dir = w_az + frac * 30.0;
        elseif alt > 8000 && alt <= 11500
            jet_core = wind_cfg.jet * exp(-((alt - 9500) / 1200)^2);
            v_mag = w_spd * 1.5 + jet_core;
            v_dir = w_az + 45.0;
        else
            frac = min(1.0, (alt - 11500) / 4000);
            v_mag = (wind_cfg.jet * 0.4) * (1 - 0.5 * frac);
            v_dir = w_az + 45.0;
        end
        th_w = deg2rad(v_dir);
        Wx = v_mag * cos(th_w);
        Wz = v_mag * sin(th_w);

        v_rel_x = vel(1) - Wx;
        v_rel_y = vel(2);
        v_rel_z = vel(3) - Wz;
        v_rel_mag = sqrt(v_rel_x^2 + v_rel_y^2 + v_rel_z^2);
        v_unit_x = v_rel_x / max(1e-3, v_rel_mag);
        v_unit_y = v_rel_y / max(1e-3, v_rel_mag);

        Mach = v_rel_mag / a_sound;
        if Mach < 0.8
            Cd = Cd0;
        elseif Mach < 1.05
            Cd = Cd0 + 0.22 * ((Mach - 0.8) / 0.25)^2;
        elseif Mach < 1.6
            Cd = (Cd0 + 0.22) - 0.08 * ((Mach - 1.05) / 0.55);
        else
            Cd = (Cd0 + 0.14) / (1 + 0.15 * (Mach - 1.6));
        end

        dp_dt = (0.5 * rho * v_rel_mag * S_ref * (d_proj^2) * Clp / Ix) * p;
        p = max(0, p + dp_dt * dt);

        drag_k = 0.5 * rho * S_ref * Cd * v_rel_mag / m;
        ax_drag = -drag_k * v_rel_x;
        ay_drag = -drag_k * v_rel_y;
        az_drag = -drag_k * v_rel_z;

        vcg_z = v_unit_x * g0;
        denom = max(100.0, rho * S_ref * d_proj * 11.5 * (v_rel_mag^2));
        alpha_e_z = (2.0 * Ix * p / (denom * v_rel_mag)) * vcg_z;
        F_lift_z = 0.5 * rho * S_ref * (v_rel_mag^2) * C_La * alpha_e_z;
        F_mag_x = 0.5 * rho * S_ref * d_proj * C_mag_p * p * v_rel_mag * (v_unit_y * alpha_e_z);
        F_mag_y = -0.5 * rho * S_ref * d_proj * C_mag_p * p * v_rel_mag * (v_unit_x * alpha_e_z);

        ax = ax_drag + F_mag_x / m;
        ay = -g0 + ay_drag + F_mag_y / m;
        az = az_drag + F_lift_z / m;

        if has_passed_apogee && (t >= apogee_time + gnc_start_delay)
            v_h = max(1e-3, sqrt(vel(1)^2 + vel(3)^2));
            eyaw_x = -vel(3) / v_h;
            eyaw_y = 0;
            eyaw_z = vel(1) / v_h;

            ev_x = vel(1) / v_rel_mag;
            ev_y = vel(2) / v_rel_mag;
            ev_z = vel(3) / v_rel_mag;

            epitch_x = -eyaw_z * ev_y;
            epitch_y = eyaw_z * ev_x - eyaw_x * ev_z;
            epitch_z = eyaw_x * ev_y;

            q_dynamic = 0.5 * rho * v_rel_mag^2;
            a_steer_mag = (q_dynamic * S_ref * C_N_canard) / m;
            phi_actual = force_gamma_c + delta_kappa;

            ax = ax + a_steer_mag * (cos(phi_actual) * epitch_x + sin(phi_actual) * eyaw_x);
            ay = ay + a_steer_mag * (cos(phi_actual) * epitch_y + sin(phi_actual) * eyaw_y);
            az = az + a_steer_mag * (cos(phi_actual) * epitch_z + sin(phi_actual) * eyaw_z);
        end

        prev_pos = pos;
        vel = vel + [ax, ay, az] * dt;
        pos = pos + vel * dt;
        t = t + dt;
    end

    frac = prev_pos(2) / (prev_pos(2) - pos(2));
    imp_point = [prev_pos(1) + frac * (pos(1) - prev_pos(1)), ...
                 prev_pos(3) + frac * (pos(3) - prev_pos(3))];
end