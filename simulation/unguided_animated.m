function unguided_animated(anim_duration_sec, save_gif)
    % =========================================================================
    % 155 mm CONVENTIONAL ARTILLERY - UNGUIDED M107 15-SECOND ANIMATION
    % =========================================================================
    % Simulates and produces an exact 15-second animated ballistic flight of
    % the standard 155 mm M107 High-Explosive projectile:
    %   - Physical Model: STANAG 4355 Modified Point Mass (MPM)
    %   - Spin Dynamics: 1:20 twist barrel, viscous roll decay, yaw of repose drift
    %   - Environmental: ISA Atmosphere + Jet Stream Wind Profile
    %   - Visual: Real-time 3D flight trail, ground shadow, vector pointer,
    %             real-time HUD telemetry, side altitude profile, ground drift,
    %             Mach number & spin rate plots, and terminal detonation shockwave.
    %   - Output: Exports both a 15-second GIF and a 15-second MP4 video.
    %
    % Usage:
    %   unguided_animated()          % Exactly 15.0-second animation (default)
    %   unguided_animated(15.0)      % Explicit 15.0-second duration
    %   unguided_animated(10.0)      % 10.0-second duration
    % =========================================================================

    if nargin < 1 || isempty(anim_duration_sec), anim_duration_sec = 15.0; end
    if nargin < 2 || isempty(save_gif), save_gif = true; end

    % Nominal Gun Firing Solution (M109/ATAGS 155mm, Top Zone Propelling Charge)
    v0_nominal    = 827.0;             % M107 nominal muzzle velocity at Charge 8 (m/s)
    theta_nominal = 54.5;              % Quadrant elevation angle (degrees) for ~18 km range
    psi_nominal   = 0.0;               % Gun azimuth laying angle (degrees)
    target        = [18000, 400];      % Designated Target [Downrange, Crossrange] (m)

    % Environmental Wind Configuration
    env_cfg.ground_wind_speed   = 4.0;  % Ground wind speed (m/s)
    env_cfg.ground_wind_azimuth = 45.0; % Wind direction azimuth (deg)
    env_cfg.jet_stream_speed    = 38.0; % Peak jet stream at 9.5 km (m/s)

    fprintf('===============================================================\n');
    fprintf('  155 mm M107 UNGUIDED ARTILLERY - %.1f-SECOND FLIGHT ANIMATION\n', anim_duration_sec);
    fprintf('===============================================================\n');
    fprintf('Integrating 6-DoF Modified Point Mass trajectory...\n');

    % Run high-fidelity physics integration
    flight_data = compute_flight_telemetry(v0_nominal, theta_nominal, psi_nominal, env_cfg);
    
    N_steps = length(flight_data.time);
    t_final = flight_data.time(end);
    imp_x   = flight_data.x(end);
    imp_y   = flight_data.y(end);
    imp_z   = flight_data.z(end);
    miss_d  = sqrt((imp_x - target(1))^2 + (imp_z - target(2))^2);
    
    fprintf('Real Flight Time   : %.2f seconds (Mach %.2f -> Mach %.2f)\n', ...
            t_final, flight_data.Mach(1), flight_data.Mach(end));
    fprintf('Impact Location    : Downrange = %.1f m | Lateral Drift = %.1f m\n', imp_x, imp_z);
    fprintf('Target Location    : [%.0f, %.0f] m (30m PGK accuracy requirement)\n', target(1), target(2));
    fprintf('Total Miss Distance: %.2f m (Bias dx = %+.1fm, dz = %+.1fm)\n', ...
            miss_d, imp_x - target(1), imp_z - target(2));

    % Target Frame Rate & Exact 15-Second Timing Calculation
    fps = 15; % 15 frames per second for smooth, crisp playback
    total_frames = round(anim_duration_sec * fps);
    gif_delay = anim_duration_sec / total_frames; % exact delay per frame (e.g. 1/15 = 0.0667s)

    % Allocate frames: ~83% flight phase + ~17% terminal impact aftermath
    flight_frames = round(total_frames * 0.83);
    impact_frames = total_frames - flight_frames;

    frame_stride  = max(1, floor(N_steps / flight_frames));
    frame_indices = round(linspace(1, N_steps, flight_frames));

    fprintf('Animation Timing   : %d Total Frames at %d FPS = %.2f seconds total duration\n', ...
            total_frames, fps, total_frames * gif_delay);
    fprintf('  - Flight Phase   : %d frames (%.2f s, %0.1fx real-time speedup)\n', ...
            flight_frames, flight_frames * gif_delay, t_final / (flight_frames * gif_delay));
    fprintf('  - Impact Aftermath: %d frames (%.2f s)\n', ...
            impact_frames, impact_frames * gif_delay);

    % Setup Unified High-Resolution Figure Window
    is_headless = isempty(getenv('DISPLAY'));
    vis_mode = 'on';
    if is_headless, vis_mode = 'off'; end

    fig = figure('Visible', vis_mode, 'Color', [0.08 0.10 0.14], ...
                 'Position', [20, 20, 1560, 860], ...
                 'Name', sprintf('155 mm M107 Artillery - %.0fs Ballistic Flight Animation', anim_duration_sec));

    % Precompute Target Rings
    circ_ang = linspace(0, 2*pi, 120);
    c_x30  = target(1) + 30 * cos(circ_ang);
    c_z30  = target(2) + 30 * sin(circ_ang);
    c_x100 = target(1) + 100 * cos(circ_ang);
    c_z100 = target(2) + 100 * sin(circ_ang);

    max_x = max(flight_data.x) * 1.05;
    max_y = max(flight_data.y) * 1.15;
    min_z = min([0, flight_data.z(:)', target(2)]) - 120;
    max_z = max([0, flight_data.z(:)', target(2)]) + 180;

    % =========================================================================
    % SUBPLOT 1 (Left 2 Columns): 3D Flight Profile & Live HUD
    % =========================================================================
    ax3d = subplot(2, 3, [1, 4]);
    set(ax3d, 'Color', [0.12 0.14 0.19], 'XColor', [0.7 0.75 0.85], ...
              'YColor', [0.7 0.75 0.85], 'ZColor', [0.7 0.75 0.85], ...
              'GridColor', [0.25 0.30 0.38], 'GridAlpha', 0.6);
    hold(ax3d, 'on'); grid(ax3d, 'on'); box(ax3d, 'on');

    % Battery Origin [0,0,0]
    plot3(ax3d, 0, 0, 0, '^', 'MarkerSize', 11, 'MarkerFaceColor', [0.95 0.75 0.2], ...
          'MarkerEdgeColor', 'w', 'DisplayName', 'Battery Origin [0,0,0]');

    % Target Ground Markers
    plot3(ax3d, target(1), target(2), 0, 'rx', 'MarkerSize', 14, 'LineWidth', 2.5, ...
          'DisplayName', 'Target [18000, 400]');
    plot3(ax3d, c_x30, c_z30, zeros(size(c_x30)), 'r--', 'LineWidth', 1.8, ...
          'DisplayName', '30m PGK Spec Circle');
    plot3(ax3d, c_x100, c_z100, zeros(size(c_x100)), ':', 'Color', [0.8 0.4 0.4], ...
          'LineWidth', 1.0, 'DisplayName', '100m Zone');

    % Animated Graphic Handles for 3D View
    h_traj3d   = plot3(ax3d, 0, 0, 0, 'Color', [0.20 0.80 1.00], 'LineWidth', 2.2);
    h_shadow3d = plot3(ax3d, 0, 0, 0, 'Color', [0.35 0.40 0.50], 'LineWidth', 1.2, 'LineStyle', ':');
    h_shell3d  = plot3(ax3d, 0, 0, 0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', [1.0 0.30 0.20], ...
                       'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
    h_vect3d   = quiver3(ax3d, 0, 0, 0, 0, 0, 0, 0, 'Color', [1.0 0.85 0.3], ...
                         'LineWidth', 2.0, 'MaxHeadSize', 2.0);

    % Expanding blast wave handle for impact
    h_blast3d  = plot3(ax3d, 0, 0, 0, 'Color', [1.0 0.5 0.1], 'LineWidth', 2.5, 'Visible', 'off');

    xlabel(ax3d, 'Downrange X (m)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    ylabel(ax3d, 'Crossrange Z (m)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    zlabel(ax3d, 'Altitude Y (m)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    title(ax3d, '3D Ballistic Trajectory - 155 mm M107 HE (Modified Point Mass)', ...
          'FontSize', 11, 'FontWeight', 'bold', 'Color', [1 1 1]);
    xlim(ax3d, [-500, max_x]);
    ylim(ax3d, [min_z, max_z]);
    zlim(ax3d, [0, max_y]);
    view(ax3d, [-38, 22]);

    % Telemetry HUD Overlay in Subplot 1
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

    % Atmospheric Layer Bands
    patch(ax2d_side, [0, max_x, max_x, 0], [0, 0, 2000, 2000], [0.15 0.20 0.28], ...
          'EdgeColor', 'none', 'FaceAlpha', 0.4);
    text(ax2d_side, 500, 1000, 'Boundary Layer', 'Color', [0.5 0.6 0.7], 'FontSize', 7);
    patch(ax2d_side, [0, max_x, max_x, 0], [8000, 8000, 11500, 11500], [0.25 0.18 0.28], ...
          'EdgeColor', 'none', 'FaceAlpha', 0.4);
    text(ax2d_side, 500, 9500, 'Jet Stream Core', 'Color', [0.7 0.5 0.7], 'FontSize', 7);

    h_side_traj  = plot(ax2d_side, 0, 0, 'Color', [0.20 0.80 1.00], 'LineWidth', 2.0);
    h_side_shell = plot(ax2d_side, 0, 0, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [1.0 0.3 0.2], ...
                        'MarkerEdgeColor', 'w');

    xlabel(ax2d_side, 'Downrange X (m)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    ylabel(ax2d_side, 'Altitude Y (m)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    title(ax2d_side, 'Vertical Flight Profile (Altitude vs Downrange)', ...
          'FontSize', 10, 'FontWeight', 'bold', 'Color', [1 1 1]);
    xlim(ax2d_side, [0, max_x]);
    ylim(ax2d_side, [0, max_y]);

    % =========================================================================
    % SUBPLOT 3 (Top Right): Mach Number Profile
    % =========================================================================
    ax_mach = subplot(2, 3, 3);
    set(ax_mach, 'Color', [0.12 0.14 0.19], 'XColor', [0.7 0.75 0.85], ...
                 'YColor', [0.7 0.75 0.85], 'GridColor', [0.25 0.30 0.38], 'GridAlpha', 0.5);
    hold(ax_mach, 'on'); grid(ax_mach, 'on'); box(ax_mach, 'on');

    t_full = flight_data.time;
    plot(ax_mach, t_full, flight_data.Mach, 'Color', [0.3 0.45 0.7], 'LineStyle', ':', 'LineWidth', 1.0);
    h_dyn_mach = plot(ax_mach, 0, 0, 'Color', [0.3 0.85 1.0], 'LineWidth', 2.2);
    h_mach_dot = plot(ax_mach, 0, 0, 'o', 'MarkerSize', 6, 'MarkerFaceColor', [0.3 0.85 1.0], ...
                      'MarkerEdgeColor', 'w');

    xlabel(ax_mach, 'Flight Time (s)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    ylabel(ax_mach, 'Mach Number (M)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    title(ax_mach, 'Mach Number (Transonic / Supersonic)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [1 1 1]);
    xlim(ax_mach, [0, t_final * 1.05]);
    ylim(ax_mach, [0, 2.7]);

    % =========================================================================
    % SUBPLOT 4 (Bottom Middle): Ground Track (Crossrange vs Downrange)
    % =========================================================================
    ax2d_top = subplot(2, 3, 5);
    set(ax2d_top, 'Color', [0.12 0.14 0.19], 'XColor', [0.7 0.75 0.85], ...
                  'YColor', [0.7 0.75 0.85], 'GridColor', [0.25 0.30 0.38], 'GridAlpha', 0.6);
    hold(ax2d_top, 'on'); grid(ax2d_top, 'on'); box(ax2d_top, 'on');

    plot(ax2d_top, target(1), target(2), 'rx', 'MarkerSize', 13, 'LineWidth', 2.2);
    plot(ax2d_top, c_x30, c_z30, 'r--', 'LineWidth', 1.5);
    text(ax2d_top, target(1)+80, target(2), 'TARGET', 'Color', [1.0 0.4 0.4], ...
         'FontSize', 8, 'FontWeight', 'bold');

    h_top_traj  = plot(ax2d_top, 0, 0, 'Color', [0.20 0.80 1.00], 'LineWidth', 2.0);
    h_top_shell = plot(ax2d_top, 0, 0, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [1.0 0.3 0.2], ...
                       'MarkerEdgeColor', 'w');

    xlabel(ax2d_top, 'Downrange X (m)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    ylabel(ax2d_top, 'Crossrange Drift Z (m)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    title(ax2d_top, 'Ground Track (Spin Drift + Crosswind)', ...
          'FontSize', 10, 'FontWeight', 'bold', 'Color', [1 1 1]);
    xlim(ax2d_top, [0, max_x]);
    ylim(ax2d_top, [min_z, max_z]);

    % =========================================================================
    % SUBPLOT 5 (Bottom Right): Spin Decay & Yaw of Repose
    % =========================================================================
    ax_spin = subplot(2, 3, 6);
    set(ax_spin, 'Color', [0.12 0.14 0.19], 'XColor', [0.7 0.75 0.85], ...
                 'YColor', [0.7 0.75 0.85], 'GridColor', [0.25 0.30 0.38], 'GridAlpha', 0.5);
    hold(ax_spin, 'on'); grid(ax_spin, 'on'); box(ax_spin, 'on');

    plot(ax_spin, t_full, flight_data.spin_hz, 'Color', [0.7 0.45 0.3], 'LineStyle', ':', 'LineWidth', 1.0);
    h_dyn_spin = plot(ax_spin, 0, 0, 'Color', [1.0 0.6 0.2], 'LineWidth', 2.2);
    h_spin_dot = plot(ax_spin, 0, 0, 'o', 'MarkerSize', 6, 'MarkerFaceColor', [1.0 0.6 0.2], ...
                      'MarkerEdgeColor', 'w');

    xlabel(ax_spin, 'Flight Time (s)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    ylabel(ax_spin, 'Spin Rate (Hz)', 'FontWeight', 'bold', 'Color', [0.85 0.9 1.0]);
    title(ax_spin, 'Spin Roll Decay (Viscous Damping)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [1 1 1]);
    xlim(ax_spin, [0, t_final * 1.05]);
    ylim(ax_spin, [120, 290]);

    % Preallocate frame storage for lightning-fast one-shot batch GIF generation
    recorded_frames = cell(total_frames, 1);
    color_map = [];

    fprintf('Rendering 15-second animation frames...\n');

    % =========================================================================
    % PHASE 1: ACTIVE FLIGHT TRAJECTORY (Frames 1 to flight_frames)
    % =========================================================================
    for f = 1:flight_frames
        idx = frame_indices(f);

        cur_t   = flight_data.time(idx);
        cur_x   = flight_data.x(idx);
        cur_y   = flight_data.y(idx);
        cur_z   = flight_data.z(idx);
        cur_v   = flight_data.v_mag(idx);
        cur_m   = flight_data.Mach(idx);
        cur_p   = flight_data.spin_hz(idx);
        cur_ae  = flight_data.alpha_e_mils(idx);
        cur_rho = flight_data.rho(idx);
        cur_q   = 0.5 * cur_rho * (cur_v^2) / 1000.0;

        sub_x = flight_data.x(1:idx);
        sub_y = flight_data.y(1:idx);
        sub_z = flight_data.z(1:idx);

        % Update 3D Visuals
        set(h_traj3d,   'XData', sub_x, 'YData', sub_z, 'ZData', sub_y);
        set(h_shadow3d, 'XData', sub_x, 'YData', sub_z, 'ZData', zeros(size(sub_x)));
        set(h_shell3d,  'XData', cur_x, 'YData', cur_z, 'ZData', cur_y);

        v_scale = 1200.0;
        set(h_vect3d, 'XData', cur_x, 'YData', cur_z, 'ZData', cur_y, ...
                      'UData', (flight_data.vx(idx) / cur_v) * v_scale, ...
                      'VData', (flight_data.vz(idx) / cur_v) * v_scale, ...
                      'WData', (flight_data.vy(idx) / cur_v) * v_scale);

        % HUD Text
        anim_time_elapsed = (f - 1) * gif_delay;
        hud_str = sprintf([ ...
            '-- 155mm M107 TELEMETRY --\n' ...
            'Sim Flight Time : %6.2f s\n' ...
            'Downrange X     : %7.1f m\n' ...
            'Crossrange Z    : %7.1f m\n' ...
            'Altitude Y      : %7.1f m\n' ...
            'Velocity        : %6.1f m/s\n' ...
            'Mach Number     : %6.2f M\n' ...
            'Spin Rate       : %6.1f Hz (%d RPM)\n' ...
            'Yaw of Repose   : %6.2f mils\n' ...
            'Dyn Pressure    : %6.1f kPa\n' ...
            'Target Vector   : [%+.0fm, %+.0fm]'], ...
            cur_t, anim_time_elapsed, anim_duration_sec, ...
            cur_x, cur_z, cur_y, cur_v, cur_m, cur_p, round(cur_p * 60), ...
            cur_ae, cur_q, cur_x - target(1), cur_z - target(2));
        set(hud_text, 'String', hud_str);

        % Update 2D Subplots
        set(h_side_traj,  'XData', sub_x, 'YData', sub_y);
        set(h_side_shell, 'XData', cur_x, 'YData', cur_y);

        set(h_top_traj,  'XData', sub_x, 'YData', sub_z);
        set(h_top_shell, 'XData', cur_x, 'YData', cur_z);

        sub_t = flight_data.time(1:idx);
        set(h_dyn_mach, 'XData', sub_t, 'YData', flight_data.Mach(1:idx));
        set(h_mach_dot, 'XData', cur_t, 'YData', cur_m);
        set(h_dyn_spin, 'XData', sub_t, 'YData', flight_data.spin_hz(1:idx));
        set(h_spin_dot, 'XData', cur_t, 'YData', cur_p);

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
                % Graceful skip
            end
        end
    end

    % =========================================================================
    % PHASE 2: TERMINAL IMPACT & DETONATION AFTERMATH (Last ~2.5 seconds)
    % =========================================================================
    % Detonation impact marker in 3D
    plot3(ax3d, imp_x, imp_z, 0, 'p', 'MarkerSize', 18, 'MarkerFaceColor', [1.0 0.85 0.1], ...
          'MarkerEdgeColor', [1.0 0.1 0.0], 'LineWidth', 2.2);
    text(ax3d, imp_x + 120, imp_z + 80, 50, sprintf('IMPACT [%.0fm, %.0fm]', imp_x, imp_z), ...
         'FontSize', 10, 'FontWeight', 'bold', 'Color', [1.0 0.65 0.2]);

    % Red line connecting Target to Impact point (Miss Vector)
    plot3(ax3d, [target(1), imp_x], [target(2), imp_z], [0, 0], 'Color', [1.0 0.2 0.2], ...
          'LineWidth', 2.0, 'LineStyle', '--');

    plot(ax2d_top, [target(1), imp_x], [target(2), imp_z], 'Color', [1.0 0.2 0.2], ...
         'LineWidth', 2.0, 'LineStyle', '--');
    plot(ax2d_top, imp_x, imp_z, 'p', 'MarkerSize', 14, 'MarkerFaceColor', [1.0 0.8 0.1], ...
         'MarkerEdgeColor', [1.0 0.1 0.0], 'LineWidth', 2.0);

    set(h_blast3d, 'Visible', 'on');

    for imp_i = 1:impact_frames
        f = flight_frames + imp_i;
        anim_time_elapsed = (f - 1) * gif_delay;

        % Expanding shockwave radius at impact
        blast_radius = 20.0 + (imp_i / impact_frames) * 350.0;
        b_x = imp_x + blast_radius * cos(circ_ang);
        b_z = imp_z + blast_radius * sin(circ_ang);
        set(h_blast3d, 'XData', b_x, 'YData', b_z, 'ZData', zeros(size(b_x)), ...
                       'Color', [1.0, max(0.2, 0.9 - imp_i/impact_frames), 0.1]);

        % Telemetry HUD showing final impact statistics
        hud_str = sprintf([ ...
            '== IMPACT RESULT ==\n' ...
            'Flight Duration : %6.2f s\n' ...
            'Impact Point    : [%.0fm, %.0fm]\n' ...
            'Target Point    : [%.0fm, %.0fm]\n' ...
            'Range Bias (dx) : %+6.1f m\n' ...
            'Drift Bias (dz) : %+6.1f m\n' ...
            'MISS DISTANCE   : %6.2f METERS\n' ...
            'PGK Spec Limit  : <= 30.0 METERS\n' ...
            'RESULT STATUS   : FAILED (>30m)'], ...
            t_final, anim_time_elapsed, anim_duration_sec, ...
            imp_x, imp_z, target(1), target(2), ...
            imp_x - target(1), imp_z - target(2), miss_d);
        set(hud_text, 'String', hud_str, 'Color', [1.0 0.35 0.3], ...
                      'EdgeColor', [0.8 0.2 0.2]);

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
                % Graceful skip
            end
        end
    end

    % Final High-Res Summary Screenshot
    print(fig, 'unguided_animated_analysis.png', '-dpng', '-r150');
    fprintf('Final dashboard saved to unguided_animated_analysis.png\n');

    % =========================================================================
    % EXPORT ANIMATED GIF (Batch Write) & MP4 VIDEO
    % =========================================================================
    gif_filename = 'unguided_trajectory_15s.gif';
    mp4_filename = 'unguided_trajectory_15s.mp4';

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

        % Convert GIF to high-compatibility MP4 video using ffmpeg
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

    fprintf('===============================================================\n');
    fprintf('  15-SECOND ANIMATION COMPLETE\n');
    fprintf('===============================================================\n');
end

% =========================================================================
% HIGH-FIDELITY FLIGHT TELEMETRY COMPUTATION (STANAG 4355 MPM + SPIN)
% =========================================================================
function data = compute_flight_telemetry(v0, theta, psi, env_cfg)
    g0     = 9.80665;
    m      = 43.10;                % Shell mass (kg)
    d_proj = 0.155;                % Caliber diameter (m)
    S_ref  = pi * (d_proj / 2)^2;  % Reference area (m^2)
    Cd0    = 0.25;                 % Subsonic zero-lift drag
    Ix     = 0.142;                % Axial moment of inertia (kg*m^2)
    Iy     = 1.62;                 % Transverse moment of inertia (kg*m^2)

    twist_calibers = 20.0;
    p = (2.0 * pi * v0) / (twist_calibers * d_proj); % Initial spin (rad/s) (~267 Hz)

    Clp      = -0.015;             % Viscous roll damping coefficient
    C_La     = 2.0;                % Lift slope derivative (per rad)
    C_mag_p  = 0.008;              % Magnus derivative

    % Initial velocity vector
    vx = v0 * cosd(theta) * cosd(psi);
    vy = v0 * sind(theta);
    vz = v0 * cosd(theta) * sind(psi);

    state = [0; 0; 0; vx; vy; vz];
    dt = 0.01;
    t = 0.0;

    max_steps = 14000;
    log_t       = zeros(max_steps, 1);
    log_pos     = zeros(max_steps, 3);
    log_vel     = zeros(max_steps, 3);
    log_mach    = zeros(max_steps, 1);
    log_spin_hz = zeros(max_steps, 1);
    log_alpha_e = zeros(max_steps, 1);
    log_rho     = zeros(max_steps, 1);

    k = 1;

    while state(2) >= 0 && state(1) < 50000 && t < 130.0
        x = state(1); y = state(2); z = state(3);
        vx = state(4); vy = state(5); vz = state(6);
        alt = max(0, y);

        % 1. ISA Atmosphere
        [~, ~, rho, a_sound] = isa_atmosphere(alt);

        % 2. Altitude-Dependent Wind Profile
        v_wind = wind_profile(alt, env_cfg);

        % 3. Relative Airflow Velocity
        v_rel_vec = [vx - v_wind(1); vy - v_wind(2); vz - v_wind(3)];
        v_rel_mag = norm(v_rel_vec);
        v_unit    = v_rel_vec / max(1e-3, v_rel_mag);

        % 4. Compressible Mach Drag
        Mach = v_rel_mag / a_sound;
        Cd   = cd_mach_model(Mach, Cd0);

        % 5. Viscous Roll Spin Decay
        dp_dt = (0.5 * rho * v_rel_mag * S_ref * (d_proj^2) * Clp / Ix) * p;
        p     = max(0, p + dp_dt * dt);

        % 6. Aerodynamic Retardation Drag Force
        F_drag = -0.5 * rho * S_ref * Cd * v_rel_mag * v_rel_vec;

        % 7. Equilibrium Yaw of Repose (STANAG 4355 Gyroscopic Drift)
        v_cross_g    = cross(v_unit, [0; g0; 0]);
        denom_repose = max(100.0, rho * S_ref * d_proj * 11.5 * (v_rel_mag^2));
        alpha_e      = (2.0 * Ix * p / (denom_repose * v_rel_mag)) * v_cross_g;

        % Aerodynamic Lift Force from Equilibrium Yaw of Repose
        F_lift_drift = 0.5 * rho * S_ref * (v_rel_mag^2) * C_La * alpha_e;

        % Dimensionally Rigorous Magnus Force
        F_mag = 0.5 * rho * S_ref * d_proj * C_mag_p * p * v_rel_mag * cross(v_unit, alpha_e);

        % 8. Total Acceleration (Gravity + Aerodynamics)
        a_tot = [0; -g0; 0] + (F_drag + F_lift_drift + F_mag) / m;

        % Log telemetry
        log_t(k)         = t;
        log_pos(k, :)    = [x, y, z];
        log_vel(k, :)    = [vx, vy, vz];
        log_mach(k)      = Mach;
        log_spin_hz(k)   = p / (2 * pi);
        log_alpha_e(k)   = norm(alpha_e) * 1000.0; % in mils
        log_rho(k)       = rho;

        % Symplectic Euler-Cromer Integration
        state(4:6) = state(4:6) + a_tot * dt;
        state(1:3) = state(1:3) + state(4:6) * dt;

        t = t + dt;
        k = k + 1;
    end

    % Linear ground-plane interpolation at impact
    k_last = k - 1;
    frac = log_pos(k_last, 2) / (log_pos(k_last, 2) - state(2));
    imp_x = log_pos(k_last, 1) + frac * (state(1) - log_pos(k_last, 1));
    imp_z = log_pos(k_last, 3) + frac * (state(3) - log_pos(k_last, 3));
    imp_t = log_t(k_last) + frac * dt;

    log_pos(k, :)    = [imp_x, 0, imp_z];
    log_vel(k, :)    = state(4:6)';
    log_t(k)         = imp_t;
    log_mach(k)      = log_mach(k_last);
    log_spin_hz(k)   = log_spin_hz(k_last);
    log_alpha_e(k)   = log_alpha_e(k_last);
    log_rho(k)       = log_rho(k_last);

    % Package outputs
    data.time         = log_t(1:k);
    data.x            = log_pos(1:k, 1);
    data.y            = log_pos(1:k, 2);
    data.z            = log_pos(1:k, 3);
    data.vx           = log_vel(1:k, 1);
    data.vy           = log_vel(1:k, 2);
    data.vz           = log_vel(1:k, 3);
    data.v_mag        = sqrt(data.vx.^2 + data.vy.^2 + data.vz.^2);
    data.Mach         = log_mach(1:k);
    data.spin_hz      = log_spin_hz(1:k);
    data.alpha_e_mils = log_alpha_e(1:k);
    data.rho          = log_rho(1:k);
end

% --- International Standard Atmosphere (ISA) ---
function [T, P, rho, a_sound] = isa_atmosphere(alt)
    R_air = 287.05287;
    gamma = 1.4;
    g0    = 9.80665;
    T0 = 288.15; P0 = 101325; L = 0.0065; h_tropopause = 11000.0;
    h = max(0, min(alt, 20000));
    if h <= h_tropopause
        T = T0 - L * h;
        P = P0 * (1 - L * h / T0)^(g0 / (R_air * L));
    else
        T_trop = T0 - L * h_tropopause;
        P_trop = P0 * (1 - L * h_tropopause / T0)^(g0 / (R_air * L));
        T = T_trop;
        P = P_trop * exp(-g0 * (h - h_tropopause) / (R_air * T_trop));
    end
    rho     = P / (R_air * T);
    a_sound = sqrt(gamma * R_air * T);
end

% --- Altitude-Dependent Jet Stream Wind Profile ---
function v_wind = wind_profile(alt, env_cfg)
    v_ground_mag = env_cfg.ground_wind_speed;
    wind_dir_deg = env_cfg.ground_wind_azimuth;
    if alt <= 2000
        v_mag = v_ground_mag * (max(10, alt) / 2000)^0.2;
        v_dir = wind_dir_deg;
    elseif alt > 2000 && alt <= 8000
        frac  = (alt - 2000) / (8000 - 2000);
        v_mag = v_ground_mag * 1.5 + frac * (env_cfg.jet_stream_speed * 0.5);
        v_dir = wind_dir_deg + frac * 30.0;
    elseif alt > 8000 && alt <= 11500
        jet_core = env_cfg.jet_stream_speed * exp(-((alt - 9500) / 1200)^2);
        v_mag    = (v_ground_mag * 1.5) + jet_core;
        v_dir    = wind_dir_deg + 45.0;
    else
        frac  = min(1.0, (alt - 11500) / 4000);
        v_mag = (env_cfg.jet_stream_speed * 0.4) * (1 - 0.5 * frac);
        v_dir = wind_dir_deg + 45.0;
    end
    theta_rad = deg2rad(v_dir);
    v_wind = [v_mag * cos(theta_rad); 0.0; v_mag * sin(theta_rad)];
end

% --- Compressibility / Mach Drag Scaling Model ---
function Cd = cd_mach_model(Mach, Cd0)
    if Mach < 0.8
        Cd = Cd0;
    elseif Mach >= 0.8 && Mach < 1.05
        Cd = Cd0 + 0.22 * ((Mach - 0.8) / 0.25)^2;
    elseif Mach >= 1.05 && Mach < 1.6
        Cd = (Cd0 + 0.22) - 0.08 * ((Mach - 1.05) / 0.55);
    else
        Cd = (Cd0 + 0.14) / (1 + 0.15 * (Mach - 1.6));
    end
end
