function guided_animated(anim_duration_sec, save_gif)
% =========================================================================
% PAPER-INSPIRED VIRTUAL-TARGET GUIDED TRAJECTORY VISUALIZATION
% =========================================================================
%
% This is a visualization/demo of a virtual-target correction concept.
%
% It shows:
%   - Reference / unguided trajectory
%   - Virtual target used for the correction concept
%   - Corrected demonstration trajectory
%   - Animated projectile motion
%   - Ground-track correction
%   - Altitude profile
%   - Position error reduction
%   - Correction magnitude
%   - Live telemetry HUD
%   - Terminal comparison between reference and corrected trajectories
%
% IMPORTANT:
%   This implementation intentionally does NOT contain:
%       - real projectile guidance laws
%       - actuator / fin / canard commands
%       - steering commands
%       - aerodynamic control derivatives
%       - real-world weapon guidance logic
%
% The correction trajectory is generated geometrically for visualization.
%
% Usage:
%   guided_animated()
%   guided_animated(15)
%   guided_animated(15, true)
%
% =========================================================================

    if nargin < 1 || isempty(anim_duration_sec)
        anim_duration_sec = 15.0;
    end

    if nargin < 2 || isempty(save_gif)
        save_gif = true;
    end

    % =========================================================================
    % DEMONSTRATION CONFIGURATION
    % =========================================================================

    % These values define a generic demonstration coordinate system.
    % They are NOT used to derive an operational guidance solution.

    reference_target = [18000, 400];       % [Downrange, Crossrange]
    virtual_target   = [17400, 300];       % Demonstration virtual target

    % Initial conditions for the reference trajectory.
    v0    = 827.0;
    theta = 54.5;
    psi   = 0.0;

    % Environment used only by the reference trajectory.
    env_cfg.ground_wind_speed   = 4.0;
    env_cfg.ground_wind_azimuth = 45.0;
    env_cfg.jet_stream_speed    = 38.0;

    fprintf('\n');
    fprintf('===============================================================\n');
    fprintf('     VIRTUAL-TARGET GUIDED TRAJECTORY VISUALIZATION\n');
    fprintf('===============================================================\n');
    fprintf('Generating reference trajectory...\n');

    % =========================================================================
    % REFERENCE TRAJECTORY
    % =========================================================================

    reference = compute_reference_trajectory( ...
        v0, theta, psi, env_cfg);

    N = length(reference.time);

    ref_x = reference.x;
    ref_y = reference.y;
    ref_z = reference.z;

    t_final = reference.time(end);

    % =========================================================================
    % DEMONSTRATION CORRECTION TRAJECTORY
    % =========================================================================
    %
    % This is intentionally geometric.
    %
    % The correction begins gradually after launch and becomes strongest
    % during the middle portion of the trajectory. It smoothly brings the
    % demonstration path toward the designated target region.
    %
    % No physical steering/control command is calculated.
    % =========================================================================

    corrected = generate_visual_correction( ...
        reference, reference_target, virtual_target);

    corr_x = corrected.x;
    corr_y = corrected.y;
    corr_z = corrected.z;

    % =========================================================================
    % FINAL ERROR CALCULATIONS
    % =========================================================================

    ref_impact = [ref_x(end), ref_z(end)];
    guided_impact = [corr_x(end), corr_z(end)];

    ref_error = norm(ref_impact - reference_target);
    guided_error = norm(guided_impact - reference_target);

    fprintf('\n');
    fprintf('Reference impact : X = %.2f m | Z = %.2f m\n', ...
        ref_impact(1), ref_impact(2));

    fprintf('Demonstration end: X = %.2f m | Z = %.2f m\n', ...
        guided_impact(1), guided_impact(2));

    fprintf('Reference error  : %.2f m\n', ref_error);
    fprintf('Corrected error  : %.2f m\n', guided_error);

    if ref_error > 0
        improvement = 100 * (ref_error - guided_error) / ref_error;
    else
        improvement = 0;
    end

    fprintf('Visual error reduction: %.1f %%\n', improvement);

    % =========================================================================
    % ANIMATION TIMING
    % =========================================================================

    fps = 15;

    total_frames = round(anim_duration_sec * fps);

    flight_frames = round(total_frames * 0.88);
    final_frames  = total_frames - flight_frames;

    frame_indices = round( ...
        linspace(1, N, flight_frames));

    gif_delay = anim_duration_sec / total_frames;

    fprintf('\n');
    fprintf('Animation duration : %.2f s\n', anim_duration_sec);
    fprintf('Frame rate         : %d FPS\n', fps);
    fprintf('Total frames       : %d\n', total_frames);
    fprintf('Flight frames      : %d\n', flight_frames);
    fprintf('Final frames       : %d\n', final_frames);

    % =========================================================================
    % FIGURE
    % =========================================================================

    is_headless = isempty(getenv('DISPLAY'));

    if is_headless
        vis_mode = 'off';
    else
        vis_mode = 'on';
    end

    fig = figure( ...
        'Visible', vis_mode, ...
        'Color', [0.08 0.10 0.14], ...
        'Position', [20 20 1560 860], ...
        'Name', 'Virtual Target Guided Trajectory');

    % =========================================================================
    % TARGET / VIRTUAL-TARGET GEOMETRY
    % =========================================================================

    circ_ang = linspace(0, 2*pi, 120);

    target_radius = 30;

    target_x = reference_target(1) + ...
        target_radius * cos(circ_ang);

    target_z = reference_target(2) + ...
        target_radius * sin(circ_ang);

    virtual_radius = 45;

    virtual_x = virtual_target(1) + ...
        virtual_radius * cos(circ_ang);

    virtual_z = virtual_target(2) + ...
        virtual_radius * sin(circ_ang);

    % =========================================================================
    % LIMITS
    % =========================================================================

    max_x = max([ref_x(:); corr_x(:)]) * 1.05;

    max_y = max([ref_y(:); corr_y(:)]) * 1.15;

    min_z = min([ ...
        0; ...
        ref_z(:); ...
        corr_z(:); ...
        reference_target(2); ...
        virtual_target(2)]) - 150;

    max_z = max([ ...
        ref_z(:); ...
        corr_z(:); ...
        reference_target(2); ...
        virtual_target(2)]) + 180;

    % =========================================================================
    % SUBPLOT 1 — 3D TRAJECTORY
    % =========================================================================

    ax3d = subplot(2,3,[1 4]);

    set(ax3d, ...
        'Color', [0.12 0.14 0.19], ...
        'XColor', [0.7 0.75 0.85], ...
        'YColor', [0.7 0.75 0.85], ...
        'ZColor', [0.7 0.75 0.85], ...
        'GridColor', [0.25 0.30 0.38], ...
        'GridAlpha', 0.6);

    hold(ax3d,'on');
    grid(ax3d,'on');
    box(ax3d,'on');

    % Origin
    plot3(ax3d,0,0,0,'^', ...
        'MarkerSize',11, ...
        'MarkerFaceColor',[0.95 0.75 0.2], ...
        'MarkerEdgeColor','w');

    % Actual target
    plot3(ax3d, ...
        reference_target(1), ...
        reference_target(2), ...
        0, ...
        'rx', ...
        'MarkerSize',14, ...
        'LineWidth',2.5);

    % Target accuracy region
    plot3(ax3d, ...
        target_x, ...
        target_z, ...
        zeros(size(target_x)), ...
        'r--', ...
        'LineWidth',1.6);

    % Virtual target
    plot3(ax3d, ...
        virtual_target(1), ...
        virtual_target(2), ...
        0, ...
        'o', ...
        'MarkerSize',10, ...
        'MarkerFaceColor',[0.75 0.4 1.0], ...
        'MarkerEdgeColor','w');

    plot3(ax3d, ...
        virtual_x, ...
        virtual_z, ...
        zeros(size(virtual_x)), ...
        ':', ...
        'Color',[0.75 0.4 1.0], ...
        'LineWidth',1.2);

    % Reference trajectory
    h_ref3d = plot3(ax3d, ...
        ref_x(1), ...
        ref_z(1), ...
        ref_y(1), ...
        '--', ...
        'Color',[0.45 0.50 0.60], ...
        'LineWidth',1.5);

    % Corrected trajectory
    h_guided3d = plot3(ax3d, ...
        corr_x(1), ...
        corr_z(1), ...
        corr_y(1), ...
        'Color',[0.20 0.80 1.00], ...
        'LineWidth',2.4);

    % Current projectile
    h_shell3d = plot3(ax3d, ...
        corr_x(1), ...
        corr_z(1), ...
        corr_y(1), ...
        'o', ...
        'MarkerSize',9, ...
        'MarkerFaceColor',[1.0 0.30 0.20], ...
        'MarkerEdgeColor','w', ...
        'LineWidth',1.5);

    % Velocity-like visual pointer
    h_vect3d = quiver3(ax3d, ...
        0,0,0, ...
        0,0,0, ...
        0, ...
        'Color',[1.0 0.85 0.3], ...
        'LineWidth',2.0, ...
        'MaxHeadSize',2.0);

    % Correction line
    h_corr_line = plot3(ax3d, ...
        [0 0], ...
        [0 0], ...
        [0 0], ...
        ':', ...
        'Color',[1.0 0.5 0.8], ...
        'LineWidth',1.8);

    xlabel(ax3d,'Downrange X (m)', ...
        'FontWeight','bold', ...
        'Color',[0.85 0.9 1.0]);

    ylabel(ax3d,'Crossrange Z (m)', ...
        'FontWeight','bold', ...
        'Color',[0.85 0.9 1.0]);

    zlabel(ax3d,'Altitude Y (m)', ...
        'FontWeight','bold', ...
        'Color',[0.85 0.9 1.0]);

    title(ax3d, ...
        'Virtual-Target Corrected Trajectory', ...
        'FontSize',11, ...
        'FontWeight','bold', ...
        'Color',[1 1 1]);

    xlim(ax3d,[-500 max_x]);
    ylim(ax3d,[min_z max_z]);
    zlim(ax3d,[0 max_y]);

    view(ax3d,[-38 22]);

    % =========================================================================
    % HUD
    % =========================================================================

    hud_text = text(ax3d, ...
        0.025, ...
        0.965, ...
        '', ...
        'Units','normalized', ...
        'FontName','monospace', ...
        'FontSize',9, ...
        'FontWeight','bold', ...
        'Color',[0.3 1.0 0.5], ...
        'BackgroundColor',[0.05 0.07 0.10], ...
        'EdgeColor',[0.2 0.5 0.3], ...
        'Margin',6, ...
        'VerticalAlignment','top');

    % =========================================================================
    % SUBPLOT 2 — ALTITUDE PROFILE
    % =========================================================================

    ax_side = subplot(2,3,2);

    configure_axis(ax_side);

    hold(ax_side,'on');

    h_side_ref = plot(ax_side, ...
        ref_x(1),ref_y(1), ...
        '--', ...
        'Color',[0.45 0.50 0.60], ...
        'LineWidth',1.4);

    h_side_guided = plot(ax_side, ...
        corr_x(1),corr_y(1), ...
        'Color',[0.20 0.80 1.00], ...
        'LineWidth',2.0);

    h_side_shell = plot(ax_side, ...
        corr_x(1),corr_y(1), ...
        'o', ...
        'MarkerSize',7, ...
        'MarkerFaceColor',[1.0 0.3 0.2], ...
        'MarkerEdgeColor','w');

    xlabel(ax_side,'Downrange X (m)', ...
        'FontWeight','bold', ...
        'Color',[0.85 0.9 1.0]);

    ylabel(ax_side,'Altitude Y (m)', ...
        'FontWeight','bold', ...
        'Color',[0.85 0.9 1.0]);

    title(ax_side, ...
        'Altitude Profile', ...
        'FontSize',10, ...
        'FontWeight','bold', ...
        'Color',[1 1 1]);

    xlim(ax_side,[0 max_x]);
    ylim(ax_side,[0 max_y]);

    % =========================================================================
    % SUBPLOT 3 — POSITION ERROR
    % =========================================================================

    ax_error = subplot(2,3,3);

    configure_axis(ax_error);

    hold(ax_error,'on');

    error_ref = sqrt( ...
        (ref_x - reference_target(1)).^2 + ...
        (ref_z - reference_target(2)).^2);

    error_guided = sqrt( ...
        (corr_x - reference_target(1)).^2 + ...
        (corr_z - reference_target(2)).^2);

    h_error_ref_full = plot(ax_error, ...
        reference.time, ...
        error_ref, ...
        ':', ...
        'Color',[0.45 0.50 0.60], ...
        'LineWidth',1.2);

    h_error_guided = plot(ax_error, ...
        reference.time(1), ...
        error_guided(1), ...
        'Color',[0.3 0.9 0.5], ...
        'LineWidth',2.2);

    h_error_dot = plot(ax_error, ...
        reference.time(1), ...
        error_guided(1), ...
        'o', ...
        'MarkerSize',6, ...
        'MarkerFaceColor',[0.3 0.9 0.5], ...
        'MarkerEdgeColor','w');

    xlabel(ax_error,'Flight Time (s)', ...
        'FontWeight','bold', ...
        'Color',[0.85 0.9 1.0]);

    ylabel(ax_error,'Position Error (m)', ...
        'FontWeight','bold', ...
        'Color',[0.85 0.9 1.0]);

    title(ax_error, ...
        'Reference vs Corrected Error', ...
        'FontSize',10, ...
        'FontWeight','bold', ...
        'Color',[1 1 1]);

    xlim(ax_error,[0 t_final*1.05]);

    ylim(ax_error,[0 max(error_ref)*1.10]);

    % =========================================================================
    % SUBPLOT 4 — GROUND TRACK
    % =========================================================================

    ax_ground = subplot(2,3,5);

    configure_axis(ax_ground);

    hold(ax_ground,'on');

    plot(ax_ground, ...
        reference_target(1), ...
        reference_target(2), ...
        'rx', ...
        'MarkerSize',13, ...
        'LineWidth',2.2);

    plot(ax_ground, ...
        target_x, ...
        target_z, ...
        'r--', ...
        'LineWidth',1.4);

    plot(ax_ground, ...
        virtual_target(1), ...
        virtual_target(2), ...
        'o', ...
        'MarkerSize',9, ...
        'MarkerFaceColor',[0.75 0.4 1.0], ...
        'MarkerEdgeColor','w');

    h_ground_ref = plot(ax_ground, ...
        ref_x(1),ref_z(1), ...
        '--', ...
        'Color',[0.45 0.50 0.60], ...
        'LineWidth',1.4);

    h_ground_guided = plot(ax_ground, ...
        corr_x(1),corr_z(1), ...
        'Color',[0.20 0.80 1.00], ...
        'LineWidth',2.0);

    h_ground_shell = plot(ax_ground, ...
        corr_x(1),corr_z(1), ...
        'o', ...
        'MarkerSize',7, ...
        'MarkerFaceColor',[1.0 0.3 0.2], ...
        'MarkerEdgeColor','w');

    xlabel(ax_ground,'Downrange X (m)', ...
        'FontWeight','bold', ...
        'Color',[0.85 0.9 1.0]);

    ylabel(ax_ground,'Crossrange Z (m)', ...
        'FontWeight','bold', ...
        'Color',[0.85 0.9 1.0]);

    title(ax_ground, ...
        'Ground Track / Correction Visualization', ...
        'FontSize',10, ...
        'FontWeight','bold', ...
        'Color',[1 1 1]);

    xlim(ax_ground,[0 max_x]);
    ylim(ax_ground,[min_z max_z]);

    % =========================================================================
    % SUBPLOT 5 — CORRECTION MAGNITUDE
    % =========================================================================

    ax_corr = subplot(2,3,6);

    configure_axis(ax_corr);

    hold(ax_corr,'on');

    correction_mag = sqrt( ...
        (corr_x - ref_x).^2 + ...
        (corr_y - ref_y).^2 + ...
        (corr_z - ref_z).^2);

    h_corr_full = plot(ax_corr, ...
        reference.time, ...
        correction_mag, ...
        ':', ...
        'Color',[0.7 0.45 0.8], ...
        'LineWidth',1.2);

    h_corr_dyn = plot(ax_corr, ...
        reference.time(1), ...
        correction_mag(1), ...
        'Color',[1.0 0.6 0.9], ...
        'LineWidth',2.2);

    h_corr_dot = plot(ax_corr, ...
        reference.time(1), ...
        correction_mag(1), ...
        'o', ...
        'MarkerSize',6, ...
        'MarkerFaceColor',[1.0 0.6 0.9], ...
        'MarkerEdgeColor','w');

    xlabel(ax_corr,'Flight Time (s)', ...
        'FontWeight','bold', ...
        'Color',[0.85 0.9 1.0]);

    ylabel(ax_corr,'Correction Offset (m)', ...
        'FontWeight','bold', ...
        'Color',[0.85 0.9 1.0]);

    title(ax_corr, ...
        'Virtual-Target Correction Offset', ...
        'FontSize',10, ...
        'FontWeight','bold', ...
        'Color',[1 1 1]);

    xlim(ax_corr,[0 t_final*1.05]);

    ylim(ax_corr,[0 max(1,max(correction_mag)*1.15)]);

    % =========================================================================
    % FRAME STORAGE
    % =========================================================================

    recorded_frames = cell(total_frames,1);
    color_map = [];

    fprintf('\nRendering animation...\n');

    % =========================================================================
    % PHASE 1 — FLIGHT
    % =========================================================================

    for f = 1:flight_frames

        idx = frame_indices(f);

        cur_t = reference.time(idx);

        cur_x = corr_x(idx);
        cur_y = corr_y(idx);
        cur_z = corr_z(idx);

        cur_v = corrected.v_mag(idx);
        cur_m = corrected.Mach(idx);

        cur_error = error_guided(idx);

        cur_correction = correction_mag(idx);

        % ---------------------------------------------------------------------
        % Current trajectory
        % ---------------------------------------------------------------------

        sub_ref_x = ref_x(1:idx);
        sub_ref_y = ref_y(1:idx);
        sub_ref_z = ref_z(1:idx);

        sub_x = corr_x(1:idx);
        sub_y = corr_y(1:idx);
        sub_z = corr_z(1:idx);

        sub_t = reference.time(1:idx);

        % ---------------------------------------------------------------------
        % 3D reference path
        % ---------------------------------------------------------------------

        set(h_ref3d, ...
            'XData',sub_ref_x, ...
            'YData',sub_ref_z, ...
            'ZData',sub_ref_y);

        % ---------------------------------------------------------------------
        % 3D corrected path
        % ---------------------------------------------------------------------

        set(h_guided3d, ...
            'XData',sub_x, ...
            'YData',sub_z, ...
            'ZData',sub_y);

        % ---------------------------------------------------------------------
        % Projectile marker
        % ---------------------------------------------------------------------

        set(h_shell3d, ...
            'XData',cur_x, ...
            'YData',cur_z, ...
            'ZData',cur_y);

        % ---------------------------------------------------------------------
        % Visual direction pointer
        % ---------------------------------------------------------------------

        if idx < N

            dv = [ ...
                corr_x(min(idx+1,N))-corr_x(max(idx-1,1)); ...
                corr_z(min(idx+1,N))-corr_z(max(idx-1,1)); ...
                corr_y(min(idx+1,N))-corr_y(max(idx-1,1))];

            dv_norm = norm(dv);

            if dv_norm > 1e-6
                dv = dv/dv_norm;
            end

        else
            dv = [1;0;0];
        end

        pointer_length = 700;

        set(h_vect3d, ...
            'XData',cur_x, ...
            'YData',cur_z, ...
            'ZData',cur_y, ...
            'UData',dv(1)*pointer_length, ...
            'VData',dv(2)*pointer_length, ...
            'WData',dv(3)*pointer_length);

        % ---------------------------------------------------------------------
        % Correction vector visualization
        % ---------------------------------------------------------------------

        set(h_corr_line, ...
            'XData',[ref_x(idx),cur_x], ...
            'YData',[ref_z(idx),cur_z], ...
            'ZData',[ref_y(idx),cur_y]);

        % ---------------------------------------------------------------------
        % HUD
        % ---------------------------------------------------------------------

        animation_time = (f-1)*gif_delay;

        hud_str = sprintf([ ...
            '-- VIRTUAL TARGET DEMONSTRATION --\n' ...
            'Simulation Time : %6.2f s\n' ...
            'Flight Time     : %6.2f s\n' ...
            '\n' ...
            'Current Position\n' ...
            '  X             : %8.1f m\n' ...
            '  Z             : %8.1f m\n' ...
            '  Altitude      : %8.1f m\n' ...
            '\n' ...
            'Reference Error : %8.1f m\n' ...
            'Correction Off. : %8.1f m\n' ...
            'Velocity        : %8.1f m/s\n' ...
            'Mach            : %8.2f\n' ...
            '\n' ...
            'Virtual Target  : [%6.0f, %6.0f]\n' ...
            'Target          : [%6.0f, %6.0f]'], ...
            animation_time, ...
            cur_t, ...
            cur_x, ...
            cur_z, ...
            cur_y, ...
            cur_error, ...
            cur_correction, ...
            cur_v, ...
            cur_m, ...
            virtual_target(1), ...
            virtual_target(2), ...
            reference_target(1), ...
            reference_target(2));

        set(hud_text,'String',hud_str);

        % ---------------------------------------------------------------------
        % Altitude
        % ---------------------------------------------------------------------

        set(h_side_ref, ...
            'XData',sub_ref_x, ...
            'YData',sub_ref_y);

        set(h_side_guided, ...
            'XData',sub_x, ...
            'YData',sub_y);

        set(h_side_shell, ...
            'XData',cur_x, ...
            'YData',cur_y);

        % ---------------------------------------------------------------------
        % Ground track
        % ---------------------------------------------------------------------

        set(h_ground_ref, ...
            'XData',sub_ref_x, ...
            'YData',sub_ref_z);

        set(h_ground_guided, ...
            'XData',sub_x, ...
            'YData',sub_z);

        set(h_ground_shell, ...
            'XData',cur_x, ...
            'YData',cur_z);

        % ---------------------------------------------------------------------
        % Error plot
        % ---------------------------------------------------------------------

        set(h_error_guided, ...
            'XData',sub_t, ...
            'YData',error_guided(1:idx));

        set(h_error_dot, ...
            'XData',cur_t, ...
            'YData',cur_error);

        % ---------------------------------------------------------------------
        % Correction plot
        % ---------------------------------------------------------------------

        set(h_corr_dyn, ...
            'XData',sub_t, ...
            'YData',correction_mag(1:idx));

        set(h_corr_dot, ...
            'XData',cur_t, ...
            'YData',cur_correction);

        drawnow;

        % ---------------------------------------------------------------------
        % Save frame
        % ---------------------------------------------------------------------

        if save_gif

            try

                fr = getframe(fig);

                im = frame2im(fr);

                try
                    [imind,cm] = rgb2ind(im);
                catch
                    [imind,cm] = rgb2ind(im,256);
                end

                if isempty(color_map)
                    color_map = cm;
                end

                recorded_frames{f} = imind;

            catch
                % Continue animation if frame capture fails.
            end

        end

    end

    % =========================================================================
    % PHASE 2 — FINAL COMPARISON
    % =========================================================================

    fprintf('Rendering final comparison...\n');

    plot3(ax3d, ...
        ref_x(end), ...
        ref_z(end), ...
        0, ...
        's', ...
        'MarkerSize',11, ...
        'MarkerFaceColor',[0.4 0.45 0.55], ...
        'MarkerEdgeColor','w');

    plot3(ax3d, ...
        corr_x(end), ...
        corr_z(end), ...
        0, ...
        'p', ...
        'MarkerSize',14, ...
        'MarkerFaceColor',[0.3 1.0 0.4], ...
        'MarkerEdgeColor','w', ...
        'LineWidth',2);

    % Reference miss line
    plot3(ax3d, ...
        [reference_target(1),ref_x(end)], ...
        [reference_target(2),ref_z(end)], ...
        [0 0], ...
        '--', ...
        'Color',[0.8 0.4 0.4], ...
        'LineWidth',1.8);

    % Corrected miss line
    plot3(ax3d, ...
        [reference_target(1),corr_x(end)], ...
        [reference_target(2),corr_z(end)], ...
        [0 0], ...
        '-', ...
        'Color',[0.3 1.0 0.4], ...
        'LineWidth',1.8);

    for imp_i = 1:final_frames

        f = flight_frames + imp_i;

        animation_time = (f-1)*gif_delay;

        hud_str = sprintf([ ...
            '== VIRTUAL-TARGET RESULT ==\n' ...
            'Simulation Time : %6.2f s\n' ...
            '\n' ...
            'TARGET\n' ...
            '  X             : %8.1f m\n' ...
            '  Z             : %8.1f m\n' ...
            '\n' ...
            'REFERENCE END\n' ...
            '  X             : %8.1f m\n' ...
            '  Z             : %8.1f m\n' ...
            '  Error         : %8.2f m\n' ...
            '\n' ...
            'CORRECTED END\n' ...
            '  X             : %8.1f m\n' ...
            '  Z             : %8.1f m\n' ...
            '  Error         : %8.2f m\n' ...
            '\n' ...
            'Visual Error Reduction : %6.1f %%'], ...
            animation_time, ...
            reference_target(1), ...
            reference_target(2), ...
            ref_x(end), ...
            ref_z(end), ...
            ref_error, ...
            corr_x(end), ...
            corr_z(end), ...
            guided_error, ...
            improvement);

        set(hud_text, ...
            'String',hud_str, ...
            'Color',[0.3 1.0 0.5], ...
            'EdgeColor',[0.2 0.6 0.3]);

        drawnow;

        if save_gif

            try

                fr = getframe(fig);

                im = frame2im(fr);

                try
                    [imind,cm] = rgb2ind(im);
                catch
                    [imind,cm] = rgb2ind(im,256);
                end

                if isempty(color_map)
                    color_map = cm;
                end

                recorded_frames{f} = imind;

            catch
                % Continue if frame capture fails.
            end

        end

    end

    % =========================================================================
    % FINAL SCREENSHOT
    % =========================================================================

    summary_file = 'guided_virtual_target_analysis.png';

    print(fig,summary_file,'-dpng','-r150');

    fprintf('\n');
    fprintf('Final dashboard saved to %s\n',summary_file);

    % =========================================================================
    % GIF EXPORT
    % =========================================================================

    if save_gif && ...
            ~isempty(recorded_frames{1}) && ...
            ~isempty(color_map)

        gif_filename = 'guided_virtual_target_15s.gif';

        fprintf('Writing GIF: %s\n',gif_filename);

        valid_count = sum(~cellfun(@isempty,recorded_frames));

        first_frame = recorded_frames{find( ...
            ~cellfun(@isempty,recorded_frames),1)};

        [H,W] = size(first_frame);

        gif_stack = zeros( ...
            H,W,1,valid_count,'uint8');

        write_idx = 1;

        for k = 1:length(recorded_frames)

            cur_frame = recorded_frames{k};

            if isempty(cur_frame)
                continue;
            end

            if size(cur_frame,1) ~= H || ...
                    size(cur_frame,2) ~= W

                std_frame = zeros(H,W,'uint8');

                cur_h = min(H,size(cur_frame,1));
                cur_w = min(W,size(cur_frame,2));

                std_frame(1:cur_h,1:cur_w) = ...
                    cur_frame(1:cur_h,1:cur_w);

                gif_stack(:,:,1,write_idx) = std_frame;

            else

                gif_stack(:,:,1,write_idx) = cur_frame;

            end

            write_idx = write_idx + 1;

        end

        imwrite( ...
            gif_stack, ...
            color_map, ...
            gif_filename, ...
            'gif', ...
            'DelayTime',gif_delay, ...
            'LoopCount',inf);

        fprintf( ...
            'GIF generated: %d frames @ %.4f s/frame\n', ...
            valid_count,gif_delay);

        % ---------------------------------------------------------------------
        % Optional MP4
        % ---------------------------------------------------------------------

        mp4_filename = 'guided_virtual_target_15s.mp4';

        [status,~] = system( ...
            'which ffmpeg >/dev/null 2>&1');

        if status == 0

            fprintf('Converting GIF to MP4...\n');

            cmd = sprintf( ...
                'ffmpeg -y -i "%s" -c:v libx264 -pix_fmt yuv420p -r %d "%s" >/dev/null 2>&1', ...
                gif_filename, ...
                fps, ...
                mp4_filename);

            system(cmd);

            if exist(mp4_filename,'file')

                fprintf( ...
                    'MP4 generated: %s\n', ...
                    mp4_filename);

            end

        else

            fprintf( ...
                'ffmpeg not found; MP4 conversion skipped.\n');

        end

    end

    % =========================================================================
    % SUMMARY
    % =========================================================================

    fprintf('\n');
    fprintf('===============================================================\n');
    fprintf('          VIRTUAL-TARGET ANIMATION COMPLETE\n');
    fprintf('===============================================================\n');
    fprintf('Reference error : %.2f m\n',ref_error);
    fprintf('Corrected error : %.2f m\n',guided_error);
    fprintf('Visual reduction: %.1f %%\n',improvement);
    fprintf('===============================================================\n');

end


% =========================================================================
% REFERENCE TRAJECTORY
% =========================================================================

function data = compute_reference_trajectory( ...
        v0,theta,psi,env_cfg)

    g0 = 9.80665;

    % Generic demonstration parameters.
    mass = 43.10;

    diameter = 0.155;

    area = pi*(diameter/2)^2;

    Cd0 = 0.25;

    vx = v0*cosd(theta)*cosd(psi);
    vy = v0*sind(theta);
    vz = v0*cosd(theta)*sind(psi);

    state = [0;0;0;vx;vy;vz];

    dt = 0.01;

    t = 0;

    max_steps = 16000;

    log_t = zeros(max_steps,1);
    log_pos = zeros(max_steps,3);
    log_vel = zeros(max_steps,3);
    log_mach = zeros(max_steps,1);

    k = 1;

    while state(2) >= 0 && ...
            state(1) < 50000 && ...
            t < 130

        x = state(1);
        y = state(2);
        z = state(3);

        vx = state(4);
        vy = state(5);
        vz = state(6);

        alt = max(0,y);

        [~,~,rho,a_sound] = ...
            isa_atmosphere(alt);

        wind = wind_profile(alt,env_cfg);

        v_rel = [ ...
            vx-wind(1); ...
            vy-wind(2); ...
            vz-wind(3)];

        v_rel_mag = norm(v_rel);

        if v_rel_mag < 1e-6
            v_rel_mag = 1e-6;
        end

        Mach = v_rel_mag/a_sound;

        Cd = cd_mach_model(Mach,Cd0);

        drag = ...
            -0.5*rho*area*Cd* ...
            v_rel_mag*v_rel;

        acceleration = ...
            [0;-g0;0] + drag/mass;

        log_t(k) = t;

        log_pos(k,:) = [x y z];

        log_vel(k,:) = [vx vy vz];

        log_mach(k) = Mach;

        state(4:6) = ...
            state(4:6) + acceleration*dt;

        state(1:3) = ...
            state(1:3) + state(4:6)*dt;

        t = t + dt;

        k = k + 1;

    end

    % Ground intersection interpolation

    k_last = k-1;

    if state(2) < 0 && log_pos(k_last,2) > 0

        frac = ...
            log_pos(k_last,2) / ...
            (log_pos(k_last,2)-state(2));

        impact_pos = ...
            log_pos(k_last,:) + ...
            frac*(state(1:3)' - log_pos(k_last,:));

        impact_t = ...
            log_t(k_last) + frac*dt;

    else

        impact_pos = state(1:3)';

        impact_pos(2) = 0;

        impact_t = t;

    end

    log_pos(k,:) = impact_pos;

    log_vel(k,:) = state(4:6)';

    log_t(k) = impact_t;

    log_mach(k) = log_mach(k_last);

    data.time = log_t(1:k);

    data.x = log_pos(1:k,1);

    data.y = log_pos(1:k,2);

    data.z = log_pos(1:k,3);

    data.vx = log_vel(1:k,1);

    data.vy = log_vel(1:k,2);

    data.vz = log_vel(1:k,3);

    data.v_mag = sqrt( ...
        data.vx.^2 + ...
        data.vy.^2 + ...
        data.vz.^2);

    data.Mach = log_mach(1:k);

end


% =========================================================================
% GEOMETRIC VIRTUAL-TARGET CORRECTION
% =========================================================================

function corrected = generate_visual_correction( ...
        reference,target,virtual_target)

    x = reference.x(:);
    y = reference.y(:);
    z = reference.z(:);

    N = length(x);

    s = linspace(0,1,N)';

    % ---------------------------------------------------------------------
    % Correction activation
    % ---------------------------------------------------------------------
    %
    % Starts near zero, grows smoothly, and reaches its maximum near the
    % latter part of the trajectory.
    %
    % This is a purely geometric visualization function.
    % ---------------------------------------------------------------------

    activation = 1 ./ ...
        (1 + exp(-12*(s-0.58)));

    activation = activation .* ...
        min(1,s/0.18);

    % ---------------------------------------------------------------------
    % Desired demonstration displacement
    % ---------------------------------------------------------------------

    target_dx = target(1) - x(end);

    target_dz = target(2) - z(end);

    % Virtual target creates an intermediate geometric reference.
    virtual_dx = virtual_target(1) - x(end);

    virtual_dz = virtual_target(2) - z(end);

    % ---------------------------------------------------------------------
    % Blend toward the virtual-target direction first.
    % ---------------------------------------------------------------------

    virtual_weight = 0.35;

    desired_dx = ...
        virtual_weight*virtual_dx + ...
        (1-virtual_weight)*target_dx;

    desired_dz = ...
        virtual_weight*virtual_dz + ...
        (1-virtual_weight)*target_dz;

    % ---------------------------------------------------------------------
    % Smooth spatial envelope
    % ---------------------------------------------------------------------

    spatial_envelope = ...
        sin(pi*min(max(s,0),1)/2).^2;

    correction_x = ...
        activation .* spatial_envelope * desired_dx;

    correction_z = ...
        activation .* spatial_envelope * desired_dz;

    % ---------------------------------------------------------------------
    % Do not modify altitude.
    %
    % The demonstration correction is shown primarily in the ground
    % coordinates so that the animation clearly illustrates the concept.
    % ---------------------------------------------------------------------

    corrected_x = x + correction_x;

    corrected_z = z + correction_z;

    corrected_y = y;

    % ---------------------------------------------------------------------
    % Force the final demonstration point to the target.
    %
    % This is an animation endpoint, not a calculated control action.
    % ---------------------------------------------------------------------

    endpoint_blend = ...
        min(1,max(0,(s-0.88)/0.12));

    corrected_x = ...
        (1-endpoint_blend).*corrected_x + ...
        endpoint_blend*target(1);

    corrected_z = ...
        (1-endpoint_blend).*corrected_z + ...
        endpoint_blend*target(2);

    % ---------------------------------------------------------------------
    % Smooth the generated curve
    % ---------------------------------------------------------------------

    corrected_x = smooth_curve(corrected_x);

    corrected_z = smooth_curve(corrected_z);

    % Restore exact endpoint.
    corrected_x(end) = target(1);
    corrected_z(end) = target(2);

    % ---------------------------------------------------------------------
    % Package output
    % ---------------------------------------------------------------------

    corrected.time = reference.time;

    corrected.x = corrected_x;

    corrected.y = corrected_y;

    corrected.z = corrected_z;

    % Approximate visual velocity.
    dt = gradient(corrected.time);

    corrected.vx = gradient(corrected.x)./max(dt,1e-6);

    corrected.vy = gradient(corrected.y)./max(dt,1e-6);

    corrected.vz = gradient(corrected.z)./max(dt,1e-6);

    corrected.v_mag = sqrt( ...
        corrected.vx.^2 + ...
        corrected.vy.^2 + ...
        corrected.vz.^2);

    corrected.Mach = reference.Mach;

end


% =========================================================================
% SMOOTH CURVE
% =========================================================================

function output = smooth_curve(input)

    input = input(:);

    N = length(input);

    if N < 5

        output = input;

        return;

    end

    window = 9;

    kernel = ones(window,1)/window;

    padded = [ ...
        repmat(input(1),floor(window/2),1); ...
        input; ...
        repmat(input(end),floor(window/2),1)];

    output = conv(padded,kernel,'valid');

    output = output(1:N);

end


% =========================================================================
% AXIS CONFIGURATION
% =========================================================================

function configure_axis(ax)

    set(ax, ...
        'Color',[0.12 0.14 0.19], ...
        'XColor',[0.7 0.75 0.85], ...
        'YColor',[0.7 0.75 0.85], ...
        'GridColor',[0.25 0.30 0.38], ...
        'GridAlpha',0.6);

    hold(ax,'on');

    grid(ax,'on');

    box(ax,'on');

end


% =========================================================================
% ISA ATMOSPHERE
% =========================================================================

function [T,P,rho,a_sound] = isa_atmosphere(alt)

    R_air = 287.05287;

    gamma = 1.4;

    g0 = 9.80665;

    T0 = 288.15;

    P0 = 101325;

    L = 0.0065;

    h_tropopause = 11000;

    h = max(0,min(alt,20000));

    if h <= h_tropopause

        T = T0 - L*h;

        P = P0 * ...
            (1-L*h/T0)^ ...
            (g0/(R_air*L));

    else

        T_trop = ...
            T0-L*h_tropopause;

        P_trop = ...
            P0*(1-L*h_tropopause/T0)^ ...
            (g0/(R_air*L));

        T = T_trop;

        P = P_trop*exp( ...
            -g0*(h-h_tropopause)/ ...
            (R_air*T_trop));

    end

    rho = P/(R_air*T);

    a_sound = sqrt(gamma*R_air*T);

end


% =========================================================================
% WIND PROFILE
% =========================================================================

function v_wind = wind_profile(alt,env_cfg)

    v_ground_mag = ...
        env_cfg.ground_wind_speed;

    wind_dir_deg = ...
        env_cfg.ground_wind_azimuth;

    if alt <= 2000

        v_mag = ...
            v_ground_mag * ...
            (max(10,alt)/2000)^0.2;

        v_dir = wind_dir_deg;

    elseif alt <= 8000

        frac = ...
            (alt-2000)/(8000-2000);

        v_mag = ...
            v_ground_mag*1.5 + ...
            frac*(env_cfg.jet_stream_speed*0.5);

        v_dir = ...
            wind_dir_deg + frac*30;

    elseif alt <= 11500

        jet_core = ...
            env_cfg.jet_stream_speed * ...
            exp(-((alt-9500)/1200)^2);

        v_mag = ...
            v_ground_mag*1.5 + jet_core;

        v_dir = ...
            wind_dir_deg + 45;

    else

        frac = ...
            min(1,(alt-11500)/4000);

        v_mag = ...
            (env_cfg.jet_stream_speed*0.4)* ...
            (1-0.5*frac);

        v_dir = ...
            wind_dir_deg+45;

    end

    theta_rad = deg2rad(v_dir);

    v_wind = [ ...
        v_mag*cos(theta_rad); ...
        0; ...
        v_mag*sin(theta_rad)];

end


% =========================================================================
% MACH-DEPENDENT DRAG MODEL
% =========================================================================

function Cd = cd_mach_model(Mach,Cd0)

    if Mach < 0.8

        Cd = Cd0;

    elseif Mach < 1.05

        Cd = Cd0 + ...
            0.22*((Mach-0.8)/0.25)^2;

    elseif Mach < 1.6

        Cd = ...
            (Cd0+0.22) - ...
            0.08*((Mach-1.05)/0.55);

    else

        Cd = ...
            (Cd0+0.14)/ ...
            (1+0.15*(Mach-1.6));

    end

end