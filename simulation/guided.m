function guided(num_runs)
    % =========================================================================
    % 155 mm PRECISION GUIDANCE KIT (PGK) - SINGLE TRAJECTORY & ELLIPSE ANALYSIS
    % =========================================================================
    % Restructured display for clear viewer presentation:
    %   1. 3D Trajectory: Single perturbed round unguided vs. guided.
    %   2. 2D Ellipses: Bounding correction ellipses at 0s, 10s, 20s, 30s delay.
    %   3. Telemetry: Canard roll command and miss distance history.
    % =========================================================================

    if nargin < 1, num_runs = 250; end
    
    real_target = [18000, 400];        % Designated Target [Downrange, Crossrange] (m)
    v0_nominal  = 827.0;              % Nominal muzzle velocity (m/s)
    canard_deflection = 6.0;          % Canard deflection angle (degrees)
    
    % Dispersion parameters (1-Sigma Standard Deviations)
    sigma_v0    = 3.5;                % Muzzle velocity variation (m/s)
    sigma_angle = 0.15;               % Gun laying pointing error (degrees)
    
    fprintf('=== 155 mm PGK TRAJECTORY CORRECTION ANALYSIS ===\n');
    fprintf('Designated Target           : [%.1f, %.1f] m\n', real_target(1), real_target(2));
    fprintf('Canard Deflection Angle     : %.1f degrees\n', canard_deflection);
    
    % --- Step 1: Solve Firing Solution for Real Target ---
    fprintf('Solving nominal unguided firing solution...');
    [theta_trad, psi_trad] = solve_firing_solution(real_target, v0_nominal, canard_deflection);
    fprintf(' Done.\n');
    fprintf('  -> Gun Elevation (theta)  : %.4f deg\n', theta_trad);
    fprintf('  -> Gun Azimuth (psi)      : %.4f deg\n', psi_trad);

    % --- Step 2: Calibrate Correction Ellipses at Different Delays ---
    % Simulates Figure 8 from the paper (ellipses at 0s, 10s, 20s, 30s delay)
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
    
    % Virtual target is set using the 0s delay (apogee start) correction ellipse center
    x0_apogee = ellipse_data(1).x0;
    z0_apogee = ellipse_data(1).z0;
    virtual_target = real_target - [x0_apogee, z0_apogee];
    fprintf('  -> Calculated Virtual Target: [%.1f, %.1f] m\n', virtual_target(1), virtual_target(2));
    
    % Solve firing solution for the virtual target
    [theta_virt, psi_virt] = solve_firing_solution(virtual_target, v0_nominal, canard_deflection);
    
    % --- Step 3: Run Single Perturbed Round (Wind-Driven Off-Course) ---
    fprintf('Simulating single perturbed round (Strong wind perturbation)...\n');
    % We define a strong crosswind to push the round off-course
    perturbed_wind.spd = 9.0;    % Strong wind (m/s)
    perturbed_wind.az = -60.0;   % Pushes the round to the left
    perturbed_wind.jet = 42.0;
    
    % 1. Run unguided perturbed round
    [imp_s_ung, traj_s_ung, ~] = run_single_perturbed(v0_nominal, theta_trad, psi_trad, ...
                                                      canard_deflection, false, real_target, 0, perturbed_wind);
    
    % 2. Run guided perturbed round (starts GNC at apogee, guides to real target)
    [imp_s_gid, traj_s_gid, logs_s_gid] = run_single_perturbed(v0_nominal, theta_trad, psi_trad, ...
                                                      canard_deflection, true, real_target, 0, perturbed_wind);
    
    fprintf('  -> Unguided Perturbed Impact: [%.1f, %.1f] m (Miss: %.1f m)\n', ...
            imp_s_ung(1), imp_s_ung(2), norm(imp_s_ung - real_target));
    fprintf('  -> Guided Perturbed Impact  : [%.1f, %.1f] m (Miss: %.1f m)\n', ...
            imp_s_gid(1), imp_s_gid(2), norm(imp_s_gid - real_target));

    % --- Step 4: Run Statistical Monte Carlo to get CEP ---
    fprintf('\nRunning %d Monte Carlo rounds to verify statistical performance...', num_runs);
    t_start = tic;
    [imp_ung, ~, ~] = run_parallel_monte_carlo_guided(num_runs, v0_nominal, theta_trad, psi_trad, ...
                                                             sigma_v0, sigma_angle, 0, ...
                                                             real_target, false, canard_deflection, 0);
    [imp_virt, ~, ~] = run_parallel_monte_carlo_guided(num_runs, v0_nominal, theta_virt, psi_virt, ...
                                                                 sigma_v0, sigma_angle, 0, ...
                                                                 real_target, true, canard_deflection, 0);
    t_elapsed = toc(t_start);
    fprintf(' Done in %.2f s.\n', t_elapsed);

    % CEP Calculations
    miss_ung = sqrt((imp_ung(:, 1) - real_target(1)).^2 + (imp_ung(:, 2) - real_target(2)).^2);
    cep_ung = prctile(miss_ung, 50);
    miss_virt = sqrt((imp_virt(:, 1) - real_target(1)).^2 + (imp_virt(:, 2) - real_target(2)).^2);
    cep_virt = prctile(miss_virt, 50);
    pass_virt = 100 * mean(miss_virt <= 30.0);

    % --- Step 5: Visualizations & Plotting ---
    try
        is_headless = isempty(getenv('DISPLAY'));
        vis_mode = 'on';
        if is_headless, vis_mode = 'off'; end
        
        angles = linspace(0, 2*pi, 250);
        fig = figure('Visible', vis_mode, 'Color', [1 1 1], 'Position', [40, 40, 1600, 850], ...
                     'Name', '155 mm PGK Trajectory Correction - GNC Control Display');
        
        % =================================================================
        % DIAGRAM 1 (Left): 3D Trajectory Correction (Single Perturbed Round)
        % =================================================================
        ax_3d = subplot(2, 2, [1, 3]);
        hold(ax_3d, 'on'); grid(ax_3d, 'on'); box(ax_3d, 'on');
        
        % Launch Origin
        plot3(ax_3d, 0, 0, 0, 'p', 'MarkerSize', 13, 'MarkerFaceColor', [0.85 0.65 0.1], ...
              'MarkerEdgeColor', 'k', 'DisplayName', 'Launch Point [0,0,0]');
        text(ax_3d, 300, 0, 150, 'LAUNCH [0,0,0]', 'FontSize', 9, 'FontWeight', 'bold');
        
        % Unguided perturbed trajectory (Red dashed)
        plot3(ax_3d, traj_s_ung(:,1), traj_s_ung(:,3), traj_s_ung(:,2), 'r--', 'LineWidth', 1.8, ...
              'DisplayName', 'Unguided (Pushed by Wind)');
        
        % Guided perturbed trajectory (Green solid)
        plot3(ax_3d, traj_s_gid(:,1), traj_s_gid(:,3), traj_s_gid(:,2), 'g-', 'LineWidth', 2.2, ...
              'DisplayName', 'Guided (Canard Trajectory Correction)');
        
        % Target in 3D
        plot3(ax_3d, real_target(1), real_target(2), 0, 'rx', 'MarkerSize', 14, 'LineWidth', 3, 'DisplayName', 'Real Target');
        
        % Apogee point where GNC starts
        ap_idx = find(diff(traj_s_gid(:, 2)) < 0, 1);
        plot3(ax_3d, traj_s_gid(ap_idx, 1), traj_s_gid(ap_idx, 3), traj_s_gid(ap_idx, 2), 'bo', ...
              'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'GNC Activation (Apogee)');
        text(ax_3d, traj_s_gid(ap_idx, 1)+200, traj_s_gid(ap_idx, 3), traj_s_gid(ap_idx, 2)+200, ...
             'Canard Despin & Steering Starts', 'FontSize', 8, 'Color', 'b');
        
        % Spec Circle (30m)
        plot3(ax_3d, real_target(1) + 30*cos(angles), real_target(2) + 30*sin(angles), zeros(size(angles)), ...
              'k:', 'LineWidth', 1.8, 'DisplayName', '30m Spec Circle');
        
        xlabel(ax_3d, 'Downrange X (m)', 'FontSize', 10, 'FontWeight', 'bold');
        ylabel(ax_3d, 'Crossrange Z (m)', 'FontSize', 10, 'FontWeight', 'bold');
        zlabel(ax_3d, 'Altitude Y (m)', 'FontSize', 10, 'FontWeight', 'bold');
        title(ax_3d, '1. 3D Trajectory Correction - Single Perturbed Flight', 'FontSize', 12, 'FontWeight', 'bold');
        view(ax_3d, [-30, 18]);
        legend(ax_3d, 'Location', 'northeast');
        
        % =================================================================
        % DIAGRAM 2 (Top Right): Nested Correction Ellipses (Figure 8)
        % =================================================================
        ax_2d = subplot(2, 2, 2);
        hold(ax_2d, 'on'); grid(ax_2d, 'on'); box(ax_2d, 'on');
        
        % Nominal unguided impact is at (0,0) on this relative plot
        % Let's plot everything relative to the nominal unguided impact
        plot(ax_2d, 0, 0, 'r+', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Unguided nominal Impact (O_1)');
        
        % The target O_2 is at [x0_apogee, z0_apogee] relative to O_1
        % We align our ellipses centered at the target (O_2)
        target_rel = [x0_apogee, z0_apogee];
        plot(ax_2d, target_rel(1), target_rel(2), 'rx', 'MarkerSize', 14, 'LineWidth', 3, 'DisplayName', 'Real Target (O_2)');
        
        % Plot nested ellipses for different delays
        colors = {'#2ECC71', '#3498DB', '#9B59B6', '#E67E22'};
        for i = 1:length(ellipse_data)
            ed = ellipse_data(i);
            % Center of correction ellipse is O_2
            ex = target_rel(1) - (ed.x0 - x0_apogee) + ed.a * cos(angles);
            ez = target_rel(2) - (ed.z0 - z0_apogee) + ed.b * sin(angles);
            plot(ax_2d, ex, ez, 'Color', colors{i}, 'LineWidth', 1.8, ...
                 'DisplayName', sprintf('Correction Ellipse (Delay = %ds)', ed.delay));
        end
        
        % 30m Spec boundary around target
        plot(ax_2d, target_rel(1) + 30*cos(angles), target_rel(2) + 30*sin(angles), 'k:', 'LineWidth', 2.0, ...
             'DisplayName', '30m PGK Spec boundary');
        
        xlabel(ax_2d, 'Downrange relative to Unguided (m)', 'FontSize', 10, 'FontWeight', 'bold');
        ylabel(ax_2d, 'Crossrange relative to Unguided (m)', 'FontSize', 10, 'FontWeight', 'bold');
        title(ax_2d, '2. 2D Bounding Correction Ellipses (Fig. 8)', 'FontSize', 11, 'FontWeight', 'bold');
        legend(ax_2d, 'Location', 'northeast');
        axis(ax_2d, 'equal');
        
        % =================================================================
        % DIAGRAM 3 (Bottom Right): GNC Telemetry Analysis
        % =================================================================
        ax_telemetry = subplot(2, 2, 4);
        hold(ax_telemetry, 'on'); grid(ax_telemetry, 'on'); box(ax_telemetry, 'on');
        
        [yy_ax, h1, h2] = plotyy(ax_telemetry, logs_s_gid.time, rad2deg(logs_s_gid.gamma_c), ...
                                 logs_s_gid.time, logs_s_gid.pred_miss);
        
        set(h1, 'Color', [0.1 0.6 0.25], 'LineWidth', 2.0);
        set(h2, 'Color', [0.8 0.4 0.0], 'LineWidth', 2.0);
        
        ylabel(yy_ax(1), 'Canard Roll Angle \gamma_c (deg)', 'FontSize', 10, 'Color', [0.1 0.6 0.25]);
        ylabel(yy_ax(2), 'Predicted Target Miss (m)', 'FontSize', 10, 'Color', [0.8 0.4 0.0]);
        set(yy_ax(1), 'ycolor', [0.1 0.6 0.25]);
        set(yy_ax(2), 'ycolor', [0.8 0.4 0.0]);
        
        xlabel(ax_telemetry, 'Flight Time (s)', 'FontSize', 9);
        title(ax_telemetry, '3. Closed-Loop GNC Telemetry (Single Round Correction)', 'FontSize', 11, 'FontWeight', 'bold');
        
        print(fig, 'guided_all_in_one_analysis.png', '-dpng', '-r150');
        fprintf('Saved visual display plots to guided_all_in_one_analysis.png\n');
        
    catch err
        fprintf('Plotting note: %s\n', err.message);
    end
    
    % --- Command Window Summary Output ---
    fprintf('\n=== PERFORMANCE SUMMARY REPORT ===\n');
    fprintf('1. SINGLE ROUND TRAJECTORY CORRECTION:\n');
    fprintf('   Unguided Perturbed Impact: [%.1f, %.1f] m (Miss distance = %.1f m)\n', imp_s_ung(1), imp_s_ung(2), norm(imp_s_ung - real_target));
    fprintf('   Guided Perturbed Impact  : [%.1f, %.1f] m (Miss distance = %.1f m)\n', imp_s_gid(1), imp_s_gid(2), norm(imp_s_gid - real_target));
    fprintf('--------------------------------------------------\n');
    fprintf('2. MONTE CARLO STATISTICAL PERFORMANCE (%d rounds):\n', num_runs);
    fprintf('   Unguided Baseline CEP    : %.2f meters\n', cep_ung);
    fprintf('   Guided PGK CEP           : %.2f meters\n', cep_virt);
    fprintf('   Success Rate (<=30m Spec): %.1f %%\n', pass_virt);
    if cep_virt <= 30.0
        fprintf('STATUS                      : PASSED (CEP = %.2fm <= 30m)\n', cep_virt);
    else
        fprintf('STATUS                      : FAILED (CEP = %.2fm > 30m)\n', cep_virt);
    end
    fprintf('==================================================\n');
end

% =========================================================================
% PARALLEL MONTE CARLO TRAJECTORY ENGINE
% =========================================================================
function [impact_points, trajectories, control_logs] = run_parallel_monte_carlo_guided(...
    N, v0_nom, th_nom, psi_nom, sig_v0, sig_ang, max_plot, ...
    guidance_target, is_guided, canard_deflection_deg, gnc_start_delay)

    g0     = 9.80665;
    m      = 43.10;                % Shell mass (kg)
    d_proj = 0.155;                % Caliber diameter (m)
    S_ref  = pi * (d_proj / 2)^2;  % Reference area (m^2)
    Cd0    = 0.25;                 % Baseline zero-lift drag
    Ix     = 0.142;                % Axial moment of inertia (kg*m^2)
    twist_calibers = 20.0;         % 1:20 twist barrel
    dt     = 0.01;                 % Time step (s)

    Clp      = -0.015;             % Viscous roll damping coefficient
    C_La     = 2.0;                % Lift slope derivative (per rad)
    C_mag_p  = 0.008;              % Magnus derivative

    % Canard Aerodynamics
    delta_e = deg2rad(canard_deflection_deg);
    CL_delta = 1.4;                % Canard lift slope derivative (per rad)
    C_N_canard = CL_delta * delta_e; % Net steering force coefficient
    delta_kappa = deg2rad(160.0);   % Gyroscopic phase offset (160 deg)

    % Pre-generate all perturbed initial states
    v0_all  = v0_nom  + sig_v0  * randn(N, 1);
    th_all  = th_nom  + sig_ang * randn(N, 1);
    psi_all = psi_nom + sig_ang * randn(N, 1);

    % Pre-generate environmental winds per round
    gw_spd  = max(0, 4.0 + 1.5 * randn(N, 1));
    gw_az   = 45.0 + 10.0 * randn(N, 1);
    jet_spd = max(15, 38.0 + 6.0 * randn(N, 1));

    % Initial velocity vectors
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
    cmd_gamma_c = zeros(N, 1);      % Commanded canard roll angle (rad)
    impact_points = zeros(N, 2);

    % Preallocate trajectory logging buffers
    N_plot = min(N, max_plot);
    trajectories = cell(N_plot, 1);
    max_log_pts = 2000;
    plot_pos_log = zeros(N_plot, max_log_pts, 3);
    plot_log_k   = ones(N_plot, 1);

    for k = 1:N_plot
        plot_pos_log(k, 1, :) = [0, 0, 0];
        plot_log_k(k) = 2;
    end

    % Preallocate control logs
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

    % SIMD Parallel Batch Time Stepping
    while any(active) && t < 130.0
        idx = find(active);
        K = length(idx);

        y   = pos(idx, 2);
        alt = max(0, y);

        % Update apogee detection
        reached_ap = ~has_passed_apogee(idx) & (vel(idx, 2) < 0);
        if any(reached_ap)
            h_ap = idx(reached_ap);
            has_passed_apogee(h_ap) = true;
            apogee_time(h_ap) = t;
        end

        % GNC Update Loop at 2 Hz
        if is_guided && mod(step_count, 50) == 0
            % Active rounds that have passed apogee and met the start delay
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

        % ISA Atmosphere
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

        % Wind Profile
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

        % Airspeed
        v_rel_x = vel(idx, 1) - Wx;
        v_rel_y = vel(idx, 2);
        v_rel_z = vel(idx, 3) - Wz;
        v_rel_mag = sqrt(v_rel_x.^2 + v_rel_y.^2 + v_rel_z.^2);
        v_unit_x  = v_rel_x ./ max(1e-3, v_rel_mag);
        v_unit_y  = v_rel_y ./ max(1e-3, v_rel_mag);

        % Drag
        Mach = v_rel_mag ./ a_sound;
        Cd = repmat(Cd0, K, 1);
        m1 = Mach >= 0.8 & Mach < 1.05;
        Cd(m1) = Cd0 + 0.22 * ((Mach(m1) - 0.8) / 0.25).^2;
        m2 = Mach >= 1.05 & Mach < 1.6;
        Cd(m2) = (Cd0 + 0.22) - 0.08 * ((Mach(m2) - 1.05) / 0.55);
        m3 = Mach >= 1.6;
        Cd(m3) = (Cd0 + 0.14) ./ (1 + 0.15 * (Mach(m3) - 1.6));

        % Spin Decay
        dp_dt = (0.5 * rho .* v_rel_mag * S_ref * (d_proj^2) * Clp / Ix) .* p(idx);
        p(idx) = max(0, p(idx) + dp_dt * dt);

        drag_k  = 0.5 * rho * S_ref .* Cd .* v_rel_mag / m;
        ax_drag = -drag_k .* v_rel_x;
        ay_drag = -drag_k .* v_rel_y;
        az_drag = -drag_k .* v_rel_z;

        % Repose Drift
        vcg_z = v_unit_x * g0;
        denom = max(100.0, rho * S_ref * d_proj * 11.5 .* (v_rel_mag.^2));
        alpha_e_z = (2.0 * Ix * p(idx) ./ (denom .* v_rel_mag)) .* vcg_z;
        F_lift_z  = 0.5 * rho * S_ref .* (v_rel_mag.^2) * C_La .* alpha_e_z;

        % Magnus
        F_mag_x = 0.5 * rho * S_ref * d_proj * C_mag_p .* p(idx) .* v_rel_mag .* (v_unit_y .* alpha_e_z);
        F_mag_y = -0.5 * rho * S_ref * d_proj * C_mag_p .* p(idx) .* v_rel_mag .* (v_unit_x .* alpha_e_z);

        ax = ax_drag + F_mag_x / m;
        ay = -g0 + ay_drag + F_mag_y / m;
        az = az_drag + F_lift_z / m;

        % Canard Actuation Force
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

        % Integration
        prev_pos = pos(idx, :);
        vel(idx, 1) = vel(idx, 1) + ax * dt;
        vel(idx, 2) = vel(idx, 2) + ay * dt;
        vel(idx, 3) = vel(idx, 3) + az * dt;

        pos(idx, 1) = pos(idx, 1) + vel(idx, 1) * dt;
        pos(idx, 2) = pos(idx, 2) + vel(idx, 2) * dt;
        pos(idx, 3) = pos(idx, 3) + vel(idx, 3) * dt;

        % Log trajectory points
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

        % Ground Impact
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
        
        % Spin Drift (Gyroscopic Repose Lift)
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
function [imp_point, traj, control_log] = run_single_perturbed(v0, theta, psi, ...
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

    traj = zeros(15000, 3);
    traj(1, :) = pos;
    k_log = 2;

    control_log.time = [];
    control_log.phi_cmd = [];
    control_log.gamma_c = [];
    control_log.pred_miss = [];

    while pos(2) >= 0 && t < 130.0
        alt = max(0, pos(2));
        if vel(2) < 0 && ~has_passed_apogee
            has_passed_apogee = true;
            apogee_time = t;
        end

        % Atmosphere
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

        % Dynamic Wind Profile (uses wind_cfg if provided, else nominal)
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

        % GNC Closed-Loop Command Updates at 2 Hz
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
                
                control_log.time(end+1) = t;
                control_log.phi_cmd(end+1) = phi_cmd;
                control_log.gamma_c(end+1) = cmd_gamma_c;
                control_log.pred_miss(end+1) = miss_dist;
            else
                control_active = false;
            end
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

        % Apply Steering Forces
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
        
        if mod(step_count, 10) == 0
            traj(k_log, :) = pos;
            k_log = k_log + 1;
        end
        t = t + dt;
        step_count = step_count + 1;
    end

    frac = prev_pos(2) / (prev_pos(2) - pos(2));
    imp_point = [prev_pos(1) + frac * (pos(1) - prev_pos(1)), ...
                 prev_pos(3) + frac * (pos(3) - prev_pos(3))];
    traj(k_log, :) = [imp_point(1), 0, imp_point(2)];
    traj = traj(1:k_log, :);
end

% =========================================================================
% LOCAL FIRING SOLUTION SOLVER
% =========================================================================
function [theta, psi] = solve_firing_solution(target, v0_nom, canard_deflection_deg)
    theta = 54.5;
    psi = 0.0;
    max_iter = 15;
    tol = 1.0;
    
    % Default nominal wind environment for solver
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
    
    % Uncontrolled nominal impact
    [imp_unc, ~, ~] = run_single_perturbed(v0_nom, theta_nom, psi_nom, canard_deflection_deg, ...
                                           false, [0,0], 0, nom_wind);
    
    % Run nominal flights with forced steering commands in different orientations
    for i = 1:length(angles_deg)
        gamma_c_rad = deg2rad(angles_deg(i));
        
        % Force steering in this direction by overriding guidance loop inside run_single_perturbed
        imp = run_single_forced_steering(v0_nom, theta_nom, psi_nom, ...
            canard_deflection_deg, gamma_c_rad, gnc_delay, nom_wind);
        impacts(i, :) = imp;
    end
    
    rel_impacts = impacts - imp_unc;
    
    % Shift center of correction ellipse relative to O_1
    x0 = mean(rel_impacts(:, 1));
    z0 = mean(rel_impacts(:, 2));
    
    % Semi-axes bounds
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

        % Atmosphere
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

        % Wind
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

        % Apply Constant Calibration Steering Force
        if has_passed_apogee && (t >= apogee_time + gnc_start_delay)
            v_h = max(1e-3, sqrt(vel(1)^2 + vel(3)^2));
            eyaw_x = -vel(3) / v_h;
            eyaw_y = 0;
            eyaw_z = vel(1) / v_h;

            ev_x = vel(1) / v_rel_mag;
            ev_y = vel(2) / v_rel_mag;
            ev_z = vel(3) / v_rel_mag;

            epitch_x = -eyaw_z * ev_y;
            epitch_y = ...
                eyaw_z * ev_x - eyaw_x * ev_z;
            epitch_z = ...
                eyaw_x * ev_y;

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
