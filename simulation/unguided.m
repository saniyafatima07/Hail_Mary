function unguided(num_runs)
    % =========================================================================
    % 155 mm CONVENTIONAL ARTILLERY - PARALLEL UNGUIDED M107 BALLISTIC SIMULATION
    % =========================================================================
    % Simulates the standard 155 mm M107 High-Explosive (HE) projectile:
    %   - Projectile Mass: 43.10 kg (nominal 95.0 lbs)
    %   - Caliber Diameter: 155 mm (6.10 in)
    %   - Drag Model: STANAG 4355 / McCoy aerodynamic drag coefficient Cd(Mach)
    %   - Environmental: ISA Atmosphere + High-Altitude Jet Stream Shear Layer
    %   - Spin Dynamics: 1:20 twist rifling, viscous roll decay, yaw of repose
    %   - Dispersion: Gun barrel muzzle velocity and quadrant pointing variations
    %   - Parallel Architecture: High-throughput SIMD batch vectorization across
    %     all rounds simultaneously, utilizing multi-threaded OpenMP/BLAS cores.
    % =========================================================================

    if nargin < 1, num_runs = 1000; end
    target = [18000, 400];              % Designated Target [Downrange, Crossrange] (m)

    % Nominal Gun Firing Solution (M109/ATAGS 155mm, Top Zone Propelling Charge)
    v0_nominal    = 827.0;             % M107 nominal muzzle velocity at Charge 8 (m/s)
    theta_nominal = 54.5;              % Quadrant elevation angle (degrees) for ~18 km range
    psi_nominal   = 0.0;               % Gun azimuth laying angle (degrees)
    
    % Standard 155 mm Artillery Dispersion (1-Sigma Standard Deviations)
    sigma_v0      = 3.5;               % Muzzle velocity variation (m/s) (~0.4% PE)
    sigma_angle   = 0.15;              % Gun laying pointing error (degrees) (~2.6 mils)
    
    max_traj_to_plot = min(num_runs, 40);
    
    fprintf('Running %d UNGUIDED M107 Monte Carlo simulations (Parallel SIMD Engine)...', num_runs);
    t_start = tic;
    
    % --- Parallel Batch Monte Carlo Integration ---
    [impact_points, trajectories] = run_parallel_monte_carlo(num_runs, v0_nominal, theta_nominal, psi_nominal, ...
                                                             sigma_v0, sigma_angle, max_traj_to_plot);
    t_elapsed = toc(t_start);
    fprintf(' Done in %.2f s (%.0f rounds/sec).\n', t_elapsed, num_runs / t_elapsed);
    
    % --- Statistical & CEP Calculations ---
    dx = impact_points(:, 1) - target(1);
    dz = impact_points(:, 2) - target(2);
    miss_distances = sqrt(dx.^2 + dz.^2);
    
    % CEP is the median (50th percentile) radius from target
    cep_radius = prctile(miss_distances, 50);
    cep_90     = prctile(miss_distances, 90);
    pass_rate  = 100 * mean(miss_distances <= 30.0);
    fail_rate  = 100 - pass_rate;
    
    % Mean Impact Point (MIP) showing systemic wind drift bias
    mip = mean(impact_points, 1);
    
    is_hit  = miss_distances <= 30.0;
    is_miss = ~is_hit;
    
    % --- Visualizations: All 3 Diagrams in a Single Unified Window ---
    try
        is_headless = isempty(getenv('DISPLAY'));
        vis_mode = 'on';
        if is_headless, vis_mode = 'off'; end
        
        angles = linspace(0, 2*pi, 250);
        
        % UNIFIED SINGLE WINDOW (1600 x 850)
        fig = figure('Visible', vis_mode, 'Color', [1 1 1], 'Position', [40, 40, 1600, 850], ...
                     'Name', '155 mm M107 Unguided Ballistic Analysis - Parallel Engineering Display');
        
        % =================================================================
        % DIAGRAM 1 (Left 2 Subplots Span): 3D Trajectory Projection
        % =================================================================
        ax_3d = subplot(2, 2, [1, 3]);
        hold(ax_3d, 'on'); grid(ax_3d, 'on'); box(ax_3d, 'on');
        
        % Launch Origin [0,0,0]
        plot3(ax_3d, 0, 0, 0, 'p', 'MarkerSize', 13, 'MarkerFaceColor', [0.85 0.65 0.1], ...
              'MarkerEdgeColor', 'k', 'DisplayName', 'Launch Point [0,0,0]');
        text(ax_3d, 300, 0, 150, 'LAUNCH [0,0,0]', 'FontSize', 9, 'FontWeight', 'bold');
        
        % Plot 3D ballistic trajectory bundle
        first_line = true;
        for k = 1:length(trajectories)
            if ~isempty(trajectories{k})
                tr = trajectories{k};
                if first_line
                    plot3(ax_3d, tr(:,1), tr(:,3), tr(:,2), 'Color', [0.50 0.50 0.55], 'LineWidth', 1.0, ...
                          'DisplayName', 'M107 Unguided Trajectories');
                    first_line = false;
                else
                    plot3(ax_3d, tr(:,1), tr(:,3), tr(:,2), 'Color', [0.65 0.65 0.70], 'LineWidth', 0.8, ...
                          'HandleVisibility', 'off');
                end
            end
        end
        
        % Target in 3D ground plane
        plot3(ax_3d, target(1), target(2), 0, 'rx', 'MarkerSize', 14, 'LineWidth', 3, 'DisplayName', 'Target [18000, 400]');
        
        % Ground Impacts in 3D
        plot3(ax_3d, impact_points(:, 1), impact_points(:, 2), zeros(num_runs, 1), ...
              '.', 'Color', [0.8 0.25 0.2], 'MarkerSize', 8, 'DisplayName', 'M107 Ballistic Impacts');
        
        % Mean Impact Point in 3D
        plot3(ax_3d, mip(1), mip(2), 0, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', [0.15 0.5 0.95], ...
              'MarkerEdgeColor', 'k', 'DisplayName', sprintf('Mean Impact [%.1f, %.1f]', mip(1), mip(2)));
        
        % 30m Spec Circle in 3D
        plot3(ax_3d, target(1) + 30*cos(angles), target(2) + 30*sin(angles), zeros(size(angles)), ...
              'k:', 'LineWidth', 1.8, 'DisplayName', '30m PGK Spec Circle');
        
        % Calculated Unguided CEP Circle in 3D
        plot3(ax_3d, target(1) + cep_radius*cos(angles), target(2) + cep_radius*sin(angles), zeros(size(angles)), ...
              'r--', 'LineWidth', 2.0, 'DisplayName', sprintf('M107 CEP (50%% = %.1fm)', cep_radius));
        
        xlabel(ax_3d, 'Downrange X (m)', 'FontSize', 10, 'FontWeight', 'bold');
        ylabel(ax_3d, 'Crossrange Z (m)', 'FontSize', 10, 'FontWeight', 'bold');
        zlabel(ax_3d, 'Altitude Y (m)', 'FontSize', 10, 'FontWeight', 'bold');
        title(ax_3d, '1. 3D Trajectory Projection - 155 mm M107 HE', 'FontSize', 12, 'FontWeight', 'bold');
        view(ax_3d, [-35, 24]);
        legend(ax_3d, 'Location', 'northeast');
        
        % =================================================================
        % DIAGRAM 2 (Top Right): 2D Ground Impact Dispersion Footprint
        % =================================================================
        ax_2d = subplot(2, 2, 2);
        hold(ax_2d, 'on'); grid(ax_2d, 'on'); box(ax_2d, 'on');
        
        % Target
        plot(ax_2d, target(1), target(2), 'rx', 'MarkerSize', 15, 'LineWidth', 3, 'DisplayName', 'Target [18000, 400]');
        
        % 30m Requirement Circle
        plot(ax_2d, target(1) + 30*cos(angles), target(2) + 30*sin(angles), 'k:', 'LineWidth', 2.0, ...
             'DisplayName', '30m PGK Spec Boundary');
        
        % Calculated Unguided 50% CEP Circle
        plot(ax_2d, target(1) + cep_radius*cos(angles), target(2) + cep_radius*sin(angles), 'r--', 'LineWidth', 2.2, ...
             'DisplayName', sprintf('M107 CEP (50%% = %.1fm)', cep_radius));
        
        % 90% Ballistic Dispersion Circle
        plot(ax_2d, target(1) + cep_90*cos(angles), target(2) + cep_90*sin(angles), 'm-.', 'LineWidth', 1.5, ...
             'DisplayName', sprintf('90%% Dispersion (R = %.1fm)', cep_90));
        
        % Scatter Points: Hits vs Misses
        if any(is_hit)
            plot(ax_2d, impact_points(is_hit, 1), impact_points(is_hit, 2), '.', 'Color', [0.1 0.65 0.25], ...
                 'MarkerSize', 12, 'DisplayName', sprintf('<=30m (%d rounds / %.1f%%)', sum(is_hit), pass_rate));
        end
        if any(is_miss)
            plot(ax_2d, impact_points(is_miss, 1), impact_points(is_miss, 2), 'o', 'Color', [0.85 0.2 0.15], ...
                 'MarkerSize', 6, 'LineWidth', 1.2, 'DisplayName', sprintf('>30m (%d rounds / %.1f%%)', sum(is_miss), fail_rate));
        end
        
        % Mean Impact Point
        plot(ax_2d, mip(1), mip(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', [0.15 0.5 0.95], ...
             'MarkerEdgeColor', 'k', 'DisplayName', sprintf('Mean Impact [%.1f, %.1f]', mip(1), mip(2)));
        
        xlabel(ax_2d, 'Downrange Impact X (m)', 'FontSize', 10, 'FontWeight', 'bold');
        ylabel(ax_2d, 'Crossrange Impact Z (m)', 'FontSize', 10, 'FontWeight', 'bold');
        title(ax_2d, sprintf('2. 2D Ground Impact Footprint - M107 (CEP = %.1fm)', cep_radius), 'FontSize', 11, 'FontWeight', 'bold');
        legend(ax_2d, 'Location', 'northeast');
        axis(ax_2d, 'equal');
        
        % =================================================================
        % DIAGRAM 3 (Bottom Right): Atmospheric Profile (ISA & Jet Stream)
        % =================================================================
        ax_env = subplot(2, 2, 4);
        hold(ax_env, 'on'); grid(ax_env, 'on'); box(ax_env, 'on');
        alts_plot = linspace(0, 12000, 250);
        rhos_plot = zeros(size(alts_plot));
        winds_plot = zeros(size(alts_plot));
        nom_env.ground_wind_speed = 4.0;
        nom_env.ground_wind_azimuth = 45.0;
        nom_env.jet_stream_speed = 38.0;
        for a_idx = 1:length(alts_plot)
            [~, ~, rhos_plot(a_idx), ~] = isa_atmosphere(alts_plot(a_idx));
            w_vec = wind_profile(alts_plot(a_idx), nom_env);
            winds_plot(a_idx) = norm(w_vec);
        end
        plot(ax_env, alts_plot/1000, rhos_plot, 'Color', [0.0 0.45 0.85], 'LineWidth', 2, 'DisplayName', 'Air Density \rho (kg/m^3)');
        plot(ax_env, alts_plot/1000, winds_plot/10, 'Color', [0.9 0.4 0.1], 'LineWidth', 2, 'DisplayName', 'Wind Velocity / 10 (m/s)');
        xlabel(ax_env, 'Altitude (km)', 'FontSize', 9);
        ylabel(ax_env, 'Atmospheric Scaling', 'FontSize', 9);
        title(ax_env, '3. Atmospheric Profile (ISA & Jet Stream Shear)', 'FontSize', 11, 'FontWeight', 'bold');
        legend(ax_env, 'Location', 'northeast');
        
        print(fig, 'unguided_all_in_one_analysis.png', '-dpng', '-r150');
        fprintf('Saved all-in-one unified display to unguided_all_in_one_analysis.png\n');
        
    catch err
        fprintf('Plotting note: %s\n', err.message);
    end
    
    % --- Command Window Summary Output ---
    fprintf('\n=== UNGUIDED 155 mm M107 BALLISTIC PERFORMANCE ===\n');
    fprintf('Projectile Model            : 155 mm M107 HE (43.10 kg, 2.0-cal ogive)\n');
    fprintf('Simulated Rounds            : %d (Parallel SIMD Engine in %.2f s)\n', num_runs, t_elapsed);
    fprintf('Target Location             : [%.1f, %.1f] m\n', target(1), target(2));
    fprintf('Mean Impact Point (MIP)     : [%.1f, %.1f] m\n', mip(1), mip(2));
    fprintf('Systemic Range Bias (dx)    : %.1f meters\n', mip(1) - target(1));
    fprintf('Systemic Drift Bias (dz)    : %.1f meters\n', mip(2) - target(2));
    fprintf('Calculated CEP (50%%)        : %.2f meters\n', cep_radius);
    fprintf('Calculated CEP (90%%)        : %.2f meters\n', cep_90);
    fprintf('Rounds Meeting <=30m Spec   : %.1f %%\n', pass_rate);
    fprintf('Rounds Missing >30m Spec    : %.1f %%\n', fail_rate);
    if cep_radius <= 30
        fprintf('STATUS                      : PASSED\n');
    else
        fprintf('STATUS                      : FAILED (Standard unguided dispersion >> 30m)\n');
    end
    fprintf('==================================================\n');
end

% =========================================================================
% HIGH-SPEED PARALLEL VECTORIZED SIMD MONTE CARLO INTEGRATION ENGINE
% =========================================================================
function [impact_points, trajectories] = run_parallel_monte_carlo(N, v0_nom, th_nom, psi_nom, sig_v0, sig_ang, max_plot)
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
    impact_points = zeros(N, 2);

    % Preallocate trajectory logging buffers for the first max_plot rounds
    N_plot = min(N, max_plot);
    trajectories = cell(N_plot, 1);
    max_log_pts = 2000;
    plot_pos_log = zeros(N_plot, max_log_pts, 3);
    plot_log_k   = ones(N_plot, 1);

    for k = 1:N_plot
        plot_pos_log(k, 1, :) = [0, 0, 0];
        plot_log_k(k) = 2;
    end

    t = 0.0;
    step_count = 0;

    % SIMD Parallel Batch Time Stepping
    while any(active) && t < 130.0
        idx = find(active);
        K = length(idx);

        y   = pos(idx, 2);
        alt = max(0, y);

        % 1. Vectorized ISA Atmosphere
        h = max(0, min(alt, 20000));
        is_trop = h <= 11000;

        T = zeros(K, 1);
        P = zeros(K, 1);

        % Troposphere
        T(is_trop) = 288.15 - 0.0065 * h(is_trop);
        P(is_trop) = 101325 * (1 - 0.0065 * h(is_trop) / 288.15).^(9.80665 / (287.05287 * 0.0065));

        % Stratosphere
        T_trop = 216.65;
        P_trop = 101325 * (1 - 0.0065 * 11000 / 288.15)^(9.80665 / (287.05287 * 0.0065));
        T(~is_trop) = T_trop;
        P(~is_trop) = P_trop * exp(-9.80665 * (h(~is_trop) - 11000) / (287.05287 * T_trop));

        rho     = P ./ (287.05287 * T);
        a_sound = sqrt(1.4 * 287.05287 * T);

        % 2. Vectorized Wind Profile
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

        % 3. Relative Airspeed
        v_rel_x = vel(idx, 1) - Wx;
        v_rel_y = vel(idx, 2);
        v_rel_z = vel(idx, 3) - Wz;
        v_rel_mag = sqrt(v_rel_x.^2 + v_rel_y.^2 + v_rel_z.^2);
        v_unit_x  = v_rel_x ./ max(1e-3, v_rel_mag);
        v_unit_y  = v_rel_y ./ max(1e-3, v_rel_mag);

        % 4. Mach & Aerodynamic Drag
        Mach = v_rel_mag ./ a_sound;
        Cd = repmat(Cd0, K, 1);
        m1 = Mach >= 0.8 & Mach < 1.05;
        Cd(m1) = Cd0 + 0.22 * ((Mach(m1) - 0.8) / 0.25).^2;
        m2 = Mach >= 1.05 & Mach < 1.6;
        Cd(m2) = (Cd0 + 0.22) - 0.08 * ((Mach(m2) - 1.05) / 0.55);
        m3 = Mach >= 1.6;
        Cd(m3) = (Cd0 + 0.14) ./ (1 + 0.15 * (Mach(m3) - 1.6));

        % 5. Viscous Spin Decay
        dp_dt = (0.5 * rho .* v_rel_mag * S_ref * (d_proj^2) * Clp / Ix) .* p(idx);
        p(idx) = max(0, p(idx) + dp_dt * dt);

        % 6. Aerodynamic Drag Accelerations
        drag_k  = 0.5 * rho * S_ref .* Cd .* v_rel_mag / m;
        ax_drag = -drag_k .* v_rel_x;
        ay_drag = -drag_k .* v_rel_y;
        az_drag = -drag_k .* v_rel_z;

        % 7. Gyroscopic Repose Lift (Spin Drift)
        vcg_z = v_unit_x * g0;
        denom = max(100.0, rho * S_ref * d_proj * 11.5 .* (v_rel_mag.^2));
        alpha_e_z = (2.0 * Ix * p(idx) ./ (denom .* v_rel_mag)) .* vcg_z;
        F_lift_z  = 0.5 * rho * S_ref .* (v_rel_mag.^2) * C_La .* alpha_e_z;

        % 8. Magnus Force
        F_mag_x = 0.5 * rho * S_ref * d_proj * C_mag_p .* p(idx) .* v_rel_mag .* (v_unit_y .* alpha_e_z);
        F_mag_y = -0.5 * rho * S_ref * d_proj * C_mag_p .* p(idx) .* v_rel_mag .* (v_unit_x .* alpha_e_z);

        % Total Accelerations
        ax = ax_drag + F_mag_x / m;
        ay = -g0 + ay_drag + F_mag_y / m;
        az = az_drag + F_lift_z / m;

        % 9. Symplectic Euler-Cromer Integration
        prev_pos = pos(idx, :);
        vel(idx, 1) = vel(idx, 1) + ax * dt;
        vel(idx, 2) = vel(idx, 2) + ay * dt;
        vel(idx, 3) = vel(idx, 3) + az * dt;

        pos(idx, 1) = pos(idx, 1) + vel(idx, 1) * dt;
        pos(idx, 2) = pos(idx, 2) + vel(idx, 2) * dt;
        pos(idx, 3) = pos(idx, 3) + vel(idx, 3) * dt;

        % Sample trajectories for plotting (every 10 steps = 0.1s)
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

        % Ground Impact Detection & Exact Linear Interpolation
        hit = pos(idx, 2) < 0;
        if any(hit)
            h_idx = idx(hit);
            frac = prev_pos(hit, 2) ./ (prev_pos(hit, 2) - pos(h_idx, 2));
            imp_x = prev_pos(hit, 1) + frac .* (pos(h_idx, 1) - prev_pos(hit, 1));
            imp_z = prev_pos(hit, 3) + frac .* (pos(h_idx, 3) - prev_pos(hit, 3));
            impact_points(h_idx, 1) = imp_x;
            impact_points(h_idx, 2) = imp_z;

            % Append final impact coordinate to plotted trajectories
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

    % Package trajectory bundle
    for k = 1:N_plot
        total_pts = plot_log_k(k) - 1;
        trajectories{k} = squeeze(plot_pos_log(k, 1:total_pts, :));
    end
end

% --- Component 1: International Standard Atmosphere (ISA) ---
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

% --- Component 2: Altitude-Dependent Jet Stream Wind Profile ---
function v_wind = wind_profile(alt, env_cfg)
    v_ground_mag = env_cfg.ground_wind_speed;
    wind_dir_deg = env_cfg.ground_wind_azimuth;
    if alt <= 2000
        v_mag = v_ground_mag * (max(10, alt) / 2000)^0.2;
        v_dir = wind_dir_deg;
    elseif alt > 2000 && alt <= 8000
        frac = (alt - 2000) / (8000 - 2000);
        v_mag = v_ground_mag * 1.5 + frac * (env_cfg.jet_stream_speed * 0.5);
        v_dir = wind_dir_deg + frac * 30.0;
    elseif alt > 8000 && alt <= 11500
        jet_core = env_cfg.jet_stream_speed * exp(-((alt - 9500) / 1200)^2);
        v_mag = (v_ground_mag * 1.5) + jet_core;
        v_dir = wind_dir_deg + 45.0;
    else
        frac = min(1.0, (alt - 11500) / 4000);
        v_mag = (env_cfg.jet_stream_speed * 0.4) * (1 - 0.5 * frac);
        v_dir = wind_dir_deg + 45.0;
    end
    theta_rad = deg2rad(v_dir);
    v_wind = [v_mag * cos(theta_rad); 0.0; v_mag * sin(theta_rad)];
end
