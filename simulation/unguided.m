function unguided(num_runs)
    % =========================================================================
    % 155 mm CONVENTIONAL ARTILLERY - PURE UNGUIDED M107 BALLISTIC SIMULATION
    % =========================================================================
    % Simulates the standard 155 mm M107 High-Explosive (HE) projectile:
    %   - Projectile Mass: 43.10 kg (nominal 95.0 lbs)
    %   - Caliber Diameter: 155 mm (6.10 in)
    %   - Profile: 2.0-caliber radius tangent ogive with boat-tail
    %   - Drag Model: STANAG 4355 / McCoy aerodynamic drag coefficient Cd(Mach)
    %   - Environmental: ISA Atmosphere + High-Altitude Jet Stream Shear Layer
    %   - Dispersion: Gun barrel muzzle velocity and quadrant pointing variations
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
    
    % Preallocate impact positions array [X_impact, Z_impact]
    impact_points = zeros(num_runs, 2);
    
    % Store 3D trajectories for plotting
    max_traj_to_plot = min(num_runs, 40);
    trajectories = cell(max_traj_to_plot, 1);
    
    fprintf('Running %d UNGUIDED M107 Monte Carlo ballistic simulations...', num_runs);
    
    % --- Monte Carlo Loop ---
    for i = 1:num_runs
        % Launch state perturbations
        v0    = v0_nominal + sigma_v0 * randn;
        theta = theta_nominal + sigma_angle * randn;
        psi   = psi_nominal + sigma_angle * randn;
        
        % Perturbed environmental wind conditions per round
        env_cfg.ground_wind_speed   = max(0, 4.0 + 1.5 * randn);   % Ground wind speed (m/s)
        env_cfg.ground_wind_azimuth = 45.0 + 10.0 * randn;         % Wind direction azimuth (deg)
        env_cfg.jet_stream_speed    = max(15, 38.0 + 6.0 * randn); % Peak jet stream at 9.5 km (m/s)
        
        record_traj = (i <= max_traj_to_plot);
        [impact_points(i, :), traj] = run_unguided_flight(v0, theta, psi, env_cfg, record_traj);
        
        if record_traj
            trajectories{i} = traj;
        end
    end
    fprintf(' Done.\n');
    
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
                     'Name', '155 mm M107 Unguided Ballistic Analysis - Unified Engineering Display');
        
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
        for a_idx = 1:length(alts_plot)
            [~, ~, rhos_plot(a_idx), ~] = isa_atmosphere(alts_plot(a_idx));
            w_vec = wind_profile(alts_plot(a_idx), env_cfg);
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

% --- Pure 3-DoF Unguided M107 Flight Integration ---
function [impact, traj] = run_unguided_flight(v0, theta, psi, env_cfg, record_traj)
    if nargin < 5, record_traj = false; end
    
    % Authentic 155 mm M107 HE Projectile Physical Properties
    g0     = 9.80665;
    m      = 43.10;                % M107 Shell mass: 43.10 kg (95.0 lbs nominal)
    d_proj = 0.155;                % Caliber diameter: 155 mm (6.10 in)
    S_ref  = pi * (d_proj / 2)^2;  % Aerodynamic reference area: 0.01887 m^2
    Cd0    = 0.25;                 % M107 zero-lift nominal subsonic drag coefficient
    
    % Initial velocities
    vx = v0 * cosd(theta) * cosd(psi);
    vy = v0 * sind(theta);
    vz = v0 * cosd(theta) * sind(psi);
    
    % State vector: [x; y; z; vx; vy; vz] starting at [0,0,0]
    state = [0; 0; 0; vx; vy; vz];
    dt = 0.01;                     % Integration step (s)
    t = 0.0;
    
    if record_traj
        max_steps = 12000;
        traj_log = zeros(max_steps, 3);
        traj_log(1, :) = [0, 0, 0];
        log_k = 2;
    else
        traj_log = [];
    end
    
    while state(2) >= 0 && state(1) < 50000 && t < 120.0
        x = state(1); y = state(2); z = state(3);
        vx = state(4); vy = state(5); vz = state(6);
        alt = max(0, y);
        
        % 1. Atmospheric Model (ISA)
        [~, ~, rho, a_sound] = isa_atmosphere(alt);
        
        % 2. Altitude-Dependent Wind Profile (Crosswind & Jet Stream)
        v_wind = wind_profile(alt, env_cfg);
        
        % Relative aerodynamic velocity
        v_rel_x = vx - v_wind(1);
        v_rel_y = vy - v_wind(2);
        v_rel_z = vz - v_wind(3);
        v_rel_mag = sqrt(v_rel_x^2 + v_rel_y^2 + v_rel_z^2);
        
        % Mach number and wave drag
        Mach = v_rel_mag / a_sound;
        Cd = cd_mach_model(Mach, Cd0);
        
        % Aerodynamic Drag Vector (NO CONTROL / NO CANARDS)
        drag_const = 0.5 * rho * S_ref * Cd * v_rel_mag / m;
        ax_aero = -drag_const * v_rel_x;
        ay_aero = -drag_const * v_rel_y;
        az_aero = -drag_const * v_rel_z;
        
        % Pure Ballistic Acceleration (Drag + Gravity only)
        ax = ax_aero;
        ay = -g0 + ay_aero;
        az = az_aero;
        
        % State Integration
        state(1) = state(1) + vx * dt;
        state(2) = state(2) + vy * dt;
        state(3) = state(3) + vz * dt;
        state(4) = state(4) + ax * dt;
        state(5) = state(5) + ay * dt;
        state(6) = state(6) + az * dt;
        
        t = t + dt;
        
        if record_traj
            traj_log(log_k, :) = [state(1), state(2), state(3)];
            log_k = log_k + 1;
        end
    end
    
    % Linear ground-plane interpolation at impact (y = 0)
    fraction = y / (y - state(2));
    impact_x = x + fraction * (state(1) - x);
    impact_z = z + fraction * (state(3) - z);
    impact   = [impact_x, impact_z];
    
    if record_traj
        traj_log(log_k, :) = [impact_x, 0, impact_z];
        traj = traj_log(1:log_k, :);
    else
        traj = [];
    end
end

% --- COMPONENT 1: International Standard Atmosphere (ISA) ---
function [T, P, rho, a_sound] = isa_atmosphere(alt)
    R_air = 287.05287;      % Gas constant for dry air (J/(kg*K))
    gamma = 1.4;            % Specific heat ratio
    g0    = 9.80665;        % Standard gravitational acceleration (m/s^2)
    
    T0 = 288.15;            % Sea-level temperature (K)
    P0 = 101325;            % Sea-level pressure (Pa)
    L  = 0.0065;            % Temperature lapse rate (K/m)
    h_tropopause = 11000.0; % Tropopause boundary (m)
    
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

% --- COMPONENT 2: Altitude-Dependent Jet Stream Wind Profile ---
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
    Wx = v_mag * cos(theta_rad);
    Wy = 0.0;
    Wz = v_mag * sin(theta_rad);
    
    v_wind = [Wx; Wy; Wz];
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
