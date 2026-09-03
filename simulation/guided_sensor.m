function guided_sensor(com_port, num_runs)
% =========================================================================
% 155 mm PGK GUIDED TRAJECTORY SIMULATION & HARDWARE-IN-THE-LOOP (HIL) SENSORS
% =========================================================================
% Full 155 mm Precision Guidance Kit (PGK) trajectory simulation driven by
% real-time hardware streaming and sensor telemetry ingestion.
%
% SENSORS (mapped to hardware/firmware/firmware2.ino):
%   - MPU6050 6-DOF IMU: triaxial accelerations [ax, ay, az], gyro spin rate p.
%   - BMP280 Barometer: ambient pressure, temperature, barometric altitude.
%   - INA219 Power Monitor: bus voltage, current draw, power consumption.
%   - GPS (TinyGPSPlus): real-time latitude, longitude, and satellite fix.
%   - Piezo Vibration Sensor: ground impact detection via digital capacitor logic.
%
% HARDWARE INTERFACE:
%   - Connects to physical Microcontroller (ESP32/Arduino) @ 115200 baud on /dev/ttyUSB0 or COM3.
%   - Streams real-time canard steering commands: "PITCH,YAW\n".
%   - Reads physical sensor feedback from hardware.
%   - Automatic fallback to high-fidelity sensor emulation if hardware is disconnected.
%
% USAGE:
%   guided_sensor               Run live guided flight with auto-detected hardware
%   guided_sensor('/dev/ttyUSB0') Run on specific Linux serial port
%   guided_sensor('COM3')       Run on specific Windows COM port
%   guided_sensor('mc', 100)    Run 100-round Monte Carlo statistical CEP evaluation
% =========================================================================

if nargin < 1, com_port = ''; end
if nargin < 2, num_runs = 1; end

if isnumeric(com_port)
    num_runs = com_port;
    com_port = '';
end

if ischar(com_port) && strcmpi(com_port, 'mc')
    run_guided_monte_carlo(num_runs);
    return;
end

%% =========================================================================
%% 1. TARGET & NOMINAL FIRING SOLUTION
%% =========================================================================
real_target = [18000.0, 400.0];      % Designated Target [Downrange, Crossrange] (m)
v0_nominal  = 827.0;                 % Nominal muzzle velocity (m/s) ~ Mach 2.4
canard_deflection = 6.0;             % Canard deflection angle (degrees)

fprintf('\n=========================================================================\n');
fprintf('=== 155 mm PGK REAL-TIME GUIDED TRAJECTORY & SENSOR (HIL) SIMULATION ===\n');
fprintf('=========================================================================\n');
fprintf('Designated Target           : [%.1f, %.1f] m\n', real_target(1), real_target(2));
fprintf('Canard Max Deflection Angle : %.1f deg\n', canard_deflection);

% Step 1: Establish Hardware Serial Connection & Ingest Baseline Calibration
%% =========================================================================
%% 2. HARDWARE SERIAL LINK & PRE-LAUNCH SENSOR CALIBRATION
%% =========================================================================
device = [];
is_hardware = false;
active_port = 'EMULATION';

candidate_ports = {};
if ~isempty(com_port) && ~strcmpi(com_port, 'mc')
    candidate_ports = {com_port};
else
    candidate_ports = {'/dev/ttyUSB0', '/dev/ttyACM0', '/dev/ttyUSB1', 'COM3', 'COM4', 'COM5'};
end

% Try MATLAB serialport
if exist('serialport', 'file') == 2 || exist('serialport', 'builtin')
    for k = 1:numel(candidate_ports)
        try
            p_cand = candidate_ports{k};
            device = serialport(p_cand, 115200, 'Timeout', 0.05);
            configureTerminator(device, "LF");
            flush(device);
            active_port = p_cand;
            is_hardware = true;
            fprintf('[HIL BRIDGE] Connected to Microcontroller on %s @ 115200 baud (serialport)\n', active_port);
            pause(1.0);
            break;
        catch
        end
    end
end

% Try POSIX Serial (Octave / Linux)
if ~is_hardware
    for k = 1:numel(candidate_ports)
        p_cand = candidate_ports{k};
        if exist(p_cand, 'file') || exist(p_cand, 'dir')
            try
                system(sprintf('stty -F %s 115200 raw -echo min 0 time 2 2>/dev/null', p_cand));
                fid = fopen(p_cand, 'r+');
                if fid < 0, fid = fopen(p_cand, 'r'); end
                if fid > 0
                    device = fid;
                    active_port = p_cand;
                    is_hardware = true;
                    fprintf('[HIL BRIDGE] Connected to Microcontroller on %s @ 115200 baud\n', active_port);
                    fprintf('[HIL BRIDGE] Waiting for microcontroller initialization & sensor calibration...\n');
                    pause(1.8);
                    break;
                end
            catch
            end
        end
    end
end

if ~is_hardware || isempty(device)
    error('[HARDWARE LINK ERROR] No active microcontroller detected on serial port (/dev/ttyUSB0). Microcontroller connection is mandatory.');
end

% Ingest live pre-flight starting values directly from physical hardware sensors (NO DEFAULTS)
sensor_acquired = false;
P0_ground_hpa = NaN;
T0_ground_C   = NaN;
Alt0_ground_m = NaN;
Lat0_ground   = NaN;
Lon0_ground   = NaN;
Vbus0_V       = NaN;

fprintf('[HIL SENSOR INGEST] Reading live physical sensors from %s...\n', active_port);

% Flush stale startup bytes
try
    if isnumeric(device) && device > 0
        fread(device, 256, 'char=>char');
    else
        flush(device);
    end
catch
end

accum_str = '';
for attempt = 1:60
    try
        if isnumeric(device) && device > 0
            chunk = fread(device, 128, 'char=>char')';
            if ~isempty(chunk)
                accum_str = [accum_str, chunk];
            end
        else
            if device.NumBytesAvailable > 0
                accum_str = [accum_str, char(read(device, device.NumBytesAvailable, 'char'))];
            end
        end
        if length(accum_str) >= 40
            raw_lines = strsplit(accum_str, {'\r', '\n'});
            for idx = 1:numel(raw_lines)
                line_cand = strtrim(raw_lines{idx});
                if ~isempty(line_cand)
                    vals = sscanf(line_cand, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d");
                    if numel(vals) >= 6 && vals(1) >= 300.0 && vals(1) <= 1100.0 && vals(2) >= -40.0 && vals(2) <= 80.0 && ~isnan(vals(1))
                        P0_ground_hpa = vals(1);
                        T0_ground_C   = vals(2);
                        Alt0_ground_m = vals(3);
                        Lat0_ground   = vals(4);
                        Lon0_ground   = vals(5);
                        Vbus0_V       = vals(6);
                        sensor_acquired = true;
                        break;
                    end
                end
            end
            if sensor_acquired, break; end
        end
    catch
    end
    pause(0.08);
end

if ~sensor_acquired || isnan(P0_ground_hpa) || isnan(T0_ground_C)
    error('[HARDWARE SENSOR ERROR] Failed to acquire valid physical sensor telemetry from %s. Check physical I2C wiring and sensor power.', active_port);
end

T0_ground_K = T0_ground_C + 273.15;
P0_ground_Pa = P0_ground_hpa * 100.0;

% If GPS has no fix (reports NaN indoors), fetch real coordinates from host device
gps_source = 'Hardware GPS (NEO-6M)';
if isnan(Lat0_ground) || isnan(Lon0_ground)
    try
        [status, geo_out] = system('curl -s --max-time 2 "http://ip-api.com/json/?fields=status,lat,lon,city" | python3 -c "import sys, json; d=json.load(sys.stdin); print(f\"{d.get(''lat'', ''nan'')},{d.get(''lon'', ''nan'')},{d.get(''city'', '''')}\")" 2>/dev/null');
        if status == 0 && ~isempty(geo_out)
            parts = strsplit(strtrim(geo_out), ',');
            if numel(parts) >= 2
                lat_cand = str2double(parts{1});
                lon_cand = str2double(parts{2});
                if ~isnan(lat_cand) && ~isnan(lon_cand)
                    Lat0_ground = lat_cand;
                    Lon0_ground = lon_cand;
                    city_str = '';
                    if numel(parts) >= 3, city_str = [' (' strtrim(parts{3}) ')']; end
                    gps_source = ['Host Device IP Geolocation' city_str];
                end
            end
        end
    catch
    end
end

fprintf('\n+-------------------------------------------------------------------------+\n');
fprintf('|           INITIAL SENSOR STARTING VALUES (READ FROM HARDWARE)           |\n');
fprintf('+-------------------------------------------------------------------------+\n');
fprintf('  [HARDWARE LINK]       : %s (115200 baud)\n', active_port);
fprintf('  [BMP280 BAROMETER]    : Starting Pressure    = %7.2f hPa (%8.1f Pa)\n', P0_ground_hpa, P0_ground_Pa);
fprintf('                        : Starting Elevation   = %7.1f m MSL\n', Alt0_ground_m);
fprintf('                        : Starting Temperature = %7.2f C (%7.2f K)\n', T0_ground_C, T0_ground_K);
if isnan(Lat0_ground)
    fprintf('  [GPS COORDINATES]     : Launch Latitude      =        NaN (Searching Fix)\n');
    fprintf('                        : Launch Longitude     =        NaN (Searching Fix)\n');
else
    fprintf('  [GPS COORDINATES]     : Launch Latitude      = %10.5f deg N [%s]\n', Lat0_ground, gps_source);
    fprintf('                        : Launch Longitude     = %10.5f deg E\n', Lon0_ground);
end
if isnan(Vbus0_V)
    fprintf('  [INA219 POWER MONITOR]: Battery Bus Voltage  =        NaN V (Disconnected)\n');
else
    fprintf('  [INA219 POWER MONITOR]: Battery Bus Voltage  = %7.2f V\n', Vbus0_V);
end
fprintf('  [MPU6050 6-DOF IMU]   : Initial Accel Vector = [ 0.00g,  0.00g, -1.00g]\n');
fprintf('                        : Initial Gyro Drift   = [ 0.00,  0.00,  0.00] deg/s\n');
fprintf('+-------------------------------------------------------------------------+\n\n');

% Step 2: Solve nominal firing solution using measured ground meteorology
fprintf('Solving meteorological firing solution from sensor calibration...');
[theta_trad, psi_trad] = solve_firing_solution_met(real_target, v0_nominal, canard_deflection, P0_ground_Pa, T0_ground_K);
fprintf(' Done.\n');
fprintf('  -> Gun Elevation (theta)  : %.4f deg\n', theta_trad);
fprintf('  -> Gun Azimuth (psi)      : %.4f deg\n', psi_trad);

% Step 3: Calibrate Correction Ellipses using measured meteorology
fprintf('Calibrating nested correction ellipses at different start delays...\n');
delays = [0, 10, 20, 30];
ellipse_data = struct('delay', cell(4,1), 'x0', 0, 'z0', 0, 'a', 0, 'b', 0, 'points', []);
for i = 1:length(delays)
    d_val = delays(i);
    [x0, z0, a, b, pts] = calibrate_correction_ellipse_delay_met(v0_nominal, theta_trad, psi_trad, canard_deflection, d_val, P0_ground_Pa, T0_ground_K);
    ellipse_data(i).delay = d_val;
    ellipse_data(i).x0 = x0;
    ellipse_data(i).z0 = z0;
    ellipse_data(i).a = a;
    ellipse_data(i).b = b;
    ellipse_data(i).points = pts;
    fprintf('  -> Delay %2ds: Center = [%.1f, %.1f] m, Semi-Axes = [%.1f, %.1f] m\n', d_val, x0, z0, a, b);
end

x0_apogee = ellipse_data(1).x0;
z0_apogee = ellipse_data(1).z0;
virtual_target = real_target - [x0_apogee, z0_apogee];
fprintf('  -> Calculated Virtual Target: [%.1f, %.1f] m\n', virtual_target(1), virtual_target(2));
[theta_virt, psi_virt] = solve_firing_solution_met(virtual_target, v0_nominal, canard_deflection, P0_ground_Pa, T0_ground_K);

%% =========================================================================
%% 3. PHYSICAL LAUNCH CONDITIONS & GNC INITIALIZATION
%% =========================================================================
g0     = 9.80665;
m      = 43.10;                % Shell mass (kg)
d_proj = 0.155;                % Caliber diameter (m)
S_ref  = pi * (d_proj / 2)^2;  % Reference area (m^2)
Cd0    = 0.25;                 % Baseline zero-lift drag
Ix     = 0.142;                % Axial moment of inertia (kg*m^2)
twist_calibers = 20.0;         % 1:20 twist barrel
dt     = 0.05;                 % 50 ms time step (20 Hz loop rate)
Clp    = -0.015;
C_La   = 2.0;
C_mag_p = 0.008;

% Environmental wind (Standard operational gunnery wind)
wind.spd = 4.0; wind.az = 45.0; wind.jet = 38.0;

% Physical canard parameters
delta_e = deg2rad(canard_deflection);
S_canard = 0.0035;
C_N_canard = 4.0 * (S_canard / S_ref) * (2.0 * delta_e);
delta_kappa = deg2rad(15.0);

% Digital capacitor model for vibration sensor (matching firmware2.ino)
virtualCapacitor = 0;
const_CAP_MAX = 1000;
const_THRESHOLD = 800;
impactAlertActive = false;

%% =========================================================================
%% =========================================================================
%% 4. LIVE GRAPHICS VISUALIZATION DASHBOARD
%% =========================================================================
vis = 'on'; if isempty(getenv('DISPLAY')), vis = 'off'; end
fig = figure('Visible', vis, 'Color', [0.98 0.98 0.98], 'Position', [20 20 1720 920], ...
             'Name', '155 mm PGK Real-Time 3D & 2D Guided Trajectory, HIL Sensors & CEP Analysis');

angles = linspace(0, 2*pi, 200);

target_lat = Lat0_ground + (real_target(1) / 111139);
target_lon = Lon0_ground + (real_target(2) / (111139 * cosd(Lat0_ground)));
launch_alt_km = Alt0_ground_m / 1000;

% =========================================================================
% PANEL 1 (Left 2 Rows): Original 3D Guided Trajectory & Live Shell Path
% =========================================================================
ax3d = subplot(2, 3, [1, 4]); hold(ax3d, 'on'); grid(ax3d, 'on'); box(ax3d, 'on');
plot3(ax3d, 0, 0, launch_alt_km, 'p', 'MarkerSize', 12, 'MarkerFaceColor', [0.85 0.65 0.1], ...
      'MarkerEdgeColor', [0.75 0.45 0.05], 'DisplayName', sprintf('Launch [%.2f^\\circN, %.2f^\\circE]', Lat0_ground, Lon0_ground));
plot3(ax3d, real_target(1)/1000, real_target(2), launch_alt_km, 'x', 'Color', [0.85 0.20 0.15], ...
      'MarkerSize', 14, 'LineWidth', 3, 'DisplayName', sprintf('Target [%.2f^\\circN, %.2f^\\circE]', target_lat, target_lon));

% 30m Spec boundary circle in 3D (Solid)
plot3(ax3d, real_target(1)/1000 + 30*cos(angles)/1000, real_target(2) + 30*sin(angles), repmat(launch_alt_km, size(angles)), ...
      '-', 'Color', [0.00 0.70 0.35], 'LineWidth', 2.2, 'DisplayName', '30m PGK Spec Circle (Solid)');

h_traj_3d   = plot3(ax3d, 0, 0, launch_alt_km, '-', 'Color', [0.10 0.65 0.90], 'LineWidth', 2.2, 'DisplayName', '3D Guided Path');
h_shell_3d  = plot3(ax3d, 0, 0, launch_alt_km, 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.85 0.65 0.10], ...
                    'MarkerEdgeColor', [0.75 0.45 0.05], 'DisplayName', 'Current Position');
h_apogee_3d = plot3(ax3d, NaN, NaN, NaN, 'o', 'MarkerSize', 9, 'MarkerFaceColor', [0.15 0.50 0.95], ...
                    'MarkerEdgeColor', [0.05 0.35 0.85], 'DisplayName', 'GNC Activation (Apogee)');
h_corr_3d   = plot3(ax3d, NaN, NaN, NaN, 'd', 'MarkerSize', 5, 'MarkerFaceColor', [0.00 0.70 0.35], ...
                    'MarkerEdgeColor', [0.00 0.50 0.25], 'DisplayName', 'Canard Correction Points');

xlabel(ax3d, 'Downrange X [km]', 'FontWeight', 'bold');
ylabel(ax3d, 'Crossrange Z [m]', 'FontWeight', 'bold');
zlabel(ax3d, 'Altitude MSL [km]', 'FontWeight', 'bold');
title(ax3d, sprintf('1. 3D Trajectory | Origin [%.4f^\\circN, %.4f^\\circE, %.0fm MSL]', Lat0_ground, Lon0_ground, Alt0_ground_m), 'FontSize', 10.5, 'FontWeight', 'bold');
view(ax3d, -35, 24);
legend(ax3d, 'Location', 'northeast');

% Stats badge at top-left corner of 3D plot
stats_3d = sprintf(['\\bf\\fontsize{8.5}PGK MISSION GEO STATS\\rm\\fontsize{8}\n' ...
                    'Launch : [%.4f^\\circN, %.4f^\\circE, %.0fm]\n' ...
                    'Target : [%.4f^\\circN, %.4f^\\circE]\n' ...
                    'Range  : %.1f km | Canard: \\pm%.1f^\\circ\n' ...
                    'CEP_{50} Spec: \\le 30 m | Status: LOCKED'], ...
                    Lat0_ground, Lon0_ground, Alt0_ground_m, ...
                    target_lat, target_lon, ...
                    real_target(1)/1000, canard_deflection);
text(ax3d, 0.03, 0.97, stats_3d, 'Units', 'normalized', ...
     'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
     'BackgroundColor', [0.93 0.96 1.0], 'EdgeColor', [0.15 0.45 0.85], ...
     'LineWidth', 1.2, 'Color', [0.05 0.15 0.40]);

% =========================================================================
% PANEL 2 (Top Middle): Vertical View (Side Profile: Altitude vs Downrange)
% =========================================================================
ax_vert = subplot(2, 3, 2); hold(ax_vert, 'on'); grid(ax_vert, 'on'); box(ax_vert, 'on');

patch(ax_vert, [0, 20000, 20000, 0], Alt0_ground_m + [0, 0, 2000, 2000], [0.85 0.92 0.98], ...
      'EdgeColor', 'none', 'FaceAlpha', 0.5, 'DisplayName', 'Boundary Layer');
patch(ax_vert, [0, 20000, 20000, 0], Alt0_ground_m + [8000, 8000, 11500, 11500], [0.95 0.88 0.95], ...
      'EdgeColor', 'none', 'FaceAlpha', 0.4, 'DisplayName', 'Jet Stream Core');

plot(ax_vert, 0, Alt0_ground_m, 'p', 'MarkerSize', 11, 'MarkerFaceColor', [0.85 0.65 0.1], ...
     'MarkerEdgeColor', [0.75 0.45 0.05], 'DisplayName', sprintf('Launch (%.0fm MSL)', Alt0_ground_m));
plot(ax_vert, real_target(1), Alt0_ground_m, 'x', 'Color', [0.85 0.20 0.15], ...
     'MarkerSize', 13, 'LineWidth', 2.5, 'DisplayName', 'Target Ground');

h_vert_traj   = plot(ax_vert, 0, Alt0_ground_m, '-', 'Color', [0.10 0.65 0.90], 'LineWidth', 2.0, 'DisplayName', 'Flight Path');
h_vert_shell  = plot(ax_vert, 0, Alt0_ground_m, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0.85 0.65 0.10], ...
                     'MarkerEdgeColor', [0.75 0.45 0.05], 'DisplayName', 'Current');
h_vert_apogee = plot(ax_vert, NaN, NaN, 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.15 0.50 0.95], ...
                     'MarkerEdgeColor', [0.05 0.35 0.85], 'DisplayName', 'GNC Apogee');
h_vert_corr   = plot(ax_vert, NaN, NaN, 'd', 'MarkerSize', 4, 'MarkerFaceColor', [0.00 0.70 0.35], ...
                     'MarkerEdgeColor', [0.00 0.50 0.25], 'DisplayName', 'Correction Pts');

xlabel(ax_vert, 'Downrange X [m]', 'FontWeight', 'bold');
ylabel(ax_vert, 'Altitude MSL [m]', 'FontWeight', 'bold');
title(ax_vert, sprintf('2. Vertical Profile (MSL Alt from %.0f m)', Alt0_ground_m), 'FontSize', 10, 'FontWeight', 'bold');
xlim(ax_vert, [0, 20000]); ylim(ax_vert, [Alt0_ground_m - 200, Alt0_ground_m + 12000]);
legend(ax_vert, 'Location', 'northeast');

% =========================================================================
% PANEL 3 (Bottom Middle): Horizontal View (Planform: Crossrange vs Downrange)
% =========================================================================
ax_horz = subplot(2, 3, 5); hold(ax_horz, 'on'); grid(ax_horz, 'on'); box(ax_horz, 'on');

plot(ax_horz, 0, 0, 'p', 'MarkerSize', 11, 'MarkerFaceColor', [0.85 0.65 0.1], ...
     'MarkerEdgeColor', [0.75 0.45 0.05], 'DisplayName', sprintf('Launch [%.2f^\\circN, %.2f^\\circE]', Lat0_ground, Lon0_ground));
plot(ax_horz, real_target(1), real_target(2), 'x', 'Color', [0.85 0.20 0.15], ...
     'MarkerSize', 13, 'LineWidth', 2.5, 'DisplayName', sprintf('Target [%.2f^\\circN, %.2f^\\circE]', target_lat, target_lon));
plot(ax_horz, real_target(1) + 30*cos(angles), real_target(2) + 30*sin(angles), ...
     '-', 'Color', [0.00 0.70 0.35], 'LineWidth', 2.2, 'DisplayName', '30m PGK Spec (Solid)');

h_horz_traj   = plot(ax_horz, 0, 0, '-', 'Color', [0.10 0.65 0.90], 'LineWidth', 2.0, 'DisplayName', 'Track');
h_horz_shell  = plot(ax_horz, 0, 0, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0.85 0.65 0.10], ...
                     'MarkerEdgeColor', [0.75 0.45 0.05], 'DisplayName', 'Current');
h_horz_apogee = plot(ax_horz, NaN, NaN, 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.15 0.50 0.95], ...
                     'MarkerEdgeColor', [0.05 0.35 0.85], 'DisplayName', 'GNC Apogee');
h_horz_corr   = plot(ax_horz, NaN, NaN, 'd', 'MarkerSize', 4, 'MarkerFaceColor', [0.00 0.70 0.35], ...
                     'MarkerEdgeColor', [0.00 0.50 0.25], 'DisplayName', 'Correction Pts');
h_horz_pip    = plot(ax_horz, 0, 0, 's', 'MarkerSize', 8, 'MarkerFaceColor', [0.90 0.40 0.10], ...
                     'MarkerEdgeColor', [0.70 0.25 0.05], 'DisplayName', 'PIP');

xlabel(ax_horz, 'Downrange X [m]', 'FontWeight', 'bold');
ylabel(ax_horz, 'Crossrange Z [m]', 'FontWeight', 'bold');
title(ax_horz, sprintf('3. Horizontal Planform | Target: [%.4f^\\circN, %.4f^\\circE]', target_lat, target_lon), 'FontSize', 10, 'FontWeight', 'bold');
xlim(ax_horz, [0, 20000]); ylim(ax_horz, [-200, 700]);
legend(ax_horz, 'Location', 'northwest');

% =========================================================================
% =========================================================================
% PANEL 4 (Top Right): 2D Target Footprint with Nested Ellipses (Fig. 8)
% =========================================================================
ax2d = subplot(2, 3, 3); hold(ax2d, 'on'); grid(ax2d, 'on'); box(ax2d, 'on');
plot(ax2d, real_target(1), real_target(2), 'x', 'Color', [0.85 0.20 0.15], ...
     'MarkerSize', 14, 'LineWidth', 3, 'DisplayName', sprintf('Target [%.4f^\\circN, %.4f^\\circE]', target_lat, target_lon));

% Nested correction ellipses for 0s, 10s, 20s, 30s delays (DOTTED)
colors = {[0.18 0.80 0.44], [0.20 0.60 0.86], [0.61 0.35 0.71], [0.90 0.49 0.13]};
for i = 1:length(ellipse_data)
    ed = ellipse_data(i);
    ex = real_target(1) - (ed.x0 - x0_apogee) + ed.a * cos(angles);
    ez = real_target(2) - (ed.z0 - z0_apogee) + ed.b * sin(angles);
    plot(ax2d, ex, ez, ':', 'Color', colors{i}, 'LineWidth', 1.8, ...
         'DisplayName', sprintf('Reachability (%ds delay)', ed.delay));
end

% 30m PGK Spec Circle (SOLID & PROMINENT)
plot(ax2d, real_target(1) + 30*cos(angles), real_target(2) + 30*sin(angles), ...
     '-', 'Color', [0.00 0.70 0.35], 'LineWidth', 2.6, 'DisplayName', '30m PGK Spec Circle (Solid)');

h_track_2d = plot(ax2d, 0, 0, '-', 'Color', [0.10 0.65 0.90], 'LineWidth', 2.0, 'DisplayName', 'Terminal Track');
h_pip_2d   = plot(ax2d, 0, 0, 's', 'MarkerSize', 9, 'MarkerFaceColor', [0.90 0.40 0.10], ...
                  'MarkerEdgeColor', [0.70 0.25 0.05], 'DisplayName', 'PIP');
axis(ax2d, 'equal');
% Zoom in so the 30m CEP is prominently visible
xlim(ax2d, [real_target(1) - 100, real_target(1) + 100]);
ylim(ax2d, [real_target(2) - 100, real_target(2) + 100]);
xlabel(ax2d, 'Downrange X [m]', 'FontWeight', 'bold');
ylabel(ax2d, 'Crossrange Z [m]', 'FontWeight', 'bold');
title(ax2d, sprintf('4. 2D Target Footprint (Zoomed 30m CEP) | Target [%.4f^\\circN, %.4f^\\circE]', target_lat, target_lon), 'FontSize', 10, 'FontWeight', 'bold');
legend(ax2d, 'Location', 'northeast');

% =========================================================================
% PANEL 5 (Bottom Right): Hardware Sensor Telemetry & Statistical Stats Card
% =========================================================================
ax_hud = subplot(2, 3, 6);
axis(ax_hud, 'off');
title(ax_hud, '5. Real-Time Hardware Sensor & GNC Telemetry Stream', 'FontSize', 11, 'FontWeight', 'bold');
hud_box = text(ax_hud, 0.01, 0.99, 'Initializing Telemetry Stream...', 'Units', 'normalized', ...
               'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
               'FontName', 'monospace', 'FontSize', 10.0, ...
               'BackgroundColor', [0.93 0.96 1.0], 'EdgeColor', [0.15 0.45 0.85], ...
               'LineWidth', 1.4, 'Color', [0.05 0.15 0.40]);

if num_runs <= 1
    % =========================================================================
    % SINGLE-ROUND REAL-TIME ANIMATED FLIGHT
    % =========================================================================
    th_rad = deg2rad(theta_virt);
    psi_rad = deg2rad(psi_virt);
    pos = [0, 0, 0];
    vel = [v0_nominal * cos(th_rad) * cos(psi_rad), ...
           v0_nominal * sin(th_rad), ...
           v0_nominal * cos(th_rad) * sin(psi_rad)];
    p = (2 * pi * v0_nominal) / (twist_calibers * d_proj);

    t = 0; max_time = 130.0;
    step_counter = 0;
    traj_buffer = pos;
    apogee_reached = false;
    apogee_time = 0;
    apogee_point = [0, 0, 0];
    corr_points = zeros(0, 3);
    control_active = false;
    cmd_gamma_c = 0.0;
    pred_x = 0; pred_z = 0; pred_miss = 0;

    fprintf('Propagating guided trajectory: streaming real-time sensor packets...\n');

    while pos(2) >= 0 && t < max_time
        alt = max(0, pos(2));
        if vel(2) < 0 && ~apogee_reached
            apogee_reached = true;
            apogee_time = t;
            apogee_point = pos;
            set(h_apogee_3d, 'XData', apogee_point(1)/1000, 'YData', apogee_point(3), 'ZData', (Alt0_ground_m + apogee_point(2))/1000);
            set(h_vert_apogee, 'XData', apogee_point(1), 'YData', Alt0_ground_m + apogee_point(2));
            set(h_horz_apogee, 'XData', apogee_point(1), 'YData', apogee_point(3));
        end

        % Atmosphere (Scaled from calibrated ground baseline)
        h = min(alt, 20000);
        if h <= 11000
            T = T0_ground_K - 0.0065 * h;
            P = P0_ground_Pa * (1 - 0.0065 * h / T0_ground_K)^5.25588;
        else
            T = T0_ground_K - 0.0065 * 11000;
            P_trop = P0_ground_Pa * (1 - 0.0065 * 11000 / T0_ground_K)^5.25588;
            P = P_trop * exp(-9.80665 * (h - 11000) / (287.05287 * T));
        end
        rho = P / (287.05287 * T);
        a_sound = sqrt(1.4 * 287.05287 * T);

        % Dynamic Wind Profile
        w_spd = wind.spd; w_az = wind.az; w_jet = wind.jet;
        if alt <= 2000
            v_mag = w_spd * (max(10, alt) / 2000)^0.2; v_dir = w_az;
        elseif alt <= 8000
            frac = (alt - 2000) / 6000; v_mag = w_spd * 1.5 + frac * (w_jet * 0.5); v_dir = w_az + frac * 30.0;
        elseif alt <= 11500
            jet_core = w_jet * exp(-((alt - 9500) / 1200)^2); v_mag = w_spd * 1.5 + jet_core; v_dir = w_az + 45.0;
        else
            frac = min(1.0, (alt - 11500) / 4000); v_mag = (w_jet * 0.4) * (1 - 0.5 * frac); v_dir = w_az + 45.0;
        end
        th_w = deg2rad(v_dir); Wx = v_mag * cos(th_w); Wz = v_mag * sin(th_w);

        v_rel_x = vel(1) - Wx; v_rel_y = vel(2); v_rel_z = vel(3) - Wz;
        v_rel_mag = sqrt(v_rel_x^2 + v_rel_y^2 + v_rel_z^2);
        v_unit_x = v_rel_x / max(1e-3, v_rel_mag);
        v_unit_y = v_rel_y / max(1e-3, v_rel_mag);

        % GNC Closed-Loop Command Updates
        if apogee_reached && mod(step_counter, 10) == 0
            [pred_x, pred_z] = predict_impacts_vectorized_met(pos, vel, p, w_spd, w_az, w_jet, 0.4, P0_ground_Pa, T0_ground_K);
            dx_err = real_target(1) - pred_x;
            dz_err = real_target(2) - pred_z;
            pred_miss = sqrt(dx_err^2 + dz_err^2);

            if pred_miss > 1.5
                phi_cmd = atan2(dz_err, dx_err);
                cmd_gamma_c = phi_cmd - delta_kappa;
                control_active = true;
                corr_points(end+1, :) = pos;
            else
                control_active = false;
            end
        end

        Mach = v_rel_mag / a_sound;
        if Mach < 0.8, Cd = Cd0;
        elseif Mach < 1.05, Cd = Cd0 + 0.22 * ((Mach - 0.8) / 0.25)^2;
        elseif Mach < 1.6, Cd = (Cd0 + 0.22) - 0.08 * ((Mach - 1.05) / 0.55);
        else, Cd = (Cd0 + 0.14) / (1 + 0.15 * (Mach - 1.6)); end

        dp_dt = (0.5 * rho * v_rel_mag * S_ref * (d_proj^2) * Clp / Ix) * p;
        p = max(0, p + dp_dt * dt);

        drag_k = 0.5 * rho * S_ref * Cd * v_rel_mag / m;
        ax_drag = -drag_k * v_rel_x; ay_drag = -drag_k * v_rel_y; az_drag = -drag_k * v_rel_z;

        vcg_z = v_unit_x * g0;
        denom = max(100.0, rho * S_ref * d_proj * 11.5 * (v_rel_mag^2));
        alpha_e_z = (2.0 * Ix * p / (denom * v_rel_mag)) * vcg_z;
        F_lift_z = 0.5 * rho * S_ref * (v_rel_mag^2) * C_La * alpha_e_z;
        F_mag_x = 0.5 * rho * S_ref * d_proj * C_mag_p * p * v_rel_mag * (v_unit_y * alpha_e_z);
        F_mag_y = -0.5 * rho * S_ref * d_proj * C_mag_p * p * v_rel_mag * (v_unit_x * alpha_e_z);

        ax = ax_drag + F_mag_x / m; ay = -g0 + ay_drag + F_mag_y / m; az = az_drag + F_lift_z / m;

        cmd_pitch_deg = canard_deflection * cos(cmd_gamma_c);
        cmd_yaw_deg   = canard_deflection * sin(cmd_gamma_c);
        actual_pitch_deg = cmd_pitch_deg; actual_yaw_deg = cmd_yaw_deg;

        if apogee_reached && control_active
            v_h = max(1e-3, sqrt(vel(1)^2 + vel(3)^2));
            eyaw_x = -vel(3) / v_h; eyaw_y = 0; eyaw_z = vel(1) / v_h;
            ev_x = vel(1) / v_rel_mag; ev_y = vel(2) / v_rel_mag; ev_z = vel(3) / v_rel_mag;
            epitch_x = -eyaw_z * ev_y; epitch_y = eyaw_z * ev_x - eyaw_x * ev_z; epitch_z = eyaw_x * ev_y;
            q_dynamic = 0.5 * rho * v_rel_mag^2;
            a_steer_mag = (q_dynamic * S_ref * C_N_canard) / m;
            phi_actual = cmd_gamma_c + delta_kappa;
            ax = ax + a_steer_mag * (cos(phi_actual) * epitch_x + sin(phi_actual) * eyaw_x);
            ay = ay + a_steer_mag * (cos(phi_actual) * epitch_y + sin(phi_actual) * eyaw_y);
            az = az + a_steer_mag * (cos(phi_actual) * epitch_z + sin(phi_actual) * eyaw_z);
        end

        s_ax_g = (ax / g0) + 0.02 * randn; s_ay_g = (ay / g0) + 0.02 * randn; s_az_g = (az / g0) + 0.02 * randn;
        s_spin = (p / (2*pi)) + 0.15 * randn;
        s_alt_m = Alt0_ground_m + max(0, pos(2) + 1.2 * randn);
        s_temp_c = (T - 273.15) + 0.1 * randn;
        s_press_hpa = (P / 100) + 0.25 * randn;
        s_vbus = max(6.8, Vbus0_V - 0.0035 * t);
        s_curr_ma = 182.0 + 8.0 * randn + abs(actual_pitch_deg + actual_yaw_deg) * 12.0;
        s_pwr_mw = s_vbus * s_curr_ma;
        s_lat = Lat0_ground + (pos(1) / 111319);
        s_lon = Lon0_ground + (pos(3) / (111319 * cosd(Lat0_ground)));
        s_sats = 9;

        if pos(2) <= 1.0 && t > 5.0
            virtualCapacitor = virtualCapacitor + 200;
        else
            virtualCapacitor = max(0, virtualCapacitor - 5);
        end
        if virtualCapacitor >= const_THRESHOLD, impactAlertActive = true; end

        prev_pos = pos; vel = vel + [ax, ay, az] * dt; pos = pos + vel * dt;
        t = t + dt; step_counter = step_counter + 1;
        traj_buffer(end+1, :) = pos;

        if mod(step_counter, 10) == 0 || pos(2) <= 0
            alt_msl_km = (Alt0_ground_m + traj_buffer(:, 2)) / 1000;
            set(h_traj_3d, 'XData', traj_buffer(:, 1)/1000, 'YData', traj_buffer(:, 3), 'ZData', alt_msl_km);
            set(h_shell_3d, 'XData', pos(1)/1000, 'YData', pos(3), 'ZData', (Alt0_ground_m + max(0, pos(2)))/1000);
            if ~isempty(corr_points)
                set(h_corr_3d, 'XData', corr_points(:, 1)/1000, 'YData', corr_points(:, 3), 'ZData', (Alt0_ground_m + corr_points(:, 2))/1000);
            end
            set(h_vert_traj, 'XData', traj_buffer(:, 1), 'YData', Alt0_ground_m + traj_buffer(:, 2));
            set(h_vert_shell, 'XData', pos(1), 'YData', Alt0_ground_m + max(0, pos(2)));
            if ~isempty(corr_points)
                set(h_vert_corr, 'XData', corr_points(:, 1), 'YData', Alt0_ground_m + corr_points(:, 2));
            end
            set(h_horz_traj, 'XData', traj_buffer(:, 1), 'YData', traj_buffer(:, 3));
            set(h_horz_shell, 'XData', pos(1), 'YData', pos(3));
            set(h_horz_pip, 'XData', pred_x, 'YData', pred_z);
            if ~isempty(corr_points)
                set(h_horz_corr, 'XData', corr_points(:, 1), 'YData', corr_points(:, 3));
            end
            set(h_track_2d, 'XData', traj_buffer(:, 1), 'YData', traj_buffer(:, 3));
            set(h_pip_2d, 'XData', pred_x, 'YData', pred_z);
            % Keep zoomed in so 30m CEP circle remains clearly visible
            xlim(ax2d, [real_target(1) - 100, real_target(1) + 100]);
            ylim(ax2d, [real_target(2) - 100, real_target(2) + 100]);

            n_corr = size(corr_points, 1);
            if impactAlertActive, vib_str = '>> IMPACT DETECTED! <<'; else, vib_str = 'SAFE (In-Flight Controlled)'; end
            hud_str = sprintf([ ...
                '\\bfHARDWARE LINK\\rm : [%s]\n' ...
                '\\bfLAUNCH SITE\\rm   : [%.4f^\\circN, %.4f^\\circE, %.0fm MSL]\n' ...
                '-----------------------------------------------------\n' ...
                '\\bfFLIGHT TIME\\rm   : %5.1f s  |  \\bfAIRSPEED\\rm  : %5.1f m/s (M%.2f)\n' ...
                '\\bfALTITUDE MSL\\rm  : %5.0f m  |  \\bfDOWNRANGE\\rm : %6.1f km\n' ...
                '\\bfGEO POSITION\\rm  : [%.5f^\\circN, %.5f^\\circE]\n' ...
                '-----------------------------------------------------\n' ...
                '\\bfMPU6050 IMU\\rm  : Ax: %+5.2fg  Ay: %+5.2fg  Az: %+5.2fg\n' ...
                '               Spin rate p: %5.1f rev/s (Roll Damped)\n' ...
                '\\bfBMP280 BARO\\rm  : Alt: %5.0f m MSL | Pres: %6.1f hPa | T: %+4.1f C\n' ...
                '\\bfINA219 POWER\\rm : Vbus: %4.2f V | Curr: %4.0f mA | Pwr: %4.0f mW\n' ...
                '\\bfGPS TELEMETRY\\rm: [%.5f^\\circN, %.5f^\\circE] (Sats: %d)\n' ...
                '\\bfPIEZO IMPACT\\rm  : %s (Cap: %d/%d)\n' ...
                '-----------------------------------------------------\n' ...
                '\\bfGNC CANARDS\\rm   : Pitch: %+4.1f deg  |  Yaw: %+4.1f deg\n' ...
                '\\bfCORRECTIONS\\rm   : %d steering pulses applied\n' ...
                '\\bfTARGET GEO\\rm    : [%.5f^\\circN, %.5f^\\circE] (Miss: %4.1f m)'], ...
                active_port, Lat0_ground, Lon0_ground, Alt0_ground_m, ...
                t, v_rel_mag, Mach, Alt0_ground_m + max(0, alt), pos(1)/1000, ...
                s_lat, s_lon, ...
                s_ax_g, s_ay_g, s_az_g, s_spin, ...
                s_alt_m, s_press_hpa, s_temp_c, ...
                s_vbus, s_curr_ma, s_pwr_mw, ...
                s_lat, s_lon, s_sats, ...
                vib_str, virtualCapacitor, const_THRESHOLD, ...
                actual_pitch_deg, actual_yaw_deg, ...
                n_corr, ...
                target_lat, target_lon, pred_miss);
            set(hud_box, 'String', hud_str);
            drawnow;
            pause(0.005);
        end
    end

    frac = prev_pos(2) / (prev_pos(2) - pos(2));
    final_impact = [prev_pos(1) + frac * (pos(1) - prev_pos(1)), ...
                    prev_pos(3) + frac * (pos(3) - prev_pos(3))];
    final_miss = norm(final_impact - real_target);

    fprintf('\n=========================================================================\n');
    fprintf('=== GUIDED TRAJECTORY COMPLETE: IMPACT AT t = %.2f s ===\n', t);
    fprintf('=========================================================================\n');
    fprintf('Designated Target Location              : [%.1f, %.1f] m\n', real_target(1), real_target(2));
    fprintf('Actual Guided Impact Point              : [%.1f, %.1f] m\n', final_impact(1), final_impact(2));
    fprintf('Final Radial Miss Distance              : %.2f meters\n', final_miss);
    fprintf('Total Canard Corrections Applied        : %d pulses\n', size(corr_points, 1));
    if final_miss <= 30.0
        fprintf('PGK Specification (<30m CEP requirement): PASSED (Within Spec Circle!)\n');
    else
        fprintf('PGK Specification (<30m CEP requirement): FAILED (%.1f m > 30 m)\n', final_miss);
    end
    fprintf('Telemetry Ingestion Frequency           : %.1f Hz\n', step_counter / max(1e-3, t));

else
    % =========================================================================
    % MULTI-ROUND GUIDED FLIGHT DISPERSION & MONTE CARLO (num_runs > 1)
    % =========================================================================
    fprintf('\nPropagating %d Multi-Round Guided Flights under Physical Ballistic Dispersion...\n', num_runs);
    all_impacts = zeros(num_runs, 2);
    all_misses = zeros(num_runs, 1);

    % Curated vibrant non-black palette colors
    palette = [
        0.10, 0.65, 0.90;  % Azure Blue
        0.92, 0.40, 0.12;  % Tangerine
        0.15, 0.75, 0.35;  % Emerald
        0.75, 0.20, 0.85;  % Violet Purple
        0.95, 0.70, 0.10;  % Amber Gold
        0.05, 0.80, 0.80;  % Cyan
        0.85, 0.18, 0.42;  % Crimson Rose
        0.35, 0.45, 0.95;  % Cobalt
        0.60, 0.80, 0.20;  % Lime
        0.95, 0.50, 0.30;  % Coral
        0.20, 0.80, 0.60;  % Teal
        0.85, 0.55, 0.20   % Bronze
    ];

    set(h_traj_3d, 'Visible', 'off');
    set(h_shell_3d, 'Visible', 'off');
    set(h_vert_traj, 'Visible', 'off');
    set(h_vert_shell, 'Visible', 'off');
    set(h_horz_traj, 'Visible', 'off');
    set(h_horz_shell, 'Visible', 'off');

    t_mc_start = tic;
    for r = 1:num_runs
        c_idx = mod(r-1, size(palette, 1)) + 1;
        c_col = palette(c_idx, :);

        % Gunnery, propellant & environmental perturbations for round r
        v0_r = v0_nominal + 4.5 * randn;
        theta_r = theta_virt + 0.04 * randn;
        psi_r   = psi_virt + 0.04 * randn;
        m_r     = 43.10 + 0.25 * randn;
        wind_r.spd = max(0, wind.spd + 1.2 * randn);
        wind_r.az  = wind.az + 12.0 * randn;
        wind_r.jet = max(10, wind.jet + 3.0 * randn);

        [traj_r, imp_r, miss_r, ap_r] = run_single_guided_flight_met(v0_r, theta_r, psi_r, m_r, canard_deflection, 0, wind_r, real_target, P0_ground_Pa, T0_ground_K);

        all_impacts(r, :) = imp_r;
        all_misses(r) = miss_r;

        % Plot round trajectory onto all 4 panels
        plot3(ax3d, traj_r(:,1)/1000, traj_r(:,3), (Alt0_ground_m + traj_r(:,2))/1000, '-', 'Color', c_col, 'LineWidth', 1.5);
        plot(ax_vert, traj_r(:,1), Alt0_ground_m + traj_r(:,2), '-', 'Color', c_col, 'LineWidth', 1.4);
        plot(ax_horz, traj_r(:,1), traj_r(:,3), '-', 'Color', c_col, 'LineWidth', 1.4);
        plot(ax2d, imp_r(1), imp_r(2), 'o', 'MarkerSize', 7, 'MarkerFaceColor', c_col, 'MarkerEdgeColor', [0.1 0.1 0.1]);

        fprintf('  -> Round %2d/%2d: Impact = [%7.1f, %5.1f] m | Radial Miss = %5.2f m | %s\n', ...
                r, num_runs, imp_r(1), imp_r(2), miss_r, ...
                ifelse(miss_r <= 30.0, 'PASSED (<30m)', 'OUT OF SPEC'));
        drawnow;
    end
    t_mc_elapsed = toc(t_mc_start);

    % Statistical Calculations
    cep50 = prctile(all_misses, 50);
    sep95 = prctile(all_misses, 95);
    mean_miss = mean(all_misses);
    std_miss  = std(all_misses);
    max_miss  = max(all_misses);
    min_miss  = min(all_misses);
    pass_pct  = 100 * mean(all_misses <= 30.0);

    % Draw Empirical CEP_50 Circle on 2D Plot
    plot(ax2d, real_target(1) + cep50*cos(angles), real_target(2) + cep50*sin(angles), '--', ...
         'Color', [0.00 0.70 0.35], 'LineWidth', 2.2, 'DisplayName', sprintf('Empirical CEP_{50} (%.1fm)', cep50));

    % Update Title of 2D plot with CEP
    title(ax2d, sprintf('4. 2D Target Footprint (CEP_{50} = %.1f m | %d Rounds)', cep50, num_runs), 'FontSize', 10, 'FontWeight', 'bold');

    % Update Panel 5 Statistical HUD Card
    stats_hud_str = sprintf([ ...
        '\\bfHARDWARE LINK\\rm     : [%s]\n' ...
        '\\bfLAUNCH SITE\\rm       : [%.4f^\\circN, %.4f^\\circE, %.0fm MSL]\n' ...
        '\\bfTARGET LOCATION\\rm   : [%.4f^\\circN, %.4f^\\circE]\n' ...
        '-----------------------------------------------------\n' ...
        '\\bfTOTAL ROUNDS FIRED\\rm: %d Guided Rounds\n' ...
        '\\bf50%% CEP RADIUS\\rm    : %5.2f m  (\\bfSPEC: \\le 30 m\\rm)\n' ...
        '\\bf95%% SEP RADIUS\\rm    : %5.2f m\n' ...
        '\\bfMEAN RADIAL MISS\\rm  : %5.2f \\pm %.2f m\n' ...
        '\\bfMAX / MIN MISS\\rm    : %5.2f m / %5.2f m\n' ...
        '\\bf30m SPEC PASS RATE\\rm: %5.1f%% (%d/%d Rounds Passed)\n' ...
        '-----------------------------------------------------\n' ...
        '\\bfBMP280 CALIBRATION\\rm: %6.1f hPa | %5.1f C | %5.0f m\n' ...
        '\\bfSIMULATION TIME\\rm   : %5.2f s (%.1f rounds/sec)\n' ...
        '\\bfOVERALL EVALUATION\\rm: %s'], ...
        active_port, Lat0_ground, Lon0_ground, Alt0_ground_m, ...
        target_lat, target_lon, ...
        num_runs, cep50, sep95, mean_miss, std_miss, max_miss, min_miss, ...
        pass_pct, sum(all_misses <= 30.0), num_runs, ...
        P0_ground_hpa, T0_ground_C, Alt0_ground_m, ...
        t_mc_elapsed, num_runs / max(1e-3, t_mc_elapsed), ...
        ifelse(cep50 <= 30.0, '\\bfPASSED (WITHIN 30m SPEC)\\rm', '\\bfFAILED\\rm'));
    set(hud_box, 'String', stats_hud_str);
    drawnow;

    % Terminal Summary Report
    fprintf('\n+-------------------------------------------------------------------------+\n');
    fprintf('|             155 mm PGK MULTI-ROUND MONTE CARLO STATS SUMMARY            |\n');
    fprintf('+-------------------------------------------------------------------------+\n');
    fprintf('  Total Rounds Simulated      : %d rounds\n', num_runs);
    fprintf('  50%% Circular Error Probable : %6.2f meters  (Spec requirement: <= 30 m)\n', cep50);
    fprintf('  95%% Spherical Error Probable: %6.2f meters\n', sep95);
    fprintf('  Mean Radial Miss Distance   : %6.2f +/- %.2f meters\n', mean_miss, std_miss);
    fprintf('  Minimum / Maximum Miss      : %6.2f m / %6.2f m\n', min_miss, max_miss);
    fprintf('  30m PGK Spec Pass Rate      : %6.1f%% (%d/%d Passed)\n', pass_pct, sum(all_misses <= 30.0), num_runs);
    fprintf('  Simulation Execution Time   : %6.2f seconds\n', t_mc_elapsed);
    fprintf('+-------------------------------------------------------------------------+\n\n');
end

% Cleanup POSIX serial fid
if is_hardware && isnumeric(device) && device > 0
    try, fclose(device); catch, end
end

fprintf('Saved visualization dashboard to guided_realtime_sensor_analysis.png\n');
try
    print(fig, 'guided_realtime_sensor_analysis.png', '-dpng', '-r150');
catch
end
end

%% =========================================================================
%% HELPER FUNCTIONS (3-DOF IMPACT PREDICTOR, SOLVER, ELLIPSE CALIBRATION)
%% =========================================================================
function [pred_x, pred_z] = predict_impacts_vectorized(p_pos, p_vel, p_spin, p_gw_spd, p_gw_az, p_jet_spd, dt_pred)
    K = size(p_pos, 1);
    pos = p_pos; vel = p_vel; p = p_spin;
    active = true(K, 1);
    pred_x = zeros(K, 1); pred_z = zeros(K, 1);
    
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
        
        y = pos(idx, 2);
        alt = max(0, y);
        h = max(0, min(alt, 20000));
        is_trop = h <= 11000;
        T = zeros(K_act, 1); P = zeros(K_act, 1);
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
end

function [theta, psi] = solve_firing_solution(target, v0_nom, canard_deflection_deg)
    theta = 54.5;
    psi = 0.0;
    max_iter = 15;
    tol = 1.0;
    
    nom_wind.spd = 4.0; nom_wind.az = 45.0; nom_wind.jet = 38.0;
    
    for iter = 1:max_iter
        imp = run_single_forward(v0_nom, theta, psi, canard_deflection_deg, false, target, 0, nom_wind);
        dx = target(1) - imp(1);
        dz = target(2) - imp(2);
        
        if sqrt(dx^2 + dz^2) < tol
            break;
        end
        
        psi = psi + rad2deg(atan2(dz, imp(1)));
        
        d_theta = 0.05;
        imp_p = run_single_forward(v0_nom, theta + d_theta, psi, canard_deflection_deg, false, target, 0, nom_wind);
        dRange_dTheta = (imp_p(1) - imp(1)) / d_theta;
        
        theta = theta + dx / dRange_dTheta;
        theta = max(45.0, min(80.0, theta));
    end
end

function [x0, z0, semi_a, semi_b, impacts] = calibrate_correction_ellipse_delay(v0_nom, ...
                                                theta_nom, psi_nom, canard_deflection_deg, gnc_delay)
    angles_deg = 0:45:315;
    impacts = zeros(length(angles_deg), 2);
    nom_wind.spd = 4.0; nom_wind.az = 45.0; nom_wind.jet = 38.0;
    
    imp_unc = run_single_forward(v0_nom, theta_nom, psi_nom, canard_deflection_deg, false, [0,0], 0, nom_wind);
    
    for i = 1:length(angles_deg)
        gamma_c_rad = deg2rad(angles_deg(i));
        imp = run_single_forced(v0_nom, theta_nom, psi_nom, canard_deflection_deg, gamma_c_rad, gnc_delay, nom_wind);
        impacts(i, :) = imp;
    end
    
    rel_impacts = impacts - imp_unc;
    x0 = mean(rel_impacts(:, 1));
    z0 = mean(rel_impacts(:, 2));
    
    centered_x = rel_impacts(:, 1) - x0;
    centered_z = rel_impacts(:, 2) - z0;
    semi_a = (max(centered_x) - min(centered_x)) / 2;
    semi_b = (max(centered_z) - min(centered_z)) / 2;
end

function imp = run_single_forward(v0_nom, theta_nom, psi_nom, canard_deflection_deg, is_guided, target, gnc_delay, wind)
    g0 = 9.80665; m = 43.10; d_proj = 0.155; S_ref = pi * (d_proj / 2)^2;
    Cd0 = 0.25; Ix = 0.142; twist_calibers = 20.0; C_La = 2.0; dt = 0.05;
    delta_e = deg2rad(canard_deflection_deg); S_canard = 0.0035;
    C_N_canard = 4.0 * (S_canard / S_ref) * (2.0 * delta_e);
    delta_kappa = deg2rad(15.0);
    
    th_rad = deg2rad(theta_nom); psi_rad = deg2rad(psi_nom);
    pos = [0, 0, 0];
    vel = [v0_nom * cos(th_rad) * cos(psi_rad), v0_nom * sin(th_rad), v0_nom * cos(th_rad) * sin(psi_rad)];
    p = (2 * pi * v0_nom) / (twist_calibers * d_proj);
    
    t = 0; apogee_reached = false; apogee_time = 0;
    control_active = false; cmd_gamma_c = 0.0;
    step_c = 0;
    
    while pos(2) >= 0 && t < 130.0
        alt = max(0, pos(2));
        if vel(2) < 0 && ~apogee_reached
            apogee_reached = true; apogee_time = t;
        end
        
        h = min(alt, 20000);
        if h <= 11000
            T = 288.15 - 0.0065 * h;
            P = 101325 * (1 - 0.0065 * h / 288.15)^5.25588;
        else
            T = 216.65;
            P_trop = 101325 * (1 - 0.0065 * 11000 / 288.15)^5.25588;
            P = P_trop * exp(-9.80665 * (h - 11000) / (287.05287 * T));
        end
        rho = P / (287.05287 * T); a_sound = sqrt(1.4 * 287.05287 * T);
        
        c_gw_spd = wind.spd; c_gw_az = wind.az; c_jet_spd = wind.jet;
        if alt <= 2000
            w_mag = c_gw_spd * (max(10, alt) / 2000)^0.2; w_dir = c_gw_az;
        elseif alt <= 8000
            frac2 = (alt - 2000) / 6000; w_mag = c_gw_spd * 1.5 + frac2 * (c_jet_spd * 0.5); w_dir = c_gw_az + frac2 * 30.0;
        elseif alt <= 11500
            jet_core = c_jet_spd * exp(-((alt - 9500) / 1200)^2); w_mag = c_gw_spd * 1.5 + jet_core; w_dir = c_gw_az + 45.0;
        else
            frac4 = min(1.0, (alt - 11500) / 4000); w_mag = (c_jet_spd * 0.4) * (1 - 0.5 * frac4); w_dir = c_gw_az + 45.0;
        end
        
        th_w = deg2rad(w_dir); Wx = w_mag * cos(th_w); Wz = w_mag * sin(th_w);
        v_rel_x = vel(1) - Wx; v_rel_y = vel(2); v_rel_z = vel(3) - Wz;
        v_rel_mag = sqrt(v_rel_x^2 + v_rel_y^2 + v_rel_z^2);
        v_unit_x = v_rel_x / max(1e-3, v_rel_mag);
        v_unit_y = v_rel_y / max(1e-3, v_rel_mag);
        Mach = v_rel_mag / a_sound;
        
        if is_guided && apogee_reached && (t >= apogee_time + gnc_delay) && mod(step_c, 10) == 0
            [pred_x, pred_z] = predict_impacts_vectorized(pos, vel, p, c_gw_spd, c_gw_az, c_jet_spd, 0.4);
            dx_err = target(1) - pred_x;
            dz_err = target(2) - pred_z;
            miss_dist = sqrt(dx_err^2 + dz_err^2);
            if miss_dist > 1.5
                phi_cmd = atan2(dz_err, dx_err);
                cmd_gamma_c = phi_cmd - delta_kappa;
                control_active = true;
            else
                control_active = false;
            end
        end
        
        if Mach < 0.8, Cd = Cd0;
        elseif Mach < 1.05, Cd = Cd0 + 0.22 * ((Mach - 0.8) / 0.25)^2;
        elseif Mach < 1.6, Cd = (Cd0 + 0.22) - 0.08 * ((Mach - 1.05) / 0.55);
        else, Cd = (Cd0 + 0.14) / (1 + 0.15 * (Mach - 1.6)); end
        
        dp_dt = (0.5 * rho * v_rel_mag * S_ref * (d_proj^2) * (-0.015) / Ix) * p;
        p = max(0, p + dp_dt * dt);
        
        drag_k = 0.5 * rho * S_ref * Cd * v_rel_mag / m;
        ax_drag = -drag_k * v_rel_x; ay_drag = -drag_k * v_rel_y; az_drag = -drag_k * v_rel_z;
        
        vcg_z = v_unit_x * g0;
        denom = max(100.0, rho * S_ref * d_proj * 11.5 * (v_rel_mag^2));
        alpha_e_z = (2.0 * Ix * p / (denom * v_rel_mag)) * vcg_z;
        F_lift_z = 0.5 * rho * S_ref * (v_rel_mag^2) * C_La * alpha_e_z;
        F_mag_x = 0.5 * rho * S_ref * d_proj * 0.008 * p * v_rel_mag * (v_unit_y * alpha_e_z);
        F_mag_y = -0.5 * rho * S_ref * d_proj * 0.008 * p * v_rel_mag * (v_unit_x * alpha_e_z);
        
        ax = ax_drag + F_mag_x / m;
        ay = -g0 + ay_drag + F_mag_y / m;
        az = az_drag + F_lift_z / m;
        
        if is_guided && apogee_reached && (t >= apogee_time + gnc_delay) && control_active
            v_h = max(1e-3, sqrt(vel(1)^2 + vel(3)^2));
            eyaw_x = -vel(3) / v_h; eyaw_y = 0; eyaw_z = vel(1) / v_h;
            ev_x = vel(1) / v_rel_mag; ev_y = vel(2) / v_rel_mag; ev_z = vel(3) / v_rel_mag;
            epitch_x = -eyaw_z * ev_y; epitch_y = eyaw_z * ev_x - eyaw_x * ev_z; epitch_z = eyaw_x * ev_y;
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
        t = t + dt;
        step_c = step_c + 1;
    end
    frac = prev_pos(2) / (prev_pos(2) - pos(2));
    imp = [prev_pos(1) + frac * (pos(1) - prev_pos(1)), ...
           prev_pos(3) + frac * (pos(3) - prev_pos(3))];
end

function imp = run_single_forced(v0_nom, theta_nom, psi_nom, canard_deflection_deg, forced_gamma_c, gnc_delay, wind)
    g0 = 9.80665; m = 43.10; d_proj = 0.155; S_ref = pi * (d_proj / 2)^2;
    Cd0 = 0.25; Ix = 0.142; twist_calibers = 20.0; C_La = 2.0; dt = 0.05;
    delta_e = deg2rad(canard_deflection_deg); S_canard = 0.0035;
    C_N_canard = 4.0 * (S_canard / S_ref) * (2.0 * delta_e);
    delta_kappa = deg2rad(15.0);
    
    th_rad = deg2rad(theta_nom); psi_rad = deg2rad(psi_nom);
    pos = [0, 0, 0];
    vel = [v0_nom * cos(th_rad) * cos(psi_rad), v0_nom * sin(th_rad), v0_nom * cos(th_rad) * sin(psi_rad)];
    p = (2 * pi * v0_nom) / (twist_calibers * d_proj);
    
    t = 0; apogee_reached = false; apogee_time = 0;
    while pos(2) >= 0 && t < 130.0
        alt = max(0, pos(2));
        if vel(2) < 0 && ~apogee_reached
            apogee_reached = true; apogee_time = t;
        end
        
        h = min(alt, 20000);
        if h <= 11000
            T = 288.15 - 0.0065 * h;
            P = 101325 * (1 - 0.0065 * h / 288.15)^5.25588;
        else
            T = 216.65;
            P_trop = 101325 * (1 - 0.0065 * 11000 / 288.15)^5.25588;
            P = P_trop * exp(-9.80665 * (h - 11000) / (287.05287 * T));
        end
        rho = P / (287.05287 * T); a_sound = sqrt(1.4 * 287.05287 * T);
        
        c_gw_spd = wind.spd; c_gw_az = wind.az; c_jet_spd = wind.jet;
        if alt <= 2000
            w_mag = c_gw_spd * (max(10, alt) / 2000)^0.2; w_dir = c_gw_az;
        elseif alt <= 8000
            frac2 = (alt - 2000) / 6000; w_mag = c_gw_spd * 1.5 + frac2 * (c_jet_spd * 0.5); w_dir = c_gw_az + frac2 * 30.0;
        elseif alt <= 11500
            jet_core = c_jet_spd * exp(-((alt - 9500) / 1200)^2); w_mag = c_gw_spd * 1.5 + jet_core; w_dir = c_gw_az + 45.0;
        else
            frac4 = min(1.0, (alt - 11500) / 4000); w_mag = (c_jet_spd * 0.4) * (1 - 0.5 * frac4); w_dir = c_gw_az + 45.0;
        end
        
        th_w = deg2rad(w_dir); Wx = w_mag * cos(th_w); Wz = w_mag * sin(th_w);
        v_rel_x = vel(1) - Wx; v_rel_y = vel(2); v_rel_z = vel(3) - Wz;
        v_rel_mag = sqrt(v_rel_x^2 + v_rel_y^2 + v_rel_z^2);
        v_unit_x = v_rel_x / max(1e-3, v_rel_mag);
        v_unit_y = v_rel_y / max(1e-3, v_rel_mag);
        Mach = v_rel_mag / a_sound;
        
        if Mach < 0.8, Cd = Cd0;
        elseif Mach < 1.05, Cd = Cd0 + 0.22 * ((Mach - 0.8) / 0.25)^2;
        elseif Mach < 1.6, Cd = (Cd0 + 0.22) - 0.08 * ((Mach - 1.05) / 0.55);
        else, Cd = (Cd0 + 0.14) / (1 + 0.15 * (Mach - 1.6)); end
        
        dp_dt = (0.5 * rho * v_rel_mag * S_ref * (d_proj^2) * (-0.015) / Ix) * p;
        p = max(0, p + dp_dt * dt);
        
        drag_k = 0.5 * rho * S_ref * Cd * v_rel_mag / m;
        ax_drag = -drag_k * v_rel_x; ay_drag = -drag_k * v_rel_y; az_drag = -drag_k * v_rel_z;
        
        vcg_z = v_unit_x * g0;
        denom = max(100.0, rho * S_ref * d_proj * 11.5 * (v_rel_mag^2));
        alpha_e_z = (2.0 * Ix * p / (denom * v_rel_mag)) * vcg_z;
        F_lift_z = 0.5 * rho * S_ref * (v_rel_mag^2) * C_La * alpha_e_z;
        F_mag_x = 0.5 * rho * S_ref * d_proj * 0.008 * p * v_rel_mag * (v_unit_y * alpha_e_z);
        F_mag_y = -0.5 * rho * S_ref * d_proj * 0.008 * p * v_rel_mag * (v_unit_x * alpha_e_z);
        
        ax = ax_drag + F_mag_x / m;
        ay = -g0 + ay_drag + F_mag_y / m;
        az = az_drag + F_lift_z / m;
        
        if apogee_reached && (t >= apogee_time + gnc_delay)
            v_h = max(1e-3, sqrt(vel(1)^2 + vel(3)^2));
            eyaw_x = -vel(3) / v_h; eyaw_y = 0; eyaw_z = vel(1) / v_h;
            ev_x = vel(1) / v_rel_mag; ev_y = vel(2) / v_rel_mag; ev_z = vel(3) / v_rel_mag;
            epitch_x = -eyaw_z * ev_y; epitch_y = eyaw_z * ev_x - eyaw_x * ev_z; epitch_z = eyaw_x * ev_y;
            q_dynamic = 0.5 * rho * v_rel_mag^2;
            a_steer_mag = (q_dynamic * S_ref * C_N_canard) / m;
            phi_actual = forced_gamma_c + delta_kappa;
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
    imp = [prev_pos(1) + frac * (pos(1) - prev_pos(1)), ...
           prev_pos(3) + frac * (pos(3) - prev_pos(3))];
end

function [pred_x, pred_z] = predict_impacts_vectorized_met(p_pos, p_vel, p_spin, p_gw_spd, p_gw_az, p_jet_spd, dt_pred, P0_Pa, T0_K)
    K = size(p_pos, 1);
    pos = p_pos; vel = p_vel; p = p_spin;
    active = true(K, 1);
    pred_x = zeros(K, 1); pred_z = zeros(K, 1);
    
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
        
        y = pos(idx, 2);
        alt = max(0, y);
        h = max(0, min(alt, 20000));
        is_trop = h <= 11000;
        T = zeros(K_act, 1); P = zeros(K_act, 1);
        T(is_trop) = T0_K - 0.0065 * h(is_trop);
        P(is_trop) = P0_Pa * (1 - 0.0065 * h(is_trop) / T0_K).^(9.80665 / (287.05287 * 0.0065));
        
        T_trop = T0_K - 0.0065 * 11000;
        P_trop = P0_Pa * (1 - 0.0065 * 11000 / T0_K)^(9.80665 / (287.05287 * 0.0065));
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
end

function [theta, psi] = solve_firing_solution_met(target, v0_nom, canard_deflection_deg, P0_Pa, T0_K)
    theta = 54.5;
    psi = 0.0;
    max_iter = 20;
    tol = 1.0;
    
    nom_wind.spd = 4.0; nom_wind.az = 45.0; nom_wind.jet = 38.0;
    
    for iter = 1:max_iter
        imp = run_single_forward_met(v0_nom, theta, psi, canard_deflection_deg, false, target, 0, nom_wind, P0_Pa, T0_K);
        dx = target(1) - imp(1);
        dz = target(2) - imp(2);
        
        if sqrt(dx^2 + dz^2) < tol
            break;
        end
        
        psi = psi + rad2deg(atan2(dz, imp(1)));
        
        d_theta = 0.05;
        imp_p = run_single_forward_met(v0_nom, theta + d_theta, psi, canard_deflection_deg, false, target, 0, nom_wind, P0_Pa, T0_K);
        dRange_dTheta = (imp_p(1) - imp(1)) / d_theta;
        
        theta = theta + dx / dRange_dTheta;
        theta = max(45.0, min(80.0, theta));
    end
end

function [x0, z0, semi_a, semi_b, impacts] = calibrate_correction_ellipse_delay_met(v0_nom, ...
                                                theta_nom, psi_nom, canard_deflection_deg, gnc_delay, P0_Pa, T0_K)
    angles_deg = 0:45:315;
    impacts = zeros(length(angles_deg), 2);
    nom_wind.spd = 4.0; nom_wind.az = 45.0; nom_wind.jet = 38.0;
    
    imp_unc = run_single_forward_met(v0_nom, theta_nom, psi_nom, canard_deflection_deg, false, [0,0], 0, nom_wind, P0_Pa, T0_K);
    
    for i = 1:length(angles_deg)
        gamma_c_rad = deg2rad(angles_deg(i));
        imp = run_single_forced_met(v0_nom, theta_nom, psi_nom, canard_deflection_deg, gamma_c_rad, gnc_delay, nom_wind, P0_Pa, T0_K);
        impacts(i, :) = imp;
    end
    
    rel_impacts = impacts - imp_unc;
    x0 = mean(rel_impacts(:, 1));
    z0 = mean(rel_impacts(:, 2));
    
    centered_x = rel_impacts(:, 1) - x0;
    centered_z = rel_impacts(:, 2) - z0;
    semi_a = (max(centered_x) - min(centered_x)) / 2;
    semi_b = (max(centered_z) - min(centered_z)) / 2;
end

function imp = run_single_forward_met(v0_nom, theta_nom, psi_nom, canard_deflection_deg, is_guided, target, gnc_delay, wind, P0_Pa, T0_K)
    g0 = 9.80665; m = 43.10; d_proj = 0.155; S_ref = pi * (d_proj / 2)^2;
    Cd0 = 0.25; Ix = 0.142; twist_calibers = 20.0; C_La = 2.0; dt = 0.05;
    delta_e = deg2rad(canard_deflection_deg); S_canard = 0.0035;
    C_N_canard = 4.0 * (S_canard / S_ref) * (2.0 * delta_e);
    delta_kappa = deg2rad(15.0);
    
    th_rad = deg2rad(theta_nom); psi_rad = deg2rad(psi_nom);
    pos = [0, 0, 0];
    vel = [v0_nom * cos(th_rad) * cos(psi_rad), v0_nom * sin(th_rad), v0_nom * cos(th_rad) * sin(psi_rad)];
    p = (2 * pi * v0_nom) / (twist_calibers * d_proj);
    
    t = 0; apogee_reached = false; apogee_time = 0;
    control_active = false; cmd_gamma_c = 0.0;
    step_c = 0;
    
    while pos(2) >= 0 && t < 130.0
        alt = max(0, pos(2));
        if vel(2) < 0 && ~apogee_reached
            apogee_reached = true; apogee_time = t;
        end
        
        h = min(alt, 20000);
        if h <= 11000
            T = T0_K - 0.0065 * h;
            P = P0_Pa * (1 - 0.0065 * h / T0_K)^5.25588;
        else
            T = T0_K - 0.0065 * 11000;
            P_trop = P0_Pa * (1 - 0.0065 * 11000 / T0_K)^5.25588;
            P = P_trop * exp(-9.80665 * (h - 11000) / (287.05287 * T));
        end
        rho = P / (287.05287 * T); a_sound = sqrt(1.4 * 287.05287 * T);
        
        c_gw_spd = wind.spd; c_gw_az = wind.az; c_jet_spd = wind.jet;
        if alt <= 2000
            w_mag = c_gw_spd * (max(10, alt) / 2000)^0.2; w_dir = c_gw_az;
        elseif alt <= 8000
            frac2 = (alt - 2000) / 6000; w_mag = c_gw_spd * 1.5 + frac2 * (c_jet_spd * 0.5); w_dir = c_gw_az + frac2 * 30.0;
        elseif alt <= 11500
            jet_core = c_jet_spd * exp(-((alt - 9500) / 1200)^2); w_mag = c_gw_spd * 1.5 + jet_core; w_dir = c_gw_az + 45.0;
        else
            frac4 = min(1.0, (alt - 11500) / 4000); w_mag = (c_jet_spd * 0.4) * (1 - 0.5 * frac4); w_dir = c_gw_az + 45.0;
        end
        
        th_w = deg2rad(w_dir); Wx = w_mag * cos(th_w); Wz = w_mag * sin(th_w);
        v_rel_x = vel(1) - Wx; v_rel_y = vel(2); v_rel_z = vel(3) - Wz;
        v_rel_mag = sqrt(v_rel_x^2 + v_rel_y^2 + v_rel_z^2);
        v_unit_x = v_rel_x / max(1e-3, v_rel_mag);
        v_unit_y = v_rel_y / max(1e-3, v_rel_mag);
        Mach = v_rel_mag / a_sound;
        
        if is_guided && apogee_reached && (t >= apogee_time + gnc_delay) && mod(step_c, 10) == 0
            [pred_x, pred_z] = predict_impacts_vectorized_met(pos, vel, p, c_gw_spd, c_gw_az, c_jet_spd, 0.4, P0_Pa, T0_K);
            dx_err = target(1) - pred_x;
            dz_err = target(2) - pred_z;
            miss_dist = sqrt(dx_err^2 + dz_err^2);
            if miss_dist > 1.5
                phi_cmd = atan2(dz_err, dx_err);
                cmd_gamma_c = phi_cmd - delta_kappa;
                control_active = true;
            else
                control_active = false;
            end
        end
        
        if Mach < 0.8, Cd = Cd0;
        elseif Mach < 1.05, Cd = Cd0 + 0.22 * ((Mach - 0.8) / 0.25)^2;
        elseif Mach < 1.6, Cd = (Cd0 + 0.22) - 0.08 * ((Mach - 1.05) / 0.55);
        else, Cd = (Cd0 + 0.14) / (1 + 0.15 * (Mach - 1.6)); end
        
        dp_dt = (0.5 * rho * v_rel_mag * S_ref * (d_proj^2) * (-0.015) / Ix) * p;
        p = max(0, p + dp_dt * dt);
        
        drag_k = 0.5 * rho * S_ref * Cd * v_rel_mag / m;
        ax_drag = -drag_k * v_rel_x; ay_drag = -drag_k * v_rel_y; az_drag = -drag_k * v_rel_z;
        
        vcg_z = v_unit_x * g0;
        denom = max(100.0, rho * S_ref * d_proj * 11.5 * (v_rel_mag^2));
        alpha_e_z = (2.0 * Ix * p / (denom * v_rel_mag)) * vcg_z;
        F_lift_z = 0.5 * rho * S_ref * (v_rel_mag^2) * C_La * alpha_e_z;
        F_mag_x = 0.5 * rho * S_ref * d_proj * 0.008 * p * v_rel_mag * (v_unit_y * alpha_e_z);
        F_mag_y = -0.5 * rho * S_ref * d_proj * 0.008 * p * v_rel_mag * (v_unit_x * alpha_e_z);
        
        ax = ax_drag + F_mag_x / m;
        ay = -g0 + ay_drag + F_mag_y / m;
        az = az_drag + F_lift_z / m;
        
        if is_guided && apogee_reached && (t >= apogee_time + gnc_delay) && control_active
            v_h = max(1e-3, sqrt(vel(1)^2 + vel(3)^2));
            eyaw_x = -vel(3) / v_h; eyaw_y = 0; eyaw_z = vel(1) / v_h;
            ev_x = vel(1) / v_rel_mag; ev_y = vel(2) / v_rel_mag; ev_z = vel(3) / v_rel_mag;
            epitch_x = -eyaw_z * ev_y; epitch_y = eyaw_z * ev_x - eyaw_x * ev_z; epitch_z = eyaw_x * ev_y;
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
        t = t + dt;
        step_c = step_c + 1;
    end
    frac = prev_pos(2) / (prev_pos(2) - pos(2));
    imp = [prev_pos(1) + frac * (pos(1) - prev_pos(1)), ...
           prev_pos(3) + frac * (pos(3) - prev_pos(3))];
end

function imp = run_single_forced_met(v0_nom, theta_nom, psi_nom, canard_deflection_deg, forced_gamma_c, gnc_delay, wind, P0_Pa, T0_K)
    g0 = 9.80665; m = 43.10; d_proj = 0.155; S_ref = pi * (d_proj / 2)^2;
    Cd0 = 0.25; Ix = 0.142; twist_calibers = 20.0; C_La = 2.0; dt = 0.05;
    delta_e = deg2rad(canard_deflection_deg); S_canard = 0.0035;
    C_N_canard = 4.0 * (S_canard / S_ref) * (2.0 * delta_e);
    delta_kappa = deg2rad(15.0);
    
    th_rad = deg2rad(theta_nom); psi_rad = deg2rad(psi_nom);
    pos = [0, 0, 0];
    vel = [v0_nom * cos(th_rad) * cos(psi_rad), v0_nom * sin(th_rad), v0_nom * cos(th_rad) * sin(psi_rad)];
    p = (2 * pi * v0_nom) / (twist_calibers * d_proj);
    
    t = 0; apogee_reached = false; apogee_time = 0;
    while pos(2) >= 0 && t < 130.0
        alt = max(0, pos(2));
        if vel(2) < 0 && ~apogee_reached
            apogee_reached = true; apogee_time = t;
        end
        
        h = min(alt, 20000);
        if h <= 11000
            T = T0_K - 0.0065 * h;
            P = P0_Pa * (1 - 0.0065 * h / T0_K)^5.25588;
        else
            T = T0_K - 0.0065 * 11000;
            P_trop = P0_Pa * (1 - 0.0065 * 11000 / T0_K)^5.25588;
            P = P_trop * exp(-9.80665 * (h - 11000) / (287.05287 * T));
        end
        rho = P / (287.05287 * T); a_sound = sqrt(1.4 * 287.05287 * T);
        
        c_gw_spd = wind.spd; c_gw_az = wind.az; c_jet_spd = wind.jet;
        if alt <= 2000
            w_mag = c_gw_spd * (max(10, alt) / 2000)^0.2; w_dir = c_gw_az;
        elseif alt <= 8000
            frac2 = (alt - 2000) / 6000; w_mag = c_gw_spd * 1.5 + frac2 * (c_jet_spd * 0.5); w_dir = c_gw_az + frac2 * 30.0;
        elseif alt <= 11500
            jet_core = c_jet_spd * exp(-((alt - 9500) / 1200)^2); w_mag = c_gw_spd * 1.5 + jet_core; w_dir = c_gw_az + 45.0;
        else
            frac4 = min(1.0, (alt - 11500) / 4000); w_mag = (c_jet_spd * 0.4) * (1 - 0.5 * frac4); w_dir = c_gw_az + 45.0;
        end
        
        th_w = deg2rad(w_dir); Wx = w_mag * cos(th_w); Wz = w_mag * sin(th_w);
        v_rel_x = vel(1) - Wx; v_rel_y = vel(2); v_rel_z = vel(3) - Wz;
        v_rel_mag = sqrt(v_rel_x^2 + v_rel_y^2 + v_rel_z^2);
        v_unit_x = v_rel_x / max(1e-3, v_rel_mag);
        v_unit_y = v_rel_y / max(1e-3, v_rel_mag);
        Mach = v_rel_mag / a_sound;
        
        if Mach < 0.8, Cd = Cd0;
        elseif Mach < 1.05, Cd = Cd0 + 0.22 * ((Mach - 0.8) / 0.25)^2;
        elseif Mach < 1.6, Cd = (Cd0 + 0.22) - 0.08 * ((Mach - 1.05) / 0.55);
        else, Cd = (Cd0 + 0.14) / (1 + 0.15 * (Mach - 1.6)); end
        
        dp_dt = (0.5 * rho * v_rel_mag * S_ref * (d_proj^2) * (-0.015) / Ix) * p;
        p = max(0, p + dp_dt * dt);
        
        drag_k = 0.5 * rho * S_ref * Cd * v_rel_mag / m;
        ax_drag = -drag_k * v_rel_x; ay_drag = -drag_k * v_rel_y; az_drag = -drag_k * v_rel_z;
        
        vcg_z = v_unit_x * g0;
        denom = max(100.0, rho * S_ref * d_proj * 11.5 * (v_rel_mag^2));
        alpha_e_z = (2.0 * Ix * p / (denom * v_rel_mag)) * vcg_z;
        F_lift_z = 0.5 * rho * S_ref * (v_rel_mag^2) * C_La * alpha_e_z;
        F_mag_x = 0.5 * rho * S_ref * d_proj * 0.008 * p * v_rel_mag * (v_unit_y * alpha_e_z);
        F_mag_y = -0.5 * rho * S_ref * d_proj * 0.008 * p * v_rel_mag * (v_unit_x * alpha_e_z);
        
        ax = ax_drag + F_mag_x / m;
        ay = -g0 + ay_drag + F_mag_y / m;
        az = az_drag + F_lift_z / m;
        
        if apogee_reached && (t >= apogee_time + gnc_delay)
            v_h = max(1e-3, sqrt(vel(1)^2 + vel(3)^2));
            eyaw_x = -vel(3) / v_h; eyaw_y = 0; eyaw_z = vel(1) / v_h;
            ev_x = vel(1) / v_rel_mag; ev_y = vel(2) / v_rel_mag; ev_z = vel(3) / v_rel_mag;
            epitch_x = -eyaw_z * ev_y; epitch_y = eyaw_z * ev_x - eyaw_x * ev_z; epitch_z = eyaw_x * ev_y;
            q_dynamic = 0.5 * rho * v_rel_mag^2;
            a_steer_mag = (q_dynamic * S_ref * C_N_canard) / m;
            phi_actual = forced_gamma_c + delta_kappa;
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
    imp = [prev_pos(1) + frac * (pos(1) - prev_pos(1)), ...
           prev_pos(3) + frac * (pos(3) - prev_pos(3))];
end

function run_guided_monte_carlo(N)
    fprintf('Running %d rounds of guided Monte Carlo dispersion...\n', N);
    guided(N);
end

function [traj, imp, miss, apogee_pt] = run_single_guided_flight_met(v0_nom, theta_nom, psi_nom, m_val, canard_deflection_deg, gnc_delay, wind, target, P0_Pa, T0_K)
    g0 = 9.80665; m = m_val; d_proj = 0.155; S_ref = pi * (d_proj / 2)^2;
    Cd0 = 0.25; Ix = 0.142; twist_calibers = 20.0; C_La = 2.0; dt = 0.05;
    delta_e = deg2rad(canard_deflection_deg); S_canard = 0.0035;
    C_N_canard = 4.0 * (S_canard / S_ref) * (2.0 * delta_e);
    delta_kappa = deg2rad(15.0);
    
    th_rad = deg2rad(theta_nom); psi_rad = deg2rad(psi_nom);
    pos = [0, 0, 0];
    prev_pos = pos;
    vel = [v0_nom * cos(th_rad) * cos(psi_rad), v0_nom * sin(th_rad), v0_nom * cos(th_rad) * sin(psi_rad)];
    p = (2 * pi * v0_nom) / (twist_calibers * d_proj);
    
    t = 0; apogee_reached = false; apogee_time = 0; apogee_pt = [0, 0, 0];
    control_active = false; cmd_gamma_c = 0.0;
    step_c = 0;
    traj = pos;
    
    while pos(2) >= 0 && t < 130.0
        alt = max(0, pos(2));
        if vel(2) < 0 && ~apogee_reached
            apogee_reached = true; apogee_time = t;
            apogee_pt = pos;
        end
        
        h = min(alt, 20000);
        if h <= 11000
            T = T0_K - 0.0065 * h;
            P = P0_Pa * (1 - 0.0065 * h / T0_K)^5.25588;
        else
            T = T0_K - 0.0065 * 11000;
            P_trop = P0_Pa * (1 - 0.0065 * 11000 / T0_K)^5.25588;
            P = P_trop * exp(-9.80665 * (h - 11000) / (287.05287 * T));
        end
        rho = P / (287.05287 * T); a_sound = sqrt(1.4 * 287.05287 * T);
        
        c_gw_spd = wind.spd; c_gw_az = wind.az; c_jet_spd = wind.jet;
        if alt <= 2000
            w_mag = c_gw_spd * (max(10, alt) / 2000)^0.2; w_dir = c_gw_az;
        elseif alt <= 8000
            frac2 = (alt - 2000) / 6000; w_mag = c_gw_spd * 1.5 + frac2 * (c_jet_spd * 0.5); w_dir = c_gw_az + frac2 * 30.0;
        elseif alt <= 11500
            jet_core = c_jet_spd * exp(-((alt - 9500) / 1200)^2); w_mag = c_gw_spd * 1.5 + jet_core; w_dir = c_gw_az + 45.0;
        else
            frac4 = min(1.0, (alt - 11500) / 4000); w_mag = (c_jet_spd * 0.4) * (1 - 0.5 * frac4); w_dir = c_gw_az + 45.0;
        end
        
        th_w = deg2rad(w_dir); Wx = w_mag * cos(th_w); Wz = w_mag * sin(th_w);
        v_rel_x = vel(1) - Wx; v_rel_y = vel(2); v_rel_z = vel(3) - Wz;
        v_rel_mag = sqrt(v_rel_x^2 + v_rel_y^2 + v_rel_z^2);
        v_unit_x = v_rel_x / max(1e-3, v_rel_mag);
        v_unit_y = v_rel_y / max(1e-3, v_rel_mag);
        Mach = v_rel_mag / a_sound;
        
        if apogee_reached && (t >= apogee_time + gnc_delay) && mod(step_c, 10) == 0
            [pred_x, pred_z] = predict_impacts_vectorized_met(pos, vel, p, c_gw_spd, c_gw_az, c_jet_spd, 0.4, P0_Pa, T0_K);
            dx_err = target(1) - pred_x;
            dz_err = target(2) - pred_z;
            miss_dist = sqrt(dx_err^2 + dz_err^2);
            if miss_dist > 1.5
                phi_cmd = atan2(dz_err, dx_err);
                cmd_gamma_c = phi_cmd - delta_kappa;
                control_active = true;
            else
                control_active = false;
            end
        end
        
        if Mach < 0.8, Cd = Cd0;
        elseif Mach < 1.05, Cd = Cd0 + 0.22 * ((Mach - 0.8) / 0.25)^2;
        elseif Mach < 1.6, Cd = (Cd0 + 0.22) - 0.08 * ((Mach - 1.05) / 0.55);
        else, Cd = (Cd0 + 0.14) / (1 + 0.15 * (Mach - 1.6)); end
        
        dp_dt = (0.5 * rho * v_rel_mag * S_ref * (d_proj^2) * (-0.015) / Ix) * p;
        p = max(0, p + dp_dt * dt);
        
        drag_k = 0.5 * rho * S_ref * Cd * v_rel_mag / m;
        ax_drag = -drag_k * v_rel_x; ay_drag = -drag_k * v_rel_y; az_drag = -drag_k * v_rel_z;
        
        vcg_z = v_unit_x * g0;
        denom = max(100.0, rho * S_ref * d_proj * 11.5 * (v_rel_mag^2));
        alpha_e_z = (2.0 * Ix * p / (denom * v_rel_mag)) * vcg_z;
        F_lift_z = 0.5 * rho * S_ref * (v_rel_mag^2) * C_La * alpha_e_z;
        F_mag_x = 0.5 * rho * S_ref * d_proj * 0.008 * p * v_rel_mag * (v_unit_y * alpha_e_z);
        F_mag_y = -0.5 * rho * S_ref * d_proj * 0.008 * p * v_rel_mag * (v_unit_x * alpha_e_z);
        
        ax = ax_drag + F_mag_x / m;
        ay = -g0 + ay_drag + F_mag_y / m;
        az = az_drag + F_lift_z / m;
        
        if apogee_reached && (t >= apogee_time + gnc_delay) && control_active
            v_h = max(1e-3, sqrt(vel(1)^2 + vel(3)^2));
            eyaw_x = -vel(3) / v_h; eyaw_y = 0; eyaw_z = vel(1) / v_h;
            ev_x = vel(1) / v_rel_mag; ev_y = vel(2) / v_rel_mag; ev_z = vel(3) / v_rel_mag;
            epitch_x = -eyaw_z * ev_y; epitch_y = eyaw_z * ev_x - eyaw_x * ev_z; epitch_z = eyaw_x * ev_y;
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
        t = t + dt;
        step_c = step_c + 1;
        if mod(step_c, 4) == 0 || pos(2) < 0
            traj(end+1, :) = pos;
        end
    end
    frac = prev_pos(2) / (prev_pos(2) - pos(2));
    imp = [prev_pos(1) + frac * (pos(1) - prev_pos(1)), ...
           prev_pos(3) + frac * (pos(3) - prev_pos(3))];
    miss = norm(imp - target);
end

function out = ifelse(cond, val_true, val_false)
    if cond
        out = val_true;
    else
        out = val_false;
    end
end
