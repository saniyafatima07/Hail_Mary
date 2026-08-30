%% =========================================================================
%% YIL 155mm PGK TRAJECTORY SIMULATION & HARDWARE-IN-THE-LOOP (HIL) INTERFACE
%% =========================================================================
clear; clc; close all;

fprintf('Initializing 155mm Artillery PGK HIL System...\n');

%% 1. COM PORT CONFIGURATION (Connects to your Microcontroller)
% CHANGE 'COM3' to match your actual Arduino/ESP32 port in Device Manager
comPort = "COM3"; 
baudRate = 115200;

try
    device = serialport(comPort, baudRate);
    configureTerminator(device, "LF");
    flush(device);
    fprintf('Connected successfully to Microcontroller on %s\n', comPort);

    pause(2); % Allow microcontroller to reset/initialize
catch
    warning('Could not open serial port. Running in PURE EMULATION mode.');
    device = [];
end

%% 2. CONSTANTS & WEAPON PARAMETERS (Standard NATO 155mm)
g = 9.81;               % Gravity (m/s^2)
m = 43.5;               % Shell mass (kg)
d = 0.155;              % Shell diameter (m)
S = pi * (d/2)^2;       % Reference cross-sectional area (m^2)
rho_0 = 1.225;          % Sea-level air density (kg/m^3)
H_scale = 8500;         % Troposphere scale height for altitude-density model (m)
Cd_base = 0.25;         % Baseline aerodynamic drag coefficient

%% 3. INITIAL LAUNCH CONDITIONS & TARGET CONFIGURATION
v0 = 800;               % Muzzle velocity (m/s) ~ Mach 2.3
launchAngle = 45;       % Launch angle in degrees (Optimal for max ballistic range)
target_X = 22000;       % Downrange Target distance (meters) ~ 22 km
target_Y = 200;         % Lateral Target offset (meters) -> Simulating a crosswind shift

% Convert launch angle to radians
theta = deg2rad(launchAngle);

% State Vector initialization: [X_pos, Y_pos, Z_pos (Alt), X_vel, Y_vel, Z_vel]
state = [0, 0, 0, v0*cos(theta), 0, v0*sin(theta)]; 

% Wind Vector simulation (Steady wind pushing shell Left to Right)
wind_Y = -15;           % 15 m/s crosswind blowing along the Y-axis

%% 4. SIMULATION LOOP TIMING
dt = 0.1;               % Simulation step-time (100ms updates to prevent USB buffer clogging)
t = 0;                  % Elapsed time clock
max_time = 120;         % Safety exit time limit (seconds)

% Data logging arrays for post-flight visualization and CEP check
log_t = []; log_pos = []; log_vel = []; log_canard = [];

fprintf('\n=== Launching 155mm Shell into Virtual Trajectory ===\n');

%% 5. CORE SIMULATION & LOOP COORDINATION
while state(3) >= 0 && t < max_time % Run until shell impacts ground (Altitude Z < 0)
    
    % Extract current states for readability
    posX = state(1); posY = state(2); posZ = state(3);
    velX = state(4); velY = state(5); velZ = state(6);
    
    V_mag = sqrt(velX^2 + velY^2 + velZ^2); % Current velocity magnitude
    
    % Exponential Air Density Model based on current altitude (Z)
    rho = rho_0 * exp(-posZ / H_scale);
    
    %% GNC ALGORITHM: Calculate steering errors relative to Target location
    % Calculate Remaining Time-to-Impact estimate based on vertical ballistics
    t_rem = (velZ + sqrt(velZ^2 + 2*g*posZ)) / g;
    if isnan(t_rem) || t_rem < 0, t_rem = 0; end
    
    % Predict unguided landing footprint if no corrections are made
    predicted_X = posX + velX * t_rem;
    predicted_Y = posY + (velY + wind_Y) * t_rem; 
    
    % Compute cross-track positional errors
    error_X = target_X - predicted_X;
    error_Y = target_Y - predicted_Y;
    
    % Proportional Guidance Gain logic: Convert distance errors to desired fin deflection angles
    Kp = 0.005; 
    cmd_pitch_fin = max(min(error_X * Kp, 5), -5);   % Cap physical canard limits at +/- 5 degrees
    cmd_yaw_fin   = max(min(error_Y * Kp, 5), -5);   % Cap physical canard limits at +/- 5 degrees
    
    %% HIL HARDWARE BRIDGE: Streaming out to your servos
    actual_pitch_fin = cmd_pitch_fin; % Default fallback if hardware is disconnected
    actual_yaw_fin = cmd_yaw_fin;     % Default fallback if hardware is disconnected
    
    if ~isempty(device)
        % Package data string formatted neatly for the Microcontroller parser
        % Format: "PITCH_CMD,YAW_CMD\n"
        dataString = sprintf("%.2f,%.2f\n", cmd_pitch_fin, cmd_yaw_fin);
        writeline(device, dataString);
        
        % Read the response string coming back from physical microcontroller/actuators
        if device.NumBytesAvailable > 0
            response = readline(device);
            % Clean and parse incoming data feedback loop: [Actual_Pitch, Actual_Yaw]
            parsedData = sscanf(response, "%f,%f");
            if length(parsedData) == 2
                actual_pitch_fin = parsedData(1);
                actual_yaw_fin = parsedData(2);
            end
        end
    end
    
    %% PHYSICS ENGINE: Calculate forces acting on the Mach 2 shell
    % Compute standard aerodynamic drag force vector
    F_drag = 0.5 * rho * V_mag^2 * S * Cd_base;
    ax_drag = -(F_drag * (velX / V_mag)) / m;
    ay_drag = -(F_drag * ((velY - wind_Y) / V_mag)) / m; % Factors in structural crosswind resistance
    az_drag = -(F_drag * (velZ / V_mag)) / m;
    
    % Compute actual lift forces generated by physical canard actuator deflection
    Cl_per_deg = 0.02; % Lift slope coefficient of a micro canard winglet at high velocity
    F_lift_pitch = 0.5 * rho * V_mag^2 * S * (actual_pitch_fin * Cl_per_deg);
    F_lift_yaw   = 0.5 * rho * V_mag^2 * S * (actual_yaw_fin * Cl_per_deg);
    
    % Combine all forces via Newton's Second Law (F=ma -> a=F/m)
    accelX = ax_drag;
    accelY = ay_drag + (F_lift_yaw / m);
    accelZ = -g + az_drag + (F_lift_pitch / m);
    
    %% NUMERICAL INTEGRATION: Update flight kinematics for next time step
    state(4) = state(4) + accelX * dt; % Velocity X
    state(5) = state(5) + accelY * dt; % Velocity Y
    state(6) = state(6) + accelZ * dt; % Velocity Z
    
    state(1) = state(1) + state(4) * dt; % Position X (Downrange)
    state(2) = state(2) + state(5) * dt; % Position Y (Crossrange)
    state(3) = state(3) + state(6) * dt; % Position Z (Altitude)
    
    t = t + dt; % Increment timeline
    
    % Log data history vectors
    log_t = [log_t; t];
    log_pos = [log_pos; state(1:3)];
    log_vel = [log_vel; state(4:6)];
    log_canard = [log_canard; actual_pitch_fin, actual_yaw_fin];
end

fprintf('Trajectory ended. Shell impacted surface at t = %.1fs\n', t);

%% 6. VISUALIZATION DASHBOARD (Fulfills the Visibility requirement for YIL judges)
figure('Name', 'YIL 155mm PGK Real-Time Flight Profile', 'Position', [100, 100, 1000, 500]);

% Subplot 1: 3D Trajectory Profile
subplot(1,2,1);
plot3(log_pos(:,1)/1000, log_pos(:,2), log_pos(:,3)/1000, 'b-', 'LineWidth', 2.5); hold on;
plot3(target_X/1000, target_Y, 0, 'rx', 'MarkerSize', 12, 'LineWidth', 3);
grid on; box on;
xlabel('Downrange Distance (km)'); ylabel('Crossrange Dispersion (m)'); zlabel('Altitude (km)');
title('3D Guided Ballistic Trajectory');
legend('Shell Path', 'Target Hit Point', 'Location', 'best');
view(45, 20);

% Subplot 2: Actuator Tracking Performance
subplot(1,2,2);
plot(log_t, log_canard(:,1), 'r-', 'LineWidth', 2); hold on;
plot(log_t, log_canard(:,2), 'g-', 'LineWidth', 2);
grid on;
xlabel('Flight Time (seconds)'); ylabel('Canard Actuator Deflection (Degrees)');
title('HIL Real-Time Canard Deflection Profiles');
legend('Pitch Canard', 'Yaw Canard');

% Calculate Final Missing Distance to Target (Miss Distance / Accuracy Metric)
miss_distance = sqrt((state(1)-target_X)^2 + (state(2)-target_Y)^2);
fprintf('\n================ RESULTS ================\n');
fprintf('Final Target Coordinates: X = %dm, Y = %dm\n', target_X, target_Y);
fprintf('Shell Impact Coordinates: X = %.1fm, Y = %.1fm\n', state(1), state(2));
fprintf('Terminal Strike Miss Distance: %.2f meters\n', miss_distance);
if miss_distance <= 30
    fprintf('SUCCESS: System meets the YIL target accuracy specification (CEP <= 30m).\n');
else
    fprintf('FAILED: Miss distance exceeds structural 30m parameters.\n');
end
fprintf('=========================================\n');
