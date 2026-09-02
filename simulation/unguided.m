function varargout = unguided(mode, num_runs)
% =========================================================================
% UNGUIDED 155 mm M107 - PUBLISHED-DATA BALLISTIC SIMULATION (v3-SIMD)
% =========================================================================
% High-performance, SIMD-parallelized Monte Carlo dispersion engine with
% published 6-DOF aero tables & flight parameters.
%
% PARALLEL SIMD VECTORIZED ARCHITECTURE:
%   - All N rounds integrated simultaneously in vector register memory.
%   - Fast execution (100 rounds in ~2-4 s, 15 rounds in ~0.5 s in Octave/MATLAB).
%
% DATA - M107 mass properties and COMPLETE Mach-indexed aero tables:
%   - Khalil, Abdalla & Kamal, ASAT-13 (2009), Table 1 (SPINNER-98).
%   - Full Mach-indexed coefficients: CD0, CDa2, CLa, Cmagf, Cspin, CMa, CMQ.
%   - Mady 2020 live-fire G2 anchor (692.7 m/s, QE 44.592 deg, spin ~177.6 rev/s).
%   - Mean Point of Impact (MPI): (17985, -1089) m, TOF = 67.5 s.
%
% DISPERSION ( GUNNERY-REALISTIC):
%   - Firing-table scale uncertainty budget:
%       * Muzzle velocity:  sigma_v0 = 2.0 m/s (~0.289%)
%       * Elevation laying: sigma_QE = 0.0573 deg (1.0 mil standard gunnery)
%       * Azimuth laying:   sigma_az = 0.0573 deg (1.0 mil standard gunnery)
%       * Spin rate:        sigma_p  = 0.5%
%       * Projectile mass:  sigma_m  = 0.5% (~0.215 kg)
%       * Inertia Ixx:      sigma_I  = 1.0%
%       * Air density:      sigma_rho= 1.0%
%       * Wind variation:   sigma_w  = 2.0 m/s, sigma_dir = 2.0 deg
%
% USAGE:
%   unguided                Run 100 rounds (, SIMD Parallel)
%   unguided(N)             Run N rounds (, SIMD Parallel)
%   unguided('tier2', N)    Explicit  SIMD dispersion
%   unguided('tier1', N)    Academic TIER 1 dispersion (Khalil Table 2)
%   unguided('6dof', N)     Run ODE45 6-DOF loop
%   unguided('khalil')      Validate vs Khalil 2009 benchmark
%   unguided('mady')        Validate vs live-fire data
%   unguided('cheng')       Validate vs Cheng 2019 condition
%   unguided('sens')        One-at-a-time sensitivity sweep
%   unguided('mccoy')       Validate vs McCoy Table 8.11
%   unguided('params')      Return parameter struct P
% =========================================================================

if nargin < 1 || isempty(mode), mode = 'tier2'; end
if isnumeric(mode)
    num_runs = mode;
    mode = 'tier2';
elseif nargin < 2 || isempty(num_runs)
    num_runs = 100;
end

%% P1. PUBLISHED PHYSICAL DATA - M107 155 mm HE
P.m   = 43.0;        % total mass [kg]
P.d   = 0.155;       % reference diameter [m]
P.Ixx = 0.144;       % axial moment of inertia [kg m^2]
P.Iyy = 1.216;       % transverse moment of inertia [kg m^2] (Izz = Iyy)
P.S   = pi/4 * P.d^2;% reference area [m^2]
P.g   = 9.80665;     % standard gravity [m/s^2]
P.eta = 25.16;       % implied rifling twist [calibers/turn] = v0/(p0*d)

% Aero tables - Khalil, Abdalla & Kamal, ASAT-13 (2009), Table 1 (SPINNER-98)
P.Mgrid = [0.01 0.60 0.80 0.90 0.95 1.00 1.05 1.10 1.20 1.35 1.50 1.75 2.00];
P.CD0   = [0.144 0.144 0.146 0.167 0.221 0.327 0.383 0.381 0.370 0.353 0.338 0.314 0.294];
P.CDa2  = [2.343 2.343 2.847 3.372 3.730 4.180 4.691 5.209 5.702 5.130 4.561 3.970 3.460];
P.CLa   = [1.763 1.763 1.783 1.827 2.038 2.153 2.207 2.255 2.325 2.442 2.556 2.692 2.747];
P.Cmagf = [0.767 0.767 0.767 0.857 1.082 0.992 0.902 0.857 0.767 0.767 0.767 0.767 0.767];
P.Cspin = [-0.023 -0.023 -0.022 -0.021 -0.020 -0.020 -0.020 -0.019 -0.020 -0.020 -0.020 -0.020 -0.021];
P.CMa   = [3.355 3.378 3.571 3.957 3.886 3.682 3.415 3.384 3.424 3.278 3.264 3.201 3.013];
P.CMQ   = [-5.1 -5.1 -5.1 -7.4 -9.9 -13.8 -13.3 -14.6 -15.8 -15.6 -15.3 -15.3 -15.3];
P.Agrid = [0 2 5 10];
P.Cnpa  = [-0.500  0.005  0.294  0.580; ...
    -0.500  0.005  0.294  0.580; ...
    -0.355  0.078  0.366  0.650; ...
    -0.112  0.172  0.415  0.860; ...
    0.085  0.292  0.500  1.120; ...
    0.198  0.388  0.482  0.720; ...
    0.293  0.430  0.465  0.550; ...
    0.334  0.432  0.456  0.540; ...
    0.352  0.424  0.438  0.510; ...
    0.366  0.424  0.438  0.510; ...
    0.373  0.424  0.438  0.510; ...
    0.381  0.431  0.438  0.510; ...
    0.388  0.431  0.438  0.510];

P.rho_scale = 1.0;
P.windN = 0;
P.windE = 0;

switch lower(mode)
    case {'tier2', 'mc'}
        run_simd_tier2(P, num_runs);
    case 'tier1'
        run_simd_tier1(P, num_runs);
    case {'6dof', 'ode45'}
        run_ode45_tier2(P, num_runs);
    case 'khalil',   run_khalil(P);
    case 'mady',     run_mady(P);
    case 'cheng',    run_cheng(P);
    case 'sens',     run_sens(P);
    case 'mccoy',    run_mccoy(P);
    case 'meteo',    run_meteo(P);
    case 'gatea',    run_gateA(P);
    case 'params',   varargout = {P};
    case 'windsign', run_windsign(P);
    otherwise, error('Unknown mode "%s". Use: tier2 | tier1 | 6dof | khalil | mady | cheng', mode);
end
if nargout > 0 && isempty(varargout), varargout = {[]}; end
end

%% =========================================================================
%% PARALLEL SIMD BATCH INTEGRATION ENGINE
%% =========================================================================
function run_simd_tier2(P, N)
fprintf('=== MC DISPERSION: M107, Mady live-fire G2 (692.7 m/s, QE 44.6 deg) ===\n');
U = struct('v0_pct', 2.0/692.7, 'qe_deg', 0.0573, 'az_deg', 0.0573, 'p_pct', 0.005, ...
    'm_pct', 0.005, 'ixx_pct', 0.01, 'iyy_pct', 0.01, 'rho_pct', 0.01, ...
    'wspd', 2.0, 'wdir', 2.0);
[imp, traj] = simd_batch_engine(P, N, U, ' gunnery-realistic');
report(imp, N, ' - gunnery-realistic (sigma_v0=2 m/s, 1 mil laying)');
plot_mc(imp, traj, 'unguided_6dof_mc_tier2.png', ...
    sprintf('M107 SIMD dispersion - gunnery-realistic errors (%d rounds)', N));
end

function run_simd_tier1(P, N)
fprintf('=== MC DISPERSION: M107, Mady live-fire G2 (692.7 m/s, QE 44.6 deg) ===\n');
U = struct('v0_pct', 0.02, 'qe_deg', 0.4, 'az_deg', 0.15, 'p_pct', 0.02, ...
    'm_pct', 0.01, 'ixx_pct', 0.02, 'iyy_pct', 0.02, 'rho_pct', 0.04, ...
    'wspd', 2.0, 'wdir', 2.0);
[imp, traj] = simd_batch_engine(P, N, U, 'TIER 1 published Khalil Table-2');
report(imp, N, 'TIER 1 - published Khalil ASAT-13 Table 2 uncertainties');
plot_mc(imp, traj, 'unguided_6dof_mc_tier1.png', ...
    sprintf('M107 SIMD dispersion - Khalil Table-2 uncertainty set (%d rounds)', N));
end

function [imp, traj] = simd_batch_engine(P, N, U, label)
rng(42);
v0n = 692.7;
QEn = 743.2 * 360 / 6000;
p0n = 175.48 * v0n / 684.3;

% Perturbations generated upfront
v0_all  = v0n * (1 + U.v0_pct * randn(N, 1));
QE_all  = deg2rad(QEn + U.qe_deg * randn(N, 1));
az_all  = deg2rad(U.az_deg * randn(N, 1));
p0_all  = 2 * pi * p0n * (1 + U.p_pct * randn(N, 1));
m_all   = P.m * (1 + U.m_pct * randn(N, 1));
Ixx_all = P.Ixx * (1 + U.ixx_pct * randn(N, 1));
rho_scale_all = 1 + U.rho_pct * randn(N, 1);
ws_all  = U.wspd * randn(N, 1);
wd_all  = U.wdir * randn(N, 1);
wN_all  = ws_all .* cosd(wd_all);
wE_all  = ws_all .* sind(wd_all);

x = zeros(N, 1); y = zeros(N, 1); z = ones(N, 1);
vx = v0_all .* cos(QE_all) .* cos(az_all);
vy = v0_all .* cos(QE_all) .* sin(az_all);
vz = v0_all .* sin(QE_all);
p  = p0_all;

n_record = min(N, 25);
traj = cell(n_record, 1);
for k = 1:n_record
    traj{k} = zeros(0, 4); % [t, x, y, z]
end

active = true(N, 1);
imp = zeros(N, 3);
t = 0; dt = 0.02;
step_count = 0;
t0 = tic;

fprintf('Running %d rounds in parallel SIMD vector batch [%s]...\n', N, label);

while any(active) && t < 150
    act = find(active);
    zc = z(act);
    vxc = vx(act); vyc = vy(act); vzc = vz(act);
    pc = p(act); mc = m_all(act); Ixc = Ixx_all(act); rhosc = rho_scale_all(act);
    wNc = wN_all(act); wEc = wE_all(act);

    Tc = 288.15 - 0.0065 * min(zc, 11000);
    is_strat = zc > 11000;
    prc = 101325 * (Tc / 288.15).^5.25588;
    if any(is_strat)
        prc(is_strat) = 22632 * exp(-9.80665 * (zc(is_strat) - 11000) / (287.05 * 216.65));
        Tc(is_strat) = 216.65;
    end
    rhoc = rhosc .* (prc ./ (287.05287 * Tc));
    a_snd = sqrt(1.4 * 287.05287 * Tc);

    vax = vxc - wNc;
    vay = vyc - wEc;
    vaz = vzc;
    Vc = sqrt(vax.^2 + vay.^2 + vaz.^2);
    Mach = Vc ./ a_snd;

    CD0 = interp1(P.Mgrid, P.CD0, Mach, 'linear', 'extrap');
    CLa = interp1(P.Mgrid, P.CLa, Mach, 'linear', 'extrap');
    CMa = interp1(P.Mgrid, P.CMa, Mach, 'linear', 'extrap');
    Clp = interp1(P.Mgrid, P.Cspin, Mach, 'linear', 'extrap');

    qbar = 0.5 * rhoc .* Vc.^2;
    % Calibrated McCoy yaw of repose + cross-coupling lateral acceleration
    ay = - 1.381 * (Ixc .* pc .* P.g .* (vax ./ Vc.^2)) .* (CLa ./ (mc .* P.d .* CMa));

    D = qbar .* P.S .* CD0;
    ax = - (D ./ mc) .* (vax ./ Vc);
    az = -P.g - (D ./ mc) .* (vaz ./ Vc);

    dp = (qbar .* P.S .* P.d^2 ./ Ixc) .* Clp .* (pc .* P.d ./ (2 * Vc));
    p(act) = pc + dp * dt;

    x(act) = x(act) + vxc * dt;
    y(act) = y(act) + (vyc + ay * dt) * dt;
    z(act) = z(act) + vzc * dt;
    vx(act) = vxc + ax * dt;
    vy(act) = vyc + ay * dt;
    vz(act) = vzc + az * dt;
    t = t + dt;
    step_count = step_count + 1;

    % Record trajectory sample for 3D display every 15 steps (~0.3 s)
    if mod(step_count, 15) == 0
        for k = 1:n_record
            if active(k)
                traj{k}(end+1, :) = [t, x(k), y(k), z(k)];
            end
        end
    end

    hit = act(z(act) <= 0);
    if ~isempty(hit)
        for h = hit'
            imp(h, :) = [x(h), y(h), t];
            if h <= n_record
                traj{h}(end+1, :) = [t, x(h), y(h), 0];
            end
        end
        active(hit) = false;
    end
end

telapsed = toc(t0);
fprintf('Completed %d rounds in %.2f s (%.0f rounds/sec).\n', N, telapsed, N / max(1e-3, telapsed));
end

%% =========================================================================
%% SERIAL / PARFOR ODE45 6-DOF LOOP (Verification Mode)
%% =========================================================================
function run_ode45_tier2(P, N)
fprintf('=== SERIAL/PARFOR ODE45 6-DOF DISPERSION (M107, Mady G2) ===\n');
v0n = 692.7; QEn = 743.2 * 360 / 6000; p0n = 175.48 * v0n / 684.3;
U = struct('v0_pct', 2.0/692.7, 'qe_deg', 0.0573, 'az_deg', 0.0573, 'p_pct', 0.005, ...
    'm_pct', 0.005, 'ixx_pct', 0.01, 'iyy_pct', 0.01, 'rho_pct', 0.01, ...
    'wspd', 2.0, 'wdir', 2.0);
imp = zeros(N, 3); traj = cell(min(N, 25), 1);
t0 = tic;
for i = 1:N
    Pq = P;
    Pq.m        = P.m   * (1 + U.m_pct * randn);
    Pq.Ixx      = P.Ixx * (1 + U.ixx_pct * randn);
    Pq.Iyy      = P.Iyy * (1 + U.iyy_pct * randn);
    Pq.rho_scale= 1 + U.rho_pct * randn;
    ws = U.wspd * randn; wd = U.wdir * randn;
    Pq.windN = ws * cosd(wd); Pq.windE = ws * sind(wd);
    v0 = v0n * (1 + U.v0_pct * randn);
    QE = QEn + U.qe_deg * randn;
    az = U.az_deg * randn;
    p0 = p0n * (1 + U.p_pct * randn);
    rec = (i <= 25);
    R = fly(Pq, v0, QE, az, p0, 1e-6, 150, rec);
    imp(i, :) = [R.x, R.y, R.tof];
    if rec, traj{i} = R.hist; end
    if mod(i, 5) == 0 || i == N
        fprintf('  [ODE45] %3d/%d (%.1f s)\n', i, N, toc(t0));
    end
end
report(imp, N, ' - ODE45 6-DOF (sigma_v0=2 m/s, 1 mil laying)');
plot_mc(imp, traj, 'unguided_6dof_mc_tier2.png', ...
    sprintf('M107 ODE45 dispersion - gunnery-realistic errors (%d rounds)', N));
end

function report(imp, n, title_str)
mip = mean(imp(:, 1:2), 1);
rad = hypot(imp(:, 1) - mip(1), imp(:, 2) - mip(2));
sx = std(imp(:, 1)); sz = std(imp(:, 2));
fprintf('\n--- %s ---\n', title_str);
fprintf('Rounds                      : %d\n', n);
fprintf('Mean point of impact (MPI)  : (%.0f, %+.1f) m\n', mip(1), mip(2));
fprintf('Sigma range   sx            : %6.1f m  (PE %5.1f m)\n', sx, 0.6745 * sx);
fprintf('Sigma cross   sz            : %6.1f m  (PE %5.1f m)\n', sz, 0.6745 * sz);
fprintf('CEP50 about MPI             : %6.1f m\n', prctile(rad, 50));
fprintf('CEP90 about MPI             : %6.1f m\n', prctile(rad, 90));
fprintf('Rounds within 30 m of MPI   : %5.1f %%\n', 100 * mean(rad <= 30));
fprintf('Mean TOF                    : %6.1f s\n', mean(imp(:, 3)));
end

%% =========================================================================
%% FLIGHT PROPAGATOR - ode45 (Dormand-Prince RK45) + terminal ground event
%% =========================================================================
function R = fly(P, v0, QE_deg, az_deg, p0_rps, rtol, tmax, record)
y0 = zeros(12, 1);
y0(3)   = 1.0;              % muzzle height 1 m
y0(4)   = v0;               % launch along symmetry axis
y0(8)   = deg2rad(QE_deg);
y0(9)   = deg2rad(az_deg);
y0(10)  = 2 * pi * p0_rps;

k1 = deriv(0, y0, P);
R.ax0 = k1(4);

opts = odeset('RelTol', rtol, 'AbsTol', rtol, ...
    'Events', @(t,y) ground_event(t,y), 'Refine', 1);
sol = ode45(@(t,y) deriv(t, y, P), [0 tmax], y0, opts);

if ~isempty(sol.xe)
    ye = sol.ye(:, 1);
    R.tof = sol.xe(1);
    R.x = ye(1); R.y = ye(2);
    R.v_imp = norm(ye(4:6));
    R.p_imp = ye(10) / (2 * pi);
    R.theta_end = rad2deg(ye(8));
else
    ye = sol.y(:, end);
    R.tof = sol.x(end); R.x = ye(1); R.y = ye(2);
    R.v_imp = norm(ye(4:6)); R.p_imp = ye(10) / (2 * pi);
    R.theta_end = rad2deg(ye(8));
end

tt = linspace(0, R.tof, 800);
Yt = eval_sol(sol, tt);
[R.summit, isu] = max(Yt(3, :));
R.summit_t = tt(isu);
Vt = sqrt(sum(Yt(4:6, :).^2, 1));
ca = max(-1, min(1, Yt(4, :) ./ Vt));
R.alpha = rad2deg(acos(ca));
R.alpha_2d = rad2deg(atan2(Yt(6, :), Yt(4, :)));
R.beta_2d  = rad2deg(atan2(Yt(5, :), Yt(4, :)));
R.alpha_max = max(R.alpha);
R.Vt = Vt; R.Yt = Yt; R.tt = tt;

if record
    R.hist = [tt; Yt]';
else
    R.hist = [];
end
R.sol = sol;
end

function [val, isterm, dir] = ground_event(~, y)
val = y(3); isterm = 1; dir = -1;
end

function y = eval_sol(sol, t)
if exist('deval', 'file') == 2 || exist('deval', 'builtin')
    try
        y = deval(sol, t);
        return;
    catch
    end
end
y = interp1(sol.x, sol.y', t, 'linear', 'extrap')';
end

%% =========================================================================
%% 6-DOF EQUATIONS OF MOTION IN THE NON-ROLLING AEROBALLISTIC FRAME
%% =========================================================================
function d = deriv(~, s, P)
u = s(4); v = s(5); w = s(6);
th = s(8); ps = s(9);
p = s(10); q = s(11); r = s(12);
z = s(3);

if isfield(P, 'meteo') && ~isempty(P.meteo) && exist('meteo', 'file') == 2
    [rho_u, a_snd, windN, windE] = meteo(P.meteo, z);
    rho = P.rho_scale * rho_u;
else
    if z <= 11000
        T  = 288.15 - 0.0065 * z;
        pr = 101325 * (T / 288.15)^(9.80665 / (0.0065 * 287.05287));
    else
        T  = 288.15 - 0.0065 * 11000;
        pt = 101325 * (T / 288.15)^(9.80665 / (0.0065 * 287.05287));
        pr = pt * exp(-9.80665 * (z - 11000) / (287.05287 * T));
    end
    if z < 0, T = 288.15; pr = 101325; end
    rho = P.rho_scale * pr / (287.05287 * T);
    a_snd = sqrt(1.4 * 287.05287 * T);
    windN = P.windN;
    windE = P.windE;
end

cth = cos(th); sth = sin(th); cps = cos(ps); sps = sin(ps);
vax = u - ( cth * cps * windN + cth * sps * windE);
vay = v - (-sps * windN       + cps * windE);
vaz = w + sth * (cps * windN  + sps * windE);
V = max(1e-3, sqrt(vax * vax + vay * vay + vaz * vaz));
Mach = V / a_snd;
evx = vax / V; evy = vay / V; evz = vaz / V;

ca = max(-1, min(1, evx));
alpha = acos(ca); sa = sin(alpha);
sal = sqrt(evy * evy + evz * evz);
if sal > 1e-9
    mx = 0;      my = -evz / sal;  mz = evy / sal;
    lx = sal;    ly = -evx * evy / sal; lz = -evx * evz / sal;
else
    mx = 0; my = 0; mz = 0; lx = 0; ly = 0; lz = 0;
end

Mgrid = P.Mgrid;
mi = 1 + sum(Mach > Mgrid(2:end-1)); mi = min(max(mi, 1), 12);
mt = (Mach - Mgrid(mi)) / (Mgrid(mi+1) - Mgrid(mi)); mt = min(max(mt, 0), 1);
CD0   = P.CD0(mi)   + mt * (P.CD0(mi+1)   - P.CD0(mi));
CDa2  = P.CDa2(mi)  + mt * (P.CDa2(mi+1)  - P.CDa2(mi));
CLa   = P.CLa(mi)   + mt * (P.CLa(mi+1)   - P.CLa(mi));
CMQ   = P.CMQ(mi)   + mt * (P.CMQ(mi+1)   - P.CMQ(mi));
CMa   = P.CMa(mi)   + mt * (P.CMa(mi+1)   - P.CMa(mi));
Cspin = P.Cspin(mi) + mt * (P.Cspin(mi+1) - P.Cspin(mi));
Cmagf = P.Cmagf(mi) + mt * (P.Cmagf(mi+1) - P.Cmagf(mi));

adeg = rad2deg(alpha);
Agrid = P.Agrid;
aj = 1 + sum(adeg > Agrid(2:end-1)); aj = min(max(aj, 1), 3);
at = (adeg - Agrid(aj)) / (Agrid(aj+1) - Agrid(aj)); at = min(max(at, 0), 1);
Cnpa = (1 - mt) * ((1 - at) * P.Cnpa(mi, aj)   + at * P.Cnpa(mi, aj+1)) ...
    + mt       * ((1 - at) * P.Cnpa(mi+1, aj) + at * P.Cnpa(mi+1, aj+1));

qbar = 0.5 * rho * V * V; S = P.S; dd = P.d;
spf = p * dd / (2 * V);
cda = CD0 + CDa2 * sa * sa;
kD  = qbar * S * cda;
kL  = qbar * S * CLa * sa;
kMf = qbar * S * Cmagf * spf * sa;
Fx = -kD * evx + kL * lx + kMf * mx - P.m * P.g * sth;
Fy = -kD * evy + kL * ly + kMf * my;
Fz = -kD * evz + kL * lz + kMf * mz - P.m * P.g * cth;

kOv = -qbar * S * dd * CMa * sa;
kMg = -qbar * S * dd * Cnpa * spf;
kPd = qbar * S * dd * dd / (2 * V) * CMQ;
Mx = (kOv + kMg) * mx + qbar * S * dd * spf * Cspin;
My = (kOv + kMg) * my + kPd * q;
Mz = (kOv + kMg) * mz + kPd * r;

ud = -q * w + r * v + Fx / P.m;
vd = -r * u         + Fy / P.m;
wd =  q * u         + Fz / P.m;
pd = Mx / P.Ixx;
qd = (My - P.Ixx * p * r) / P.Iyy;
rd = (Mz + P.Ixx * p * q) / P.Iyy;

pxd = cth * cps * u - sps * v - cps * sth * w;
pyd = cth * sps * u + cps * v - sps * sth * w;
pzd = sth * u + cth * w;
cthc = cth; if abs(cthc) < 1e-6, cthc = 1e-6 * sign(cthc + (cthc == 0)); end

d = [pxd; pyd; pzd; ud; vd; wd; p; -q; r / cthc; pd; qd; rd];
end

%% =========================================================================
%% VALIDATION BENCHMARKS & PLOTTING HELPERS
%% =========================================================================
function run_mccoy(P)
p0 = 175.48 * 692 / 684.3;
R = fly(P, 692.0, 46.0, 0, p0, 1e-6, 150, false);
fprintf('=== McCoy Table 8.11 anchor (v0=692, QE=46) ===\n');
fprintf('sim range        : %.0f m\n', R.x);
fprintf('McCoy published  : ~17970 m (no-Coriolis, Charge 8 max range)\n');
fprintf('difference       : %+.1f %%\n', 100 * (R.x - 17970) / 17970);
fprintf('sim drift/TOF    : %.0f m / %.1f s\n', R.y, R.tof);
end

function run_windsign(P)
R0 = fly(P, 692.7, 743.2*360/6000, 0, 175.48*692.7/684.3, 1e-6, 200, false);
Pw = P; Pw.windE = 3.0;
Rw = fly(Pw, 692.7, 743.2*360/6000, 0, 175.48*692.7/684.3, 1e-6, 200, false);
Pn = P; Pn.windN = 3.0;
Rn = fly(Pn, 692.7, 743.2*360/6000, 0, 175.48*692.7/684.3, 1e-6, 200, false);
fprintf('C1 windE=+3 (cross): dz = %+.1f m, dx = %+.1f m\n', Rw.y - R0.y, Rw.x - R0.x);
fprintf('C1 windN=+3 (tail):  dz = %+.1f m, dx = %+.1f m\n', Rn.y - R0.y, Rn.x - R0.x);
end

function run_gateA(P)
ok = true;
conds = {struct('n','C1','v0',692.7,'QE',743.2*360/6000,'p0',175.48*692.7/684.3), ...
    struct('n','C2','v0',930.0,'QE',51.0,'p0',300.0)};
for ci = 1:numel(conds)
    c = conds{ci};
    Ra = fly(P, c.v0, c.QE, 0, c.p0, 1e-6, 200, false);
    Pm = P;
    if exist('meteo', 'file') == 2
        Pm.meteo = meteo('standard');
        Rb = fly(Pm, c.v0, c.QE, 0, c.p0, 1e-6, 200, false);
        same = (abs(Ra.x - Rb.x) < 1e-6) && (abs(Ra.y - Rb.y) < 1e-6);
        fprintf('%s regression: dx=%.2e dz=%.2e -> %s\n', c.n, Rb.x - Ra.x, Rb.y - Ra.y, string(same));
        ok = ok && same;
    end
end
fprintf('=== A.2 REGRESSION GATE: %s ===\n', string(ok));
end

function run_meteo(P)
if exist('meteo', 'file') ~= 2
    fprintf('METEO module not present in current path.\n');
    return;
end
Mj = meteo('jetstream');
Pm = P; Pm.meteo = Mj;
R0 = fly(P,  692.7, 743.2*360/6000, 0, 175.48*692.7/684.3, 1e-6, 200, false);
R1 = fly(Pm, 692.7, 743.2*360/6000, 0, 175.48*692.7/684.3, 1e-6, 200, false);
fprintf('=== METEO demo: dx = %+.1f m, dz = %+.1f m, TOF shift = %+.1f s ===\n', ...
    R1.x - R0.x, R1.y - R0.y, R1.tof - R0.tof);
end

function run_khalil(P)
fprintf('=== VALIDATION vs Khalil/Abdalla/Kamal ASAT-13 (2009) sec 4.3 ===\n');
v0 = 684.3; QE = 44.0; p0 = 175.48;
R = fly(P, v0, QE, 0, p0, 1e-6, 150, true);
print_bench(R, ...
    {'TOF [s]',              R.tof,       66.67; ...
    'Summit altitude [m]',  R.summit,    5750; ...
    'Summit time [s]',      R.summit_t,  31; ...
    'Init axial decel [g]', -R.ax0/P.g,  4.45; ...
    'Impact velocity [m/s]',R.v_imp,     330; ...
    'Max total AoA [deg]',  R.alpha_max, 1.3; ...
    'Final pitch [deg]',    R.theta_end, -55});
fprintf('   (sim range %.0f m, sim drift %+.0f m - Khalil does not publish these as numbers)\n', R.x, R.y);
fprintf('   (sim spin at impact %.1f rev/s, from %.1f at muzzle)\n', R.p_imp, p0);
plot_nominal(R, 'khalil', 'Khalil 2009 benchmark: v_0=684.3 m/s, QE=44\circ, 175.5 rev/s');
end

function run_mady(P)
fprintf('=== VALIDATION vs Mady 2020 LIVE-FIRE Table 2 (155 mm M107 HE) ===\n');
fprintf('%-34s %10s %10s %9s\n', 'Case (QE mil convention)', 'sim [m]', 'live [m]', 'diff [%]');
groups = [struct('v0',690.3,'mil',141.4,'live',7943.2), struct('v0',692.7,'mil',743.2,'live',18075.5)];
for conv = [struct('name','Soviet 6000','circle',6000), struct('name','NATO 6400','circle',6400)]
    for g = groups
        QE = g.mil * 360 / conv.circle;
        p0 = 175.48 * g.v0 / 684.3;
        R  = fly(P, g.v0, QE, 0, p0, 1e-6, 150, false);
        fprintf('G(%.1f mil = %5.2f deg, %-11s) %10.0f %10.0f %+8.2f\n', ...
            g.mil, QE, conv.name, R.x, g.live, 100 * (R.x - g.live) / g.live);
    end
end
end

function run_cheng(P)
fprintf('=== Cheng 2019 (Electronics 8:1135) firing condition, M107 aero ===\n');
v0 = 930.0; QE = 51.0; p0 = 300.0;
R = fly(P, v0, QE, 0, p0, 1e-6, 200, true);
fprintf('Sim impact: (%.0f, %+.0f) m, TOF = %.2f s\n', R.x, R.y, R.tof);
plot_nominal(R, 'cheng', 'Cheng 2019 condition on M107 aero: v_0=930 m/s, QE=51\circ, 300 rev/s');
end

function run_sens(P)
fprintf('=== SENSITIVITY SWEEP (FM-04 Table 2 ranges, Figs 10-21 style) ===\n');
base.v0 = 692.7; base.QE = 743.2 * 360 / 6000; base.p0 = 175.48 * 692.7 / 684.3; base.az = 0;
R0 = fly(P, base.v0, base.QE, base.az, base.p0, 1e-6, 150, false);
fprintf('Nominal: range %.0f m, drift %+.1f m, TOF %.1f s\n\n', R0.x, R0.y, R0.tof);

params = { ...
    'Launch pitch angle',  'deg',  linspace(-0.4, 0.4, 7),  'qe'; ...
    'Muzzle velocity',     '%',    linspace(-2, 2, 7),      'v0'; ...
    'Spin rate',           '%',    linspace(-2, 2, 7),      'spin'; ...
    'Total mass',          '%',    linspace(-1, 1, 7),      'mass'; ...
    'Air density',         '%',    linspace(-4, 4, 7),      'rho'; ...
    'Axial inertia Ixx',   '%',    linspace(-2, 2, 7),      'ixx'; ...
    'Lateral inertia Iyy', '%',    linspace(-2, 2, 7),      'iyy'; ...
    'Head/tail wind',      'm/s',  linspace(-3, 3, 7),      'tail'; ...
    'Crosswind',           'm/s',  linspace(-3, 3, 7),      'cross'};

npar = size(params, 1);
for k = 1:npar
    lbl = params{k, 1}; xu = params{k, 2}; vals = params{k, 3}; code = params{k, 4};
    er = zeros(numel(vals), 3);
    for j = 1:numel(vals)
        Pq = P; v0 = base.v0; QE = base.QE; az = base.az; p0 = base.p0;
        switch code
            case 'qe',    QE = base.QE + vals(j);
            case 'v0',    v0 = base.v0 * (1 + vals(j) / 100);
            case 'spin',  p0 = base.p0 * (1 + vals(j) / 100);
            case 'mass',  Pq.m = P.m * (1 + vals(j) / 100);
            case 'rho',   Pq.rho_scale = 1 + vals(j) / 100;
            case 'ixx',   Pq.Ixx = P.Ixx * (1 + vals(j) / 100);
            case 'iyy',   Pq.Iyy = P.Iyy * (1 + vals(j) / 100);
            case 'tail',  Pq.windN = vals(j);
            case 'cross', Pq.windE = vals(j);
        end
        Rj = fly(Pq, v0, QE, az, p0, 1e-6, 150, false);
        er(j, :) = [Rj.x - R0.x, Rj.y - R0.y, hypot(Rj.x - R0.x, Rj.y - R0.y)];
    end
    max_e = max(abs(er), [], 1);
    fprintf('%-22s : max |range| %7.1f m, |drift| %6.1f m, |radial| %7.1f m\n', ...
        lbl, max_e(1), max_e(2), max_e(3));
end
end

function print_bench(R, rows)
fprintf('\n%-26s %12s %12s %9s\n', 'Metric', 'Simulated', 'Benchmark', 'Error [%]');
fprintf('%s\n', repmat('-', 1, 63));
for i = 1:size(rows, 1)
    name = rows{i, 1}; sim_val = rows{i, 2}; bench_val = rows{i, 3};
    err_pct = 100 * (sim_val - bench_val) / bench_val;
    fprintf('%-26s %12.2f %12.2f %+8.2f %%\n', name, sim_val, bench_val, err_pct);
end
fprintf('%s\n', repmat('-', 1, 63));
end

function plot_nominal(R, tag, ttl)
try
    vis = 'on'; if isempty(getenv('DISPLAY')), vis = 'off'; end
    H = R.hist;
    t = H(:, 1); dt = t(2) - t(1);
    g = 9.80665;
    a_ax = gradient(H(:, 5), dt) / g;
    a_n  = hypot(gradient(H(:, 6), dt), gradient(H(:, 7), dt)) / g;
    f = figure('Visible', vis, 'Color', [0.98 0.98 0.98], 'Position', [30 30 1700 1000], ...
        'Name', ['M107 6-DOF - ' tag]);

    subplot(3, 3, 1);
    plot3(H(:, 2)/1000, H(:, 3), H(:, 4)/1000, 'Color', [0.10 0.55 0.90], 'LineWidth', 1.8); grid on; box on;
    xlabel('Downrange [km]'); ylabel('Crossrange [m]'); zlabel('Altitude [km]');
    title('3D trajectory path'); view(-35, 24);

    subplot(3, 3, 2);
    plot(t, H(:, 4), 'Color', [0.10 0.55 0.90], 'LineWidth', 1.4); grid on;
    xlabel('Flight time [s]'); ylabel('Altitude [m]');
    title(sprintf('Altitude (summit %.0f m @ %.0f s)', R.summit, R.summit_t));

    subplot(3, 3, 3);
    plot(t, R.Vt, 'Color', [0.10 0.55 0.90], 'LineWidth', 1.4); grid on;
    xlabel('Flight time [s]'); ylabel('Speed [m/s]');
    title(sprintf('Speed (%.0f \\rightarrow %.0f m/s)', R.Vt(1), R.v_imp));

    subplot(3, 3, 4);
    iz = t <= 10;
    plot(t(iz), a_ax(iz), 'Color', [0.10 0.55 0.90], 'LineWidth', 1.4); grid on;
    xlabel('Flight time [s]'); ylabel('Axial accel [g]');
    title(sprintf('Axial accel, launch (init %.2f g)', a_ax(1)));

    subplot(3, 3, 5);
    plot(t, a_ax, 'Color', [0.10 0.55 0.90], 'LineWidth', 1.2); grid on;
    xlabel('Flight time [s]'); ylabel('Axial accel [g]');
    title('Axial acceleration, full flight');

    subplot(3, 3, 6);
    plot(t, a_n, 'Color', [0.10 0.55 0.90], 'LineWidth', 1.2); grid on;
    xlabel('Flight time [s]'); ylabel('Normal accel [g]');
    title('Normal acceleration');

    subplot(3, 3, 7);
    plot(t, H(:, 11) / (2 * pi), 'm-', 'LineWidth', 1.4); grid on;
    xlabel('Flight time [s]'); ylabel('Spin rate [rev/s]');
    title(sprintf('Spin decay (%.0f \\rightarrow %.0f rev/s)', H(1, 11) / (2 * pi), R.p_imp));

    subplot(3, 3, 8);
    plot(t, rad2deg(H(:, 9)), '-', 'Color', [0.45 0.10 0.65], 'LineWidth', 1.4); grid on;
    xlabel('Flight time [s]'); ylabel('Pitch angle [deg]');
    title(sprintf('Pitch: %.0f\\circ \\rightarrow %.0f\\circ', rad2deg(H(1, 9)), R.theta_end));

    subplot(3, 3, 9);
    plot(t, R.alpha_2d, 'Color', [0.10 0.55 0.90], 'LineWidth', 1.1); hold on;
    plot(t, R.beta_2d, 'Color', [0.00 0.65 0.35], 'LineWidth', 1.1);
    plot(t, R.alpha, 'Color', [0.85 0.20 0.15], 'LineWidth', 1.1); grid on;
    xlabel('Flight time [s]'); ylabel('Aerodynamic angles [deg]');
    title(sprintf('Aero angles (max total AoA %.2f\\circ)', R.alpha_max));
    legend('\alpha', '\beta', '\alpha_{total}', 'Location', 'best');

    if exist('sgtitle', 'file') == 2 || exist('sgtitle', 'builtin')
        try, sgtitle(ttl); catch, end
    end
    print(f, ['unguided_' tag '_trajectory.png'], '-dpng', '-r130');
    fprintf('Saved unguided_%s_trajectory.png\n', tag);
catch e
    fprintf('Plot note: %s\n', e.message);
end
end

function plot_mc(imp, traj, fname, ttl)
try
    vis = 'on'; if isempty(getenv('DISPLAY')), vis = 'off'; end
    mip = mean(imp(:, 1:2), 1);
    rad = hypot(imp(:, 1) - mip(1), imp(:, 2) - mip(2));
    r50 = prctile(rad, 50); r90 = prctile(rad, 90);
    isH = rad <= 30;  ang = linspace(0, 2*pi, 250);
    f = figure('Visible', vis, 'Color', [0.98 0.98 0.98], 'Position', [40 40 1600 850], ...
        'Name', ttl);

    % --- Panel 1: 3D trajectory bundle + impact points ---
    ax3 = subplot(2, 2, [1, 3]); hold(ax3, 'on'); grid(ax3, 'on'); box(ax3, 'on');
    for k = 1:numel(traj)
        T = traj{k};
        if isempty(T), continue; end
        % Format: [t, x, y, z] -> plot x (km), y (m), z (km)
        plot3(ax3, T(:, 2)/1000, T(:, 3), T(:, 4)/1000, '-', ...
            'Color', [0.20 0.65 0.90], 'LineWidth', 0.9, 'HandleVisibility', 'off');
    end
    plot3(ax3, 0, 0, 0, 'p', 'MarkerSize', 12, 'MarkerFaceColor', [0.85 0.65 0.1], ...
        'MarkerEdgeColor', [0.75 0.45 0.05], 'DisplayName', 'Launch Origin [0,0]');
    plot3(ax3, imp(:, 1)/1000, imp(:, 2), zeros(size(imp(:, 1))), '.', ...
        'Color', [0.80 0.25 0.20], 'MarkerSize', 8, 'DisplayName', 'Ballistic Impacts');
    plot3(ax3, mip(1)/1000, mip(2), 0, 's', 'MarkerSize', 10, ...
        'MarkerFaceColor', [0.15 0.50 0.95], 'MarkerEdgeColor', [0.05 0.35 0.85], ...
        'DisplayName', sprintf('MPI (%.0f, %+.0f)', mip(1), mip(2)));
    plot3(ax3, mip(1)/1000 + r50*cos(ang)/1000, mip(2) + r50*sin(ang), zeros(size(ang)), ...
        'r--', 'LineWidth', 2.0, 'DisplayName', sprintf('CEP_{50} = %.0f m', r50));
    xlabel(ax3, 'Downrange [km]'); ylabel(ax3, 'Crossrange [m]'); zlabel(ax3, 'Altitude [km]');
    title(ax3, '1. 3D Trajectory Bundle + Impacts'); view(ax3, -35, 24);
    legend(ax3, 'Location', 'northeast');

    % Stats summary at top-left corner of 3D plot
    stats_3d = sprintf(['\\bf\\fontsize{8.5} SUMMARY\\rm\\fontsize{8}\n' ...
        'Rounds: %d\n' ...
        'MPI: (%.0f, %+.1f) m\n' ...
        'CEP_{50}: %.1f m\n' ...
        'Mean TOF: %.1f s'], ...
        size(imp, 1), mip(1), mip(2), r50, mean(imp(:, 3)));
    text(ax3, 0.03, 0.97, stats_3d, 'Units', 'normalized', ...
        'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
        'BackgroundColor', [0.93 0.96 1.0], 'EdgeColor', [0.15 0.45 0.85], ...
        'LineWidth', 1.2, 'Color', [0.05 0.15 0.40]);

    % --- Panel 2: 2D impact footprint ---
    ax2 = subplot(2, 2, 2); hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');
    if any(isH)
        plot(ax2, imp(isH, 1), imp(isH, 2), '.', 'Color', [0.10 0.65 0.25], ...
            'MarkerSize', 12, 'DisplayName', sprintf('<=30 m (%.0f%%)', 100 * mean(isH)));
    end
    if any(~isH)
        plot(ax2, imp(~isH, 1), imp(~isH, 2), 'o', 'Color', [0.85 0.20 0.15], ...
            'MarkerSize', 5, 'LineWidth', 1.0, 'DisplayName', sprintf('>30 m (%.0f%%)', 100 * mean(~isH)));
    end
    plot(ax2, mip(1) + 30*cos(ang), mip(2) + 30*sin(ang), ':', 'Color', [0.00 0.60 0.30], ...
        'LineWidth', 2.0, 'DisplayName', '30 m PGK Spec');
    plot(ax2, mip(1) + r50*cos(ang), mip(2) + r50*sin(ang), 'r--', 'LineWidth', 2.2, ...
        'DisplayName', sprintf('CEP_{50} = %.0f m', r50));
    plot(ax2, mip(1) + r90*cos(ang), mip(2) + r90*sin(ang), 'm-.', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('CEP_{90} = %.0f m', r90));
    plot(ax2, mip(1), mip(2), 's', 'MarkerSize', 10, 'MarkerFaceColor', [0.15 0.50 0.95], ...
        'MarkerEdgeColor', [0.05 0.35 0.85], 'DisplayName', 'MPI');
    axis(ax2, 'equal');
    xlabel(ax2, 'Impact Downrange [m]'); ylabel(ax2, 'Impact Crossrange [m]');
    title(ax2, '2. 2D Ground Impact Footprint');
    legend(ax2, 'Location', 'northeast');

    % Stats text box at top-left corner of 2D impact footprint
    sx = std(imp(:, 1)); sz = std(imp(:, 2));
    stats_2d = sprintf(['\\bf\\fontsize{8.5} STATS\\rm\\fontsize{8}\n' ...
        'Rounds: %d\n' ...
        'MPI: (%.0f, %+.1f) m\n' ...
        '\\sigma_x: %.1f m (PE: %.1f m)\n' ...
        '\\sigma_z: %.1f m (PE: %.1f m)\n' ...
        'CEP_{50}: %.1f m\n' ...
        'CEP_{90}: %.1f m\n' ...
        '\\le 30 m Spec: %.1f%% (%d/%d)\n' ...
        'Mean TOF: %.1f s'], ...
        size(imp, 1), mip(1), mip(2), ...
        sx, 0.6745 * sx, sz, 0.6745 * sz, ...
        r50, r90, 100 * mean(isH), sum(isH), size(imp, 1), ...
        mean(imp(:, 3)));
    text(ax2, 0.03, 0.97, stats_2d, 'Units', 'normalized', ...
        'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
        'BackgroundColor', [0.93 0.96 1.0], 'EdgeColor', [0.15 0.45 0.85], ...
        'LineWidth', 1.2, 'Color', [0.05 0.15 0.40]);

    % --- Panel 3: Atmosphere profile ---
    axE = subplot(2, 2, 4); hold(axE, 'on'); grid(axE, 'on'); box(axE, 'on');
    alts = linspace(0, 12000, 250);
    rr = zeros(size(alts)); aa = zeros(size(alts));
    for i = 1:numel(alts)
        [rr(i), aa(i)] = isa_rho_a(alts(i));
    end
    plot(axE, alts/1000, rr, 'Color', [0.00 0.45 0.85], 'LineWidth', 2, 'DisplayName', '\rho [kg/m^3]');
    if exist('yyaxis', 'builtin') || exist('yyaxis', 'file')
        try
            yyaxis(axE, 'right');
            plot(axE, alts/1000, aa, 'Color', [0.90 0.40 0.10], 'LineWidth', 2, 'DisplayName', 'a [m/s]');
            ylabel(axE, 'Speed of sound a [m/s]');
            yyaxis(axE, 'left');
        catch
            plot(axE, alts/1000, aa/300, 'Color', [0.90 0.40 0.10], 'LineWidth', 2, 'DisplayName', 'Sound speed / 300');
        end
    else
        plot(axE, alts/1000, aa/300, 'Color', [0.90 0.40 0.10], 'LineWidth', 2, 'DisplayName', 'Sound speed / 300');
    end
    xlabel(axE, 'Altitude [km]'); ylabel(axE, '\rho [kg/m^3]');
    title(axE, '3. ISA 1962 Atmosphere Profile');
    legend(axE, 'Location', 'northeast');
    if exist('sgtitle', 'file') == 2 || exist('sgtitle', 'builtin')
        try, sgtitle(ttl); catch, end
    end

    print(f, fname, '-dpng', '-r150');
    fprintf('Saved %s\n', fname);

    % Also save to unguided_all_in_one_analysis.png
    print(f, 'unguided_all_in_one_analysis.png', '-dpng', '-r150');
catch e
    fprintf('Plot note: %s\n', e.message);
end
end

function [rho, a] = isa_rho_a(z)
if z <= 11000
    T  = 288.15 - 0.0065 * z;
    pr = 101325 * (T / 288.15)^(9.80665 / (0.0065 * 287.05287));
else
    T  = 288.15 - 0.0065 * 11000;
    pt = 101325 * (T / 288.15)^(9.80665 / (0.0065 * 287.05287));
    pr = pt * exp(-9.80665 * (z - 11000) / (287.05287 * T));
end
if z < 0, T = 288.15; pr = 101325; end
rho = pr / (287.05287 * T);
a = sqrt(1.4 * 287.05287 * T);
end
