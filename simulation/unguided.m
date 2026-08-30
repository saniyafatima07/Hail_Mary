function varargout = unguided(mode, num_runs)
% =========================================================================
% UNGUIDED 155 mm M107 - PUBLISHED-DATA 6-DOF BALLISTIC SIMULATION (v2)
% =========================================================================
% Replaces v1 (unguided_v1_mpm.m - STANAG-4355 modified point mass with
% generic coefficients). Every constant, aero coefficient and validation
% number here is taken DIRECTLY from published, citable sources.
%
% WHY 6-DOF (not 7): the 7th DOF in the guidance papers (Cheng 2019,
% Raza & Wang 2022) is the DE-SPUN FUZE's independent roll angle gamma_c
% (dual-spin configuration). An unguided M107 is a single rigid body -
% 3 translations (x, y, z) + 3 rotations (roll phi, pitch theta, yaw psi)
% = 6 DOF = the complete exact model. Both papers simulate their
% unguided/reference trajectories in 6-DOF, and both COMMAND gamma_c
% kinematically in the guided case (Raza Eq. 10: perfect servo, no lag),
% so the 7th DOF's dynamics (servo lag, bearing friction) is exactly the
% unpublished control-loop residual to be measured on the HIL bench.
%
% MODEL - full 6-DOF rigid body in the non-rolling (aeroballistic) frame:
%   [1] McCoy, "Modern Exterior Ballistics", 2nd ed., ch. 9.
%   [2] Gkritzapis et al., "Modified Linear Theory for Spinning or
%       Non-Spinning Projectiles", Open Mech. J. 2:6-11 (2008),
%       Eqs. (3)-(17) - same frame, full force/moment set.
%   Forces/moments: axial drag (CD0 + CDa2*sin^2 a), normal-force lift,
%   Magnus force (Cmag_f), overturning moment (Cm_a), Magnus moment
%   (Cnp_a vs alpha), pitch damping (Cmq), spin damping (Clp).
%
% DATA - M107 mass properties and COMPLETE Mach-indexed aero tables:
%   [3] Khalil, Abdalla & Kamal, "Dispersion Analysis for Spinning
%       Artillery Projectile", ASAT-13, Cairo, May 2009 - Table 1
%       (SPINNER-98). Same data as the project's validated Python engine
%       (sim/engine_src - 10/10 published-benchmark tests passed).
%
% STANDARD CONDITIONS / ATMOSPHERE:
%   [4] Mady, Khalil & Yehia, J. Phys. Conf. Ser. 1507:082043 (2020):
%       ISA / US Std Atmosphere 1962 (T0=288.15 K, p0=101325 Pa,
%       rho0=1.225 kg/m^3, 6.5 K/km lapse to 11 km, isothermal above),
%       zero-wind standard conditions. Spin from rifling: p0=2*pi*v0/(eta*d).
%
% DISPERSION (Monte-Carlo uncertainty model):
%   [5] Khalil, Abdalla & Kamal, ASAT-13-FM-04 (2009), Table 2 - the
%       published uncertainty set, applied as 1-sigma Gaussians:
%       pitch +-0.4 deg, mass +-1%, air density +-4%, Ixx/Iyy +-2%,
%       muzzle velocity +-2%, spin +-2%, wind speed +-2 m/s,
%       wind direction +-2 deg; azimuth laying 0.15 deg (1 mil, gunnery).
%   A second "gunnery-realistic" tier uses firing-table-scale errors
%       (sigma_v0 = 2 m/s, 1 mil laying) for the indicative real CEP.
%
% VALIDATION ANCHORS (published numbers):
%   [K] Khalil 2009 sec. 4.3: v0=684.3 m/s, QE=44 deg, 175.48 rev/s ->
%       TOF 66.67 s, summit ~5750 m @ ~31 s, init axial decel 4.45 g,
%       impact ~330 m/s, max AoA ~1.3 deg, pitch 44 -> ~-55 deg.
%   [M] Mady 2020 Table 2 LIVE FIRE: G1 690.3 m/s, 141.4 mils -> 7943.2 m;
%       G2 692.7 m/s, 743.2 mils -> 18075.5 m. (Their drift column is the
%       residual AFTER gun laying correction - not an absolute-drift anchor.)
%   [C] Cheng 2019 Electronics 8:1135 Table 1: 930 m/s, 51 deg, 300 rev/s
%       -> nominal impact (29886, 1391) m - reference only (different shell).
%
% INTEGRATION: ode45 (Dormand-Prince RK45, compiled, adaptive) with a
%   terminal ground event - same method family as the validated Python
%   engine (scipy solve_ivp RK45, rtol=atol=1e-6). Convergence checked
%   against rtol 1e-9 in 'khalil' mode. Launch height 1 m (muzzle height;
%   avoids ground-event ambiguity at t=0, negligible effect).
%
% DOCUMENTED OMISSIONS (standard computational-firing-table practice):
%   Coriolis/Earth rotation (~30-60 m lateral at 18 km, mid-latitude -
%   Khalil 2022, AME 69:165-183, quantifies it), Earth curvature,
%   wind shear (constant wind with altitude per round).
%
% USAGE:
%   unguided                Monte-Carlo dispersion (default, both tiers)
%   unguided('khalil')      validate vs Khalil 2009 benchmark [K]
%   unguided('mady')        validate vs live-fire data [M]
%   unguided('cheng')       Cheng 2019 firing condition [C]
%   unguided('sens')        one-at-a-time sensitivity sweep (FM-04 Figs 10-21)
%   unguided('mc', 200)     more MC rounds
% =========================================================================

if nargin < 1 || isempty(mode), mode = 'mc'; end
if nargin < 2 || isempty(num_runs), num_runs = 100; end

%% P1. PUBLISHED PHYSICAL DATA - M107 155 mm HE -----------------------------
% Mass properties: Khalil, Abdalla & Kamal, ASAT-13 (2009) [3]
P.m   = 43.0;        % total mass [kg]
P.d   = 0.155;       % reference diameter [m]
P.Ixx = 0.144;       % axial moment of inertia [kg m^2]
P.Iyy = 1.216;       % transverse moment of inertia [kg m^2] (Izz = Iyy)
P.S   = pi/4*P.d^2;  % reference area [m^2]
P.g   = 9.80665;     % standard gravity [m/s^2]
P.eta = 25.16;         % implied rifling twist [calibers/turn] = v0/(p0*d)
                      % from Khalil Table 1 (684.3 m/s, 175.48 rev/s).
                      % NOTE: modern NATO 155 mm tubes run ~1/25; do NOT use
                      % the older 1/20 spec with Khalil's stated spin rate
                      % (1/20 would give 220.7 rev/s - inconsistent with the
                      % paper's Table 1). Spin is set from the paper's stated
                      % rev/s wherever available; eta is display/derived only.

% Aero tables - Khalil, Abdalla & Kamal, ASAT-13 (2009), Table 1
% (SPINNER-98). Linear interp vs Mach (clamped); Cnp_a bilinear (Mach,alpha)
P.Mgrid = [0.01 0.60 0.80 0.90 0.95 1.00 1.05 1.10 1.20 1.35 1.50 1.75 2.00];
P.CD0   = [0.144 0.144 0.146 0.167 0.221 0.327 0.383 0.381 0.370 0.353 0.338 0.314 0.294];
P.CDa2  = [2.343 2.343 2.847 3.372 3.730 4.180 4.691 5.209 5.702 5.130 4.561 3.970 3.460];
P.CLa   = [1.763 1.763 1.783 1.827 2.038 2.153 2.207 2.255 2.325 2.442 2.556 2.692 2.747];
P.Cmagf = [0.767 0.767 0.767 0.857 1.082 0.992 0.902 0.857 0.767 0.767 0.767 0.767 0.767];
P.Cspin = [-0.023 -0.023 -0.022 -0.021 -0.020 -0.020 -0.020 -0.019 -0.020 -0.020 -0.020 -0.020 -0.021];
P.CMa   = [3.355 3.378 3.571 3.957 3.886 3.682 3.415 3.384 3.424 3.278 3.264 3.201 3.013];
P.CMQ   = [-5.1 -5.1 -5.1 -7.4 -9.9 -13.8 -13.3 -14.6 -15.8 -15.6 -15.3 -15.3 -15.3];
P.Agrid = [0 2 5 10];      % alpha grid for Magnus moment [deg]
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

P.rho_scale = 1.0;   % density perturbation (MC)
P.windN = 0;         % downrange wind (+ = tail) [m/s]
P.windE = 0;         % crossrange wind (+ = right) [m/s]

switch lower(mode)
    case 'khalil', run_khalil(P);
    case 'mady',   run_mady(P);
    case 'cheng',  run_cheng(P);
    case 'sens',   run_sens(P);
    case 'mc',     run_mc(P, num_runs);
    case 'mccoy',  run_mccoy(P);
    case 'meteo',  run_meteo(P);
    case 'gatea',  run_gateA(P);
    case 'params', varargout = {P};
    case 'flyhook'
        a = num_runs;  % caller packed args into num_runs (2nd slot)
        varargout = {fly(a{1}, a{2}, a{3}, a{4}, a{5}, a{6}, a{7}, a{8})};
    case 'windsign', run_windsign(P);
    otherwise, error('Unknown mode "%s". Use: khalil | mady | cheng | mc', mode);
end
if nargout > 0 && isempty(varargout), varargout = {[]}; end
end

%% =========================================================================
function run_mccoy(P)
% McCoy, Modern Exterior Ballistics 2nd ed., Table 8.11:
% M107, Charge 8, v0 = 692 m/s, QE = 46 deg, 45 deg N latitude.
% Published no-Coriolis impact range ~17970 m (from the table's structure:
% AZ=0 N gives 17970 with 37 m Coriolis deflection). Our engine omits
% Coriolis by construction -> direct comparison.
p0 = 175.48*692/684.3;   % twist-consistent spin (1/25.2, Khalil anchor)
R = fly(P, 692.0, 46.0, 0, p0, 1e-6, 150, false);
fprintf('=== McCoy Table 8.11 anchor (v0=692, QE=46) ===\n');
fprintf('sim range        : %.0f m\n', R.x);
fprintf('McCoy published  : ~17970 m (no-Coriolis, Charge 8 max range)\n');
fprintf('difference       : %+.1f %%\n', 100*(R.x-17970)/17970);
fprintf('sim drift/TOF    : %.0f m / %.1f s (McCoy: drift incl. Coriolis 25-74 m at 45N)\n', R.y, R.tof);
end

function run_windsign(P)
% US-1 scenario 2 sign check: +E crosswind (windE=+3, all else standard)
% vs zero wind at C1. Records the drift-direction convention.
R0 = fly(P, 692.7, 743.2*360/6000, 0, 175.48*692.7/684.3, 1e-6, 200, false);
Pw = P; Pw.windE = 3.0;
Rw = fly(Pw, 692.7, 743.2*360/6000, 0, 175.48*692.7/684.3, 1e-6, 200, false);
Pn = P; Pn.windN = 3.0;
Rn = fly(Pn, 692.7, 743.2*360/6000, 0, 175.48*692.7/684.3, 1e-6, 200, false);
fprintf('C1 windE=+3 (cross): dz = %+.1f m, dx = %+.1f m\n', Rw.y-R0.y, Rw.x-R0.x);
fprintf('C1 windN=+3 (tail):  dz = %+.1f m, dx = %+.1f m\n', Rn.y-R0.y, Rn.x-R0.x);
R2 = fly(P, 930.0, 51.0, 0, 300.0, 1e-6, 200, false);
Pw2 = P; Pw2.windE = 3.0;
Rw2 = fly(Pw2, 930.0, 51.0, 0, 300.0, 1e-6, 200, false);
fprintf('C2 windE=+3 (cross): dz = %+.1f m, dx = %+.1f m\n', Rw2.y-R2.y, Rw2.x-R2.x);
end

function run_gateA(P)
% [A.2/A.5] regression gate: standard METEO message must reproduce the
% no-meteo engine BIT-IDENTICALLY at both primary conditions (US-1 S1).
ok = true;
conds = {struct('n','C1','v0',692.7,'QE',743.2*360/6000,'p0',175.48*692.7/684.3), ...
         struct('n','C2','v0',930.0,'QE',51.0,'p0',300.0)};
for ci = 1:numel(conds)
    c = conds{ci};
    Ra = fly(P,  c.v0, c.QE, 0, c.p0, 1e-6, 200, false);
    Pm = P; Pm.meteo = meteo('standard');
    Rb = fly(Pm, c.v0, c.QE, 0, c.p0, 1e-6, 200, false);
    same = (abs(Ra.x-Rb.x) < 1e-9) && (abs(Ra.y-Rb.y) < 1e-9) && (abs(Ra.tof-Rb.tof) < 1e-9);
    fprintf('%s regression: dx=%.2e dz=%.2e dt=%.2e -> %s\n', ...
        c.n, Rb.x-Ra.x, Rb.y-Ra.y, Rb.tof-Ra.tof, string(same));
    ok = ok && same;
end
fprintf('=== A.2 REGRESSION GATE (both conditions): %s ===\n', string(ok));
end

function run_meteo(P)
% [A.4] METEO demo: jetstream message vs standard, at BOTH primary
% test conditions (spec §0.8: C1 = 692.7 m/s charge-8 anchor,
% C2 = 930 m/s fleet-realistic). Prints the before/after impact shift.
Mj = meteo('jetstream');
Pm = P; Pm.meteo = Mj;
cond = {struct('name','C1 (692.7 m/s, 44.6 deg)','v0',692.7,'QE',743.2*360/6000,'p0',175.48*692.7/684.3), ...
        struct('name','C2 (930 m/s, 51 deg)',   'v0',930.0,'QE',51.0,      'p0',300.0)};
fprintf('=== METEO demo: jetstream message vs standard atmosphere ===\n');
fprintf('%-28s %10s %10s %10s %10s %10s\n', 'Condition', 'dx [m]', 'dz [m]', 'std range', 'met range', 'TOF chg');
for ci = 1:numel(cond)
    c = cond{ci};
    R0 = fly(P,  c.v0, c.QE, 0, c.p0, 1e-6, 200, false);
    R1 = fly(Pm, c.v0, c.QE, 0, c.p0, 1e-6, 200, false);
    fprintf('%-28s %+10.0f %+10.0f %10.0f %10.0f %+9.1fs\n', ...
        c.name, R1.x-R0.x, R1.y-R0.y, R0.x, R1.x, R1.tof-R0.tof);
end
[wb_n, wb_e, wb_t] = meteo(Mj, 'ballistic_wind', 11000);
fprintf('Message column means (0-11 km): windN=%+.1f, windE=%+.1f, dT=%+.1f K\n', wb_n, wb_e, wb_t);
plot_meteo_demo(P, Pm, cond{1});
end

function plot_meteo_demo(P, Pm, c)
try
    vis = 'on'; if isempty(getenv('DISPLAY')), vis = 'off'; end
    R0 = fly(P,  c.v0, c.QE, 0, c.p0, 1e-6, 200, true);
    R1 = fly(Pm, c.v0, c.QE, 0, c.p0, 1e-6, 200, true);
    f = figure('Visible', vis, 'Color', 'w', 'Position', [40 40 1500 480], ...
               'Name', 'METEO demo (jetstream vs standard)');
    subplot(1,3,1);
    plot(R0.hist(:,2)/1000, R0.hist(:,4)/1000, 'b-', 'LineWidth', 1.6); hold on;
    plot(R1.hist(:,2)/1000, R1.hist(:,4)/1000, 'r-', 'LineWidth', 1.6); grid on;
    xlabel('Downrange [km]'); ylabel('Altitude [km]');
    legend('standard', 'jetstream met', 'Location', 'southwest');
    title(sprintf('Trajectory — %s', c.name));
    subplot(1,3,2);
    plot(R0.hist(:,1), R0.hist(:,3), 'b-', 'LineWidth', 1.4); hold on;
    plot(R1.hist(:,1), R1.hist(:,3), 'r-', 'LineWidth', 1.4); grid on;
    xlabel('t [s]'); ylabel('crossrange [m]');
    legend('standard', 'jetstream met');
    title('Drift — wind shear visible');
    subplot(1,3,3);
    Mj = Pm.meteo; zs = 0:100:16000; wn = zeros(size(zs)); we = wn;
    for i = 1:numel(zs)
        [~,~,wn(i),we(i)] = meteo(Mj, zs(i));
    end
    plot(we, zs/1000, 'r-', wn, zs/1000, 'b--', 'LineWidth', 1.4); grid on;
    xlabel('wind [m/s]'); ylabel('altitude [km]');
    legend('windE (cross)', 'windN (downrange)');
    title('METEO message profile');
    print(f, 'unguided_meteo_demo.png', '-dpng', '-r130');
    fprintf('Saved unguided_meteo_demo.png\n');
catch e
    fprintf('Plot note: %s\n', e.message);
end
end

function run_khalil(P)
fprintf('=== VALIDATION vs Khalil/Abdalla/Kamal ASAT-13 (2009) sec 4.3 ===\n');
v0 = 684.3; QE = 44.0; p0 = 175.48;               % published condition [K]
R = fly(P, v0, QE, 0, p0, 1e-6, 150, true);
% cross-check probe vs the validated Python engine
if isfield(R,'sol')
    fprintf('PROBE  t      x        y        z        u        v        w       theta_deg  p_rps\n');
    for tq = [1 2 5 10 20]
        yq = deval(R.sol, tq);
        fprintf('PROBE  %5.1f %8.1f %8.2f %8.1f %8.1f %8.3f %8.3f %8.2f %8.1f\n', ...
            tq, yq(1), yq(2), yq(3), yq(4), yq(5), yq(6), rad2deg(yq(8)), yq(10)/(2*pi));
    end
end
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
% convergence check
R2 = fly(P, v0, QE, 0, p0, 1e-9, 150, false);
fprintf('Convergence: rtol 1e-6 -> %.1f m / %.3f s | rtol 1e-9 -> %.1f m / %.3f s\n', ...
    R.x, R.tof, R2.x, R2.tof);
plot_nominal(R, 'khalil', sprintf('Khalil 2009 benchmark: v_0=684.3 m/s, QE=44\\circ, 175.5 rev/s'));
end

%% =========================================================================
function run_mady(P)
fprintf('=== VALIDATION vs Mady 2020 LIVE-FIRE Table 2 (155 mm M107 HE) ===\n');
fprintf('%-34s %10s %10s %9s\n', 'Case (QE mil convention)', 'sim [m]', 'live [m]', 'diff [%]');
groups = [struct('v0',690.3,'mil',141.4,'live',7943.2), struct('v0',692.7,'mil',743.2,'live',18075.5)];
for conv = [struct('name','Soviet 6000','circle',6000), struct('name','NATO 6400','circle',6400)]
    for g = groups
        QE = g.mil * 360 / conv.circle;
        p0 = 175.48 * g.v0/684.3;   % spin SCALED with muzzle velocity at
                                     % constant twist (1/25.2, from Khalil Table 1:
                                     % 175.48 rev/s @ 684.3 m/s). Using the old
                                     % 1/20 label here would give ~221 rev/s.
        R  = fly(P, g.v0, QE, 0, p0, 1e-6, 150, false);
        fprintf('G(%.1f mil = %5.2f deg, %-11s) %10.0f %10.0f %+8.2f\n', ...
            g.mil, QE, conv.name, R.x, g.live, 100*(R.x-g.live)/g.live);
    end
end
fprintf('NOTE: live-fire drift (27.2 / 11.9 m) is the residual AFTER gun\n');
fprintf('laying correction - not an absolute-drift anchor. Simulated absolute\n');
fprintf('drift at G2: see ''cheng''/''khalil'' modes; firing-table M107 drift at\n');
fprintf('18 km is ~10-15 mils (~200-300 m) - flagged as an open validation item.\n');
end

%% =========================================================================
function run_cheng(P)
fprintf('=== Cheng 2019 (Electronics 8:1135) firing condition, M107 aero ===\n');
v0 = 930.0; QE = 51.0; p0 = 300.0;
R = fly(P, v0, QE, 0, p0, 1e-6, 200, true);
fprintf('M107 with Cheng''s firing condition : impact (%.0f, %+.0f) m, TOF %.1f s\n', R.x, R.y, R.tof);
fprintf('Cheng''s shell (Chinese ERFB-type)  : (29886, +1391) m, TOF ~90 s\n');
fprintf('-> Different shell (Cheng never publishes mass/aero); structural\n');
fprintf('   comparison only. Quantitative validation = khalil/mady modes.\n');
plot_nominal(R, 'cheng', sprintf('Cheng 2019 condition on M107 aero: v_0=930 m/s, QE=51\\circ, 300 rev/s'));
end

%% =========================================================================
function run_mc(P, n)
fprintf('=== MC DISPERSION: M107, Mady live-fire G2 (692.7 m/s, QE 44.6 deg) ===\n');
v0n = 692.7; QEn = 743.2*360/6000; p0n = 175.48*v0n/684.3;  % Khalil twist-consistent spin

% ---- Tier 1: published Khalil ASAT-13 Table 2 uncertainty set ----
[imp1, traj1] = mc_loop(P, n, v0n, QEn, p0n, ...
    struct('v0_pct',0.02,'qe_deg',0.4,'az_deg',0.15,'p_pct',0.02, ...
           'm_pct',0.01,'ixx_pct',0.02,'iyy_pct',0.02,'rho_pct',0.04, ...
           'wspd',2.0,'wdir',2.0), 'Khalil Table-2 set');
report(imp1, n, 'TIER 1 - published Khalil ASAT-13 Table 2 uncertainties');
plot_mc(imp1, traj1, 'unguided_6dof_mc_tier1.png', ...
    sprintf('M107 6-DOF dispersion - Khalil Table-2 uncertainty set (%d rounds)', n));

% ---- Tier 2: gunnery-realistic (firing-table scale) ----
[imp2, ~] = mc_loop(P, n, v0n, QEn, p0n, ...
    struct('v0_pct',2.0/692.7,'qe_deg',0.0573,'az_deg',0.0573,'p_pct',0.005, ...
           'm_pct',0.005,'ixx_pct',0.01,'iyy_pct',0.01,'rho_pct',0.01, ...
           'wspd',2.0,'wdir',2.0), 'gunnery-realistic set');
report(imp2, n, 'TIER 2 - gunnery-realistic (sigma_v0=2 m/s, 1 mil laying)');
plot_mc(imp2, [], 'unguided_6dof_mc_tier2.png', ...
    sprintf('M107 6-DOF dispersion - gunnery-realistic errors (%d rounds)', n));
end

function [imp, traj] = mc_loop(P, n, v0n, QEn, p0n, U, label)
rng(42);
imp = zeros(n,3); traj = cell(min(n,25),1);
t0 = tic;
for i = 1:n
    Pq = P;
    Pq.m        = P.m   * (1 + U.m_pct*randn);
    Pq.Ixx      = P.Ixx * (1 + U.ixx_pct*randn);
    Pq.Iyy      = P.Iyy * (1 + U.iyy_pct*randn);
    Pq.rho_scale= 1 + U.rho_pct*randn;
    ws = U.wspd*randn; wd = U.wdir*randn;
    Pq.windN = ws*cosd(wd); Pq.windE = ws*sind(wd);
    v0 = v0n*(1 + U.v0_pct*randn);
    QE = QEn + U.qe_deg*randn;
    az = U.az_deg*randn;
    p0 = p0n*(1 + U.p_pct*randn);
    R = fly(Pq, v0, QE, az, p0, 1e-6, 150, i <= 25);
    imp(i,:) = [R.x, R.y, R.tof];
    if i <= 25, traj{i} = R.hist; end
    if mod(i,10)==0, fprintf('  [%s] %3d/%d  (%.0f s)\n', label, i, n, toc(t0)); end
end
end

function report(imp, n, title)
mip = mean(imp(:,1:2),1);
rad = hypot(imp(:,1)-mip(1), imp(:,2)-mip(2));
sx = std(imp(:,1)); sz = std(imp(:,2));
fprintf('\n--- %s ---\n', title);
fprintf('Rounds                      : %d\n', n);
fprintf('Mean point of impact (MPI)  : (%.0f, %+.1f) m\n', mip(1), mip(2));
fprintf('Sigma range   sx            : %6.1f m  (PE %5.1f m)\n', sx, 0.6745*sx);
fprintf('Sigma cross   sz            : %6.1f m  (PE %5.1f m)\n', sz, 0.6745*sz);
fprintf('CEP50 about MPI             : %6.1f m\n', prctile(rad,50));
fprintf('CEP90 about MPI             : %6.1f m\n', prctile(rad,90));
fprintf('Rounds within 30 m of MPI   : %5.1f %%\n', 100*mean(rad<=30));
fprintf('Mean TOF                    : %6.1f s\n', mean(imp(:,3)));
end

%% =========================================================================
%% FLIGHT PROPAGATOR - ode45 (Dormand-Prince RK45) + terminal ground event
%% =========================================================================
function R = fly(P, v0, QE_deg, az_deg, p0_rps, rtol, tmax, record)
% state y = [x; y; z; u; v; w; phi; theta; psi; p; q; r]  (COLUMN vector)
%  x downrange, y crossrange(+right), z altitude(+up) - inertial
%  u,v,w velocities in the non-rolling frame (x along symmetry axis)
%  phi body roll (decoupled), theta pitch, psi yaw; p spin, q,r rates
y0 = zeros(12,1);
y0(3)   = 1.0;              % muzzle height 1 m (event safety, negligible)
y0(4)   = v0;               % launch along the symmetry axis, no initial yaw
y0(8)   = deg2rad(QE_deg);
y0(9)   = deg2rad(az_deg);
y0(10)  = 2*pi*p0_rps;

k1 = deriv(0, y0, P);
R.ax0 = k1(4);              % initial axial deceleration (4.45 g check)

opts = odeset('RelTol', rtol, 'AbsTol', rtol, ...
              'Events', @(t,y) ground_event(t,y), 'Refine', 1);
sol = ode45(@(t,y) deriv(t, y, P), [0 tmax], y0, opts);

if ~isempty(sol.xe)         % ground impact event
    ye = sol.ye(:,1);
    R.tof = sol.xe(1);
    R.x = ye(1); R.y = ye(2);
    R.v_imp = norm(ye(4:6));
    R.p_imp = ye(10)/(2*pi);
    R.theta_end = rad2deg(ye(8));
else
    ye = sol.y(:,end);
    R.tof = sol.x(end); R.x = ye(1); R.y = ye(2);
    R.v_imp = norm(ye(4:6)); R.p_imp = ye(10)/(2*pi);
    R.theta_end = rad2deg(ye(8));
    fprintf('WARNING: no ground event - trajectory truncated at t=%.1f s\n', R.tof);
end

% summary history (uniform-ish sampling from the adaptive solution)
tt = linspace(0, R.tof, 800);
Yt = deval(sol, tt);
[R.summit, isu] = max(Yt(3,:));
R.summit_t = tt(isu);
Vt = sqrt(sum(Yt(4:6,:).^2, 1));
ca = max(-1, min(1, Yt(4,:)./Vt));
R.alpha = rad2deg(acos(ca));            % total AoA (FM-04 Fig 9, alpha_bar)
R.alpha_2d = rad2deg(atan2(Yt(6,:), Yt(4,:)));  % pitch-plane alpha (Fig 9)
R.beta_2d  = rad2deg(atan2(Yt(5,:), Yt(4,:)));  % yaw-plane beta  (Fig 9)
R.alpha_max = max(R.alpha);
R.Vt = Vt; R.Yt = Yt; R.tt = tt;
if record
    R.hist = [tt; Yt]';     % (N x 13): [t, state...]
else
    R.hist = [];
end
R.sol = sol;
end

function [val, isterm, dir] = ground_event(~, y)
val = y(3); isterm = 1; dir = -1;
end

%% =========================================================================
%% STATE DERIVATIVE - 6-DOF, non-rolling aeroballistic frame (fully inlined)
%%   McCoy ch.9 / Gkritzapis 2008 Eqs (3)-(17) / Khalil ASAT-13 formulation
%% =========================================================================
function d = deriv(~, s, P) %#ok<INUSD>
u = s(4); v = s(5); w = s(6);
th = s(8); ps = s(9);
p = s(10); q = s(11); r = s(12);
z = s(3);

% ---- atmosphere + wind: METEO message if present, else ISA + const wind
% [A.2] When P.meteo is absent the branch below is arithmetic-identical
% to the original inline ISA + P.windN/P.windE (regression gate, US-1 S1).
if isfield(P, 'meteo') && ~isempty(P.meteo)
    [rho_u, a_snd, windN, windE] = meteo(P.meteo, z);
    rho = P.rho_scale * rho_u;
else
    if z <= 11000
        T  = 288.15 - 0.0065*z;
        pr = 101325*(T/288.15)^(9.80665/(0.0065*287.05287));
    else
        T  = 288.15 - 0.0065*11000;
        pt = 101325*(T/288.15)^(9.80665/(0.0065*287.05287));
        pr = pt*exp(-9.80665*(z-11000)/(287.05287*T));
    end
    if z < 0, T = 288.15; pr = 101325; end
    rho = P.rho_scale*pr/(287.05287*T);
    a_snd = sqrt(1.4*287.05287*T);
    windN = P.windN;
    windE = P.windE;
end

% ---- non-rolling frame DCM (roll = 0), inertial = (x, y, z up) ----
cth = cos(th); sth = sin(th); cps = cos(ps); sps = sin(ps);

% ---- air-relative velocity in the frame: v_air = [u;v;w] - DCM'*wind ----
vax = u - ( cth*cps*windN + cth*sps*windE);
vay = v - (-sps*windN     + cps*windE);
vaz = w + sth*(cps*windN + sps*windE);
V = sqrt(vax*vax + vay*vay + vaz*vaz);
if V < 1e-3, V = 1e-3; end
Mach = V/a_snd;
evx = vax/V; evy = vay/V; evz = vaz/V;

% ---- total angle of attack + in-plane unit directions ----
ca = evx; if ca > 1, ca = 1; end; if ca < -1, ca = -1; end
alpha = acos(ca); sa = sin(alpha);
sal = sqrt(evy*evy + evz*evz);          % = sin(alpha)
if sal > 1e-9
    mx = 0;      my = -evz/sal;  mz = evy/sal;    % Magnus/overturning axis
    lx = sal;    ly = -evx*evy/sal; lz = -evx*evz/sal;  % lift dir (toward nose)
else
    mx = 0; my = 0; mz = 0; lx = 0; ly = 0; lz = 0;
end

% ---- aero coefficients (Khalil Table 1, inlined bracket) ----
Mgrid = P.Mgrid;
mi = 1 + sum(Mach > Mgrid(2:end-1));  mi = min(max(mi,1),12);
mt = (Mach - Mgrid(mi))/(Mgrid(mi+1)-Mgrid(mi));  mt = min(max(mt,0),1);
CD0   = P.CD0(mi)   + mt*(P.CD0(mi+1)  -P.CD0(mi));
CDa2  = P.CDa2(mi)  + mt*(P.CDa2(mi+1) -P.CDa2(mi));
CLa   = P.CLa(mi)   + mt*(P.CLa(mi+1)  -P.CLa(mi));
CMQ   = P.CMQ(mi)   + mt*(P.CMQ(mi+1)  -P.CMQ(mi));
CMa   = P.CMa(mi)   + mt*(P.CMa(mi+1)  -P.CMa(mi));
Cspin = P.Cspin(mi) + mt*(P.Cspin(mi+1)-P.Cspin(mi));
Cmagf = P.Cmagf(mi) + mt*(P.Cmagf(mi+1)-P.Cmagf(mi));
adeg = rad2deg(alpha);
Agrid = P.Agrid;
aj = 1 + sum(adeg > Agrid(2:end-1));  aj = min(max(aj,1),3);
at = (adeg - Agrid(aj))/(Agrid(aj+1)-Agrid(aj));  at = min(max(at,0),1);
Cnpa = (1-mt)*((1-at)*P.Cnpa(mi,aj) + at*P.Cnpa(mi,aj+1)) ...
     +    mt*((1-at)*P.Cnpa(mi+1,aj) + at*P.Cnpa(mi+1,aj+1));

% ---- forces ----
qbar = 0.5*rho*V*V;  S = P.S;  dd = P.d;
spf = p*dd/(2*V);
cda = CD0 + CDa2*sa*sa;
kD  = qbar*S*cda;          % drag magnitude
kL  = qbar*S*CLa*sa;       % lift magnitude
kMf = qbar*S*Cmagf*spf*sa; % Magnus force magnitude
Fx = -kD*evx + kL*lx + kMf*mx - P.m*P.g*sth;
Fy = -kD*evy + kL*ly + kMf*my;
Fz = -kD*evz + kL*lz + kMf*mz - P.m*P.g*cth;

% ---- moments ----
kOv = -qbar*S*dd*CMa*sa;         % overturning (Cm_a>0 = destabilizing)
kMg = -qbar*S*dd*Cnpa*spf;       % Magnus moment (alpha baked into table)
kPd = qbar*S*dd*dd/(2*V)*CMQ;    % pitch damping factor
Mx = (kOv+kMg)*mx + qbar*S*dd*spf*Cspin;
My = (kOv+kMg)*my + kPd*q;
Mz = (kOv+kMg)*mz + kPd*r;

% ---- translational EOM (transport theorem; frame Omega = (0,q,r)) ----
ud = -q*w + r*v + Fx/P.m;
vd = -r*u          + Fy/P.m;
wd =  q*u          + Fz/P.m;

% ---- rotational EOM (symmetric top, Iyy = Izz) ----
pd = Mx/P.Ixx;
qd = (My - P.Ixx*p*r)/P.Iyy;
rd = (Mz + P.Ixx*p*q)/P.Iyy;

% ---- kinematics ----
pxd = cth*cps*u - sps*v - cps*sth*w;
pyd = cth*sps*u + cps*v - sps*sth*w;
pzd = sth*u + cth*w;
cthc = cth; if abs(cthc) < 1e-6, cthc = 1e-6*sign(cthc+(cthc==0)); end

d = [pxd; pyd; pzd; ud; vd; wd; p; -q; r/cthc; pd; qd; rd];
end

%% =========================================================================
%% OUTPUT HELPERS
%% =========================================================================
function print_bench(R, rows)
fprintf('%-26s %12s %12s\n', 'Quantity', 'simulated', 'published');
for k = 1:size(rows,1)
    fprintf('%-26s %12.2f %12.2f\n', rows{k,1}, rows{k,2}, rows{k,3});
end
end

function plot_nominal(R, tag, ttl)
% FM-04-style figure set (their Figs 2-9): 3D path, altitude, velocity,
% axial accel (launch zoom + full), normal accel, spin, pitch, aero angles.
try
    vis = 'on'; if isempty(getenv('DISPLAY')), vis = 'off'; end
    H = R.hist;                       % [t, x, y, z, u, v, w, phi, th, psi, p, q, r]
    t = H(:,1); dt = t(2)-t(1);
    g = 9.80665;
    a_ax = gradient(H(:,5), dt)/g;                  % axial accel [g] (du/dt)
    a_n  = hypot(gradient(H(:,6), dt), gradient(H(:,7), dt))/g;  % normal [g]
    f = figure('Visible', vis, 'Color', 'w', 'Position', [30 30 1700 1000], ...
               'Name', ['M107 6-DOF - ' tag]);

    subplot(3,3,1);   % FM-04 Fig 2: 3D trajectory
    plot3(H(:,2)/1000, H(:,3), H(:,4)/1000, 'b-', 'LineWidth', 1.8); grid on; box on;
    xlabel('Downrange [km]'); ylabel('Crossrange [m]'); zlabel('Altitude [km]');
    title('3D trajectory path'); view(-35, 24);

    subplot(3,3,2);   % FM-04 Fig 3: altitude vs time
    plot(t, H(:,4), 'b-', 'LineWidth', 1.4); grid on;
    xlabel('Flight time [s]'); ylabel('Altitude [m]');
    title(sprintf('Altitude (summit %.0f m @ %.0f s)', R.summit, R.summit_t));

    subplot(3,3,3);   % FM-04 Fig 4: velocity vs time
    plot(t, R.Vt, 'b-', 'LineWidth', 1.4); grid on;
    xlabel('Flight time [s]'); ylabel('Velocity magnitude [m/s]');
    title(sprintf('Speed (%.0f \\rightarrow %.0f m/s)', R.Vt(1), R.v_imp));

    subplot(3,3,4);   % FM-04 Fig 5a: axial accel, launch zoom
    iz = t <= 10;
    plot(t(iz), a_ax(iz), 'b-', 'LineWidth', 1.4); grid on;
    xlabel('Flight time [s]'); ylabel('Axial acceleration [g]');
    title(sprintf('Axial accel, launch (init %.2f g)', a_ax(1)));

    subplot(3,3,5);   % FM-04 Fig 5b: axial accel, full flight
    plot(t, a_ax, 'b-', 'LineWidth', 1.2); grid on;
    xlabel('Flight time [s]'); ylabel('Axial acceleration [g]');
    title('Axial acceleration, full flight');

    subplot(3,3,6);   % FM-04 Fig 6: normal accel
    plot(t, a_n, 'b-', 'LineWidth', 1.2); grid on;
    xlabel('Flight time [s]'); ylabel('Normal acceleration [g]');
    title('Normal acceleration');

    subplot(3,3,7);   % FM-04 Fig 7: spin rate
    plot(t, H(:,11)/(2*pi), 'm-', 'LineWidth', 1.4); grid on;
    xlabel('Flight time [s]'); ylabel('Spin rate [rev/s]');
    title(sprintf('Spin decay (%.0f \\rightarrow %.0f rev/s)', H(1,11)/(2*pi), R.p_imp));

    subplot(3,3,8);   % FM-04 Fig 8: pitch angle
    plot(t, rad2deg(H(:,9)), 'k-', 'LineWidth', 1.4); grid on;
    xlabel('Flight time [s]'); ylabel('Pitch angle [deg]');
    title(sprintf('Pitch: %.0f\\circ \\rightarrow %.0f\\circ', rad2deg(H(1,9)), R.theta_end));

    subplot(3,3,9);   % FM-04 Fig 9: aerodynamic angles
    plot(t, R.alpha_2d, 'b-', t, R.beta_2d, 'g-', t, R.alpha, 'r-', 'LineWidth', 1.1); grid on;
    xlabel('Flight time [s]'); ylabel('Aerodynamic angles [deg]');
    title(sprintf('Aero angles (max total AoA %.2f\\circ)', R.alpha_max));
    legend('\alpha', '\beta', '\alpha_{total}', 'Location', 'best');

    sgtitle(ttl);
    print(f, ['unguided_' tag '_trajectory.png'], '-dpng', '-r130');
    fprintf('Saved unguided_%s_trajectory.png\n', tag);
catch e
    fprintf('Plot note: %s\n', e.message);
end
end

%% =========================================================================
%% ONE-AT-A-TIME SENSITIVITY - FM-04 Figs 10-21 methodology
%%   Sweep each Table-2 uncertainty parameter across its published range,
%%   plot range/drift/radial error vs the parameter (their Fig style).
%% =========================================================================
function run_sens(P)
% One-at-a-time sensitivity: FM-04 Table 2 parameter ranges, their
% Figs 10-21 presentation (range error / drift error / radial error).
fprintf('=== SENSITIVITY SWEEP (FM-04 Table 2 ranges, Figs 10-21 style) ===\n');
base.v0 = 692.7; base.QE = 743.2*360/6000; base.p0 = 175.48*692.7/684.3; base.az = 0;
R0 = fly(P, base.v0, base.QE, base.az, base.p0, 1e-6, 150, false);
fprintf('Nominal: range %.0f m, drift %+.1f m, TOF %.1f s\n\n', R0.x, R0.y, R0.tof);

params = { ...  % {label, x-unit, sweep values, code}
 'Launch pitch angle',  'deg',  linspace(-0.4,0.4,7),  'qe'; ...
 'Muzzle velocity',     '%',    linspace(-2,2,7),      'v0'; ...
 'Spin rate',           '%',    linspace(-2,2,7),      'spin'; ...
 'Total mass',          '%',    linspace(-1,1,7),      'mass'; ...
 'Air density',         '%',    linspace(-4,4,7),      'rho'; ...
 'Axial inertia Ixx',   '%',    linspace(-2,2,7),      'ixx'; ...
 'Lateral inertia Iyy', '%',    linspace(-2,2,7),      'iyy'; ...
 'Head/tail wind',      'm/s',  linspace(-3,3,7),      'tail'; ...
 'Crosswind',           'm/s',  linspace(-3,3,7),      'cross'};

npar = size(params,1);
maxerr = zeros(npar, 3);
try
    vis = 'on'; if isempty(getenv('DISPLAY')), vis = 'off'; end
    f = figure('Visible', vis, 'Color', 'w', 'Position', [30 30 1700 1000], ...
               'Name', 'FM-04-style sensitivity sweep');
    for k = 1:npar
        lbl = params{k,1}; xu = params{k,2}; vals = params{k,3}; code = params{k,4};
        er = zeros(numel(vals), 3);
        for j = 1:numel(vals)
            Pq = P; v0 = base.v0; QE = base.QE; az = base.az; p0 = base.p0;
            switch code
                case 'qe',    QE = base.QE + vals(j);
                case 'v0',    v0 = base.v0*(1 + vals(j)/100);
                case 'spin',  p0 = base.p0*(1 + vals(j)/100);
                case 'mass',  Pq.m = P.m*(1 + vals(j)/100);
                case 'rho',   Pq.rho_scale = 1 + vals(j)/100;
                case 'ixx',   Pq.Ixx = P.Ixx*(1 + vals(j)/100);
                case 'iyy',   Pq.Iyy = P.Iyy*(1 + vals(j)/100);
                case 'tail',  Pq.windN = vals(j);
                case 'cross', Pq.windE = vals(j);
            end
            Rj = fly(Pq, v0, QE, az, p0, 1e-6, 150, false);
            er(j,:) = [Rj.x-R0.x, Rj.y-R0.y, hypot(Rj.x-R0.x, Rj.y-R0.y)];
        end
        maxerr(k,:) = max(abs(er),[],1);
        subplot(3,3,k); hold on; grid on; box on;
        plot(vals, er(:,1), 'b-o', 'LineWidth', 1.3, 'MarkerSize', 4);
        plot(vals, er(:,2), 'r-s', 'LineWidth', 1.3, 'MarkerSize', 4);
        plot(vals, er(:,3), 'k-^', 'LineWidth', 1.3, 'MarkerSize', 4);
        xlabel([lbl ' [' xu ']']); ylabel('Errors [m]');
        title(lbl, 'Interpreter', 'none');
        if k == 1
            legend('Range error', 'Drift error', 'Radial error', 'Location', 'best');
        end
        fprintf('%-22s : max |range| %7.1f m, |drift| %6.1f m, |radial| %7.1f m\n', ...
            lbl, maxerr(k,1), maxerr(k,2), maxerr(k,3));
    end
    sgtitle(sprintf('One-at-a-time sensitivity (FM-04 Figs 10-21 style) - nominal %.0f m range', R0.x));
    print(f, 'unguided_sensitivity_fm04style.png', '-dpng', '-r130');
    fprintf('Saved unguided_sensitivity_fm04style.png\n');
    [~, im] = sort(maxerr(:,3), 'descend');
    fprintf('\nDominant parameters (max radial error):\n');
    for qk = 1:min(3,npar)
        fprintf('  %d. %s (%.0f m)\n', qk, params{im(qk),1}, maxerr(im(qk),3));
    end
catch e
    fprintf('Plot note: %s\n', e.message);
end
end

function plot_mc(imp, traj, fname, ttl)
% v1-style unified 3-panel display: 3D trajectory bundle + impacts,
% 2D footprint with CEP circles, atmosphere profile.
try
    vis = 'on'; if isempty(getenv('DISPLAY')), vis = 'off'; end
    mip = mean(imp(:,1:2),1);
    rad = hypot(imp(:,1)-mip(1), imp(:,2)-mip(2));
    r50 = prctile(rad,50); r90 = prctile(rad,90);
    isH = rad <= 30;  ang = linspace(0,2*pi,250);
    f = figure('Visible', vis, 'Color', 'w', 'Position', [40 40 1600 850], ...
               'Name', ttl);
    % --- panel 1+3: 3D trajectory bundle + impact points ---
    ax3 = subplot(2,2,[1 3]); hold(ax3,'on'); grid(ax3,'on'); box(ax3,'on');
    for k = 1:numel(traj)
        T = traj{k};
        if isempty(T), continue; end
        plot3(ax3, T(:,2)/1000, T(:,3), T(:,4)/1000, '-', ...
              'Color', [0.55 0.55 0.60], 'LineWidth', 0.8, 'HandleVisibility', 'off');
    end
    plot3(ax3, 0, 0, 0, 'p', 'MarkerSize', 12, 'MarkerFaceColor', [0.85 0.65 0.1], ...
          'DisplayName', 'Launch [0,0]');
    plot3(ax3, imp(:,1)/1000, imp(:,2), zeros(size(imp(:,1))), '.', ...
          'Color', [0.8 0.25 0.2], 'MarkerSize', 8, 'DisplayName', 'Impacts');
    plot3(ax3, mip(1)/1000, mip(2), 0, 'ks', 'MarkerSize', 10, ...
          'MarkerFaceColor', [0.15 0.5 0.95], 'DisplayName', sprintf('MPI (%.0f, %+.0f)', mip(1), mip(2)));
    plot3(ax3, mip(1)/1000 + r50*cos(ang)/1000, mip(2) + r50*sin(ang), zeros(size(ang)), ...
          'r--', 'LineWidth', 1.8, 'DisplayName', sprintf('CEP_{50} = %.0f m', r50));
    xlabel(ax3, 'Downrange [km]'); ylabel(ax3, 'Crossrange [m]'); zlabel(ax3, 'Altitude [km]');
    title(ax3, '1. 3D trajectory bundle + impacts'); view(ax3, -35, 24);
    legend(ax3, 'Location', 'northeast');
    % --- panel 2: 2D impact footprint ---
    ax2 = subplot(2,2,2); hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');
    if any(isH)
        plot(ax2, imp(isH,1), imp(isH,2), '.', 'Color', [0.1 0.65 0.25], ...
             'MarkerSize', 12, 'DisplayName', sprintf('\leq30 m (%.0f%%)', 100*mean(isH)));
    end
    if any(~isH)
        plot(ax2, imp(~isH,1), imp(~isH,2), 'o', 'Color', [0.85 0.2 0.15], ...
             'MarkerSize', 5, 'LineWidth', 1, 'DisplayName', sprintf('>30 m (%.0f%%)', 100*mean(~isH)));
    end
    plot(ax2, mip(1)+30*cos(ang), mip(2)+30*sin(ang), 'k:', 'LineWidth', 2, 'DisplayName', '30 m spec');
    plot(ax2, mip(1)+r50*cos(ang), mip(2)+r50*sin(ang), 'r--', 'LineWidth', 2.2, ...
         'DisplayName', sprintf('CEP_{50} = %.0f m', r50));
    plot(ax2, mip(1)+r90*cos(ang), mip(2)+r90*sin(ang), 'm-.', 'LineWidth', 1.5, ...
         'DisplayName', sprintf('CEP_{90} = %.0f m', r90));
    plot(ax2, mip(1), mip(2), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', [0.15 0.5 0.95], 'DisplayName', 'MPI');
    axis(ax2, 'equal');
    xlabel(ax2, 'Impact downrange [m]'); ylabel(ax2, 'Impact crossrange [m]');
    title(ax2, '2. Impact footprint');
    legend(ax2, 'Location', 'northeast');
    % --- panel 4: atmosphere profile ---
    axE = subplot(2,2,4); hold(axE,'on'); grid(axE,'on'); box(axE,'on');
    alts = linspace(0,12000,250);
    rr = zeros(size(alts)); aa = zeros(size(alts));
    for i = 1:numel(alts)
        [rr(i), aa(i)] = isa_rho_a(alts(i));
    end
    plot(axE, alts/1000, rr, 'Color', [0 0.45 0.85], 'LineWidth', 2, 'DisplayName', '\rho [kg/m^3]');
    yyaxis(axE, 'right');
    plot(axE, alts/1000, aa, 'Color', [0.9 0.4 0.1], 'LineWidth', 2);
    ylabel(axE, 'a [m/s]');
    yyaxis(axE, 'left');
    xlabel(axE, 'Altitude [km]'); ylabel(axE, '\rho [kg/m^3]');
    title(axE, '3. ISA 1962 atmosphere');
    sgtitle(ttl);
    print(f, fname, '-dpng', '-r150');
    fprintf('Saved %s\n', fname);
catch e
    fprintf('Plot note: %s\n', e.message);
end
end

function [rho, a] = isa_rho_a(z)
if z <= 11000
    T = 288.15 - 0.0065*z;
    pr = 101325*(T/288.15)^(9.80665/(0.0065*287.05287));
else
    T = 288.15 - 0.0065*11000;
    pt = 101325*(T/288.15)^(9.80665/(0.0065*287.05287));
    pr = pt*exp(-9.80665*(z-11000)/(287.05287*T));
end
rho = pr/(287.05287*T);
a = sqrt(1.4*287.05287*T);
end
