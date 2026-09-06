%% Test: FactoryData table-parsing override for MunitionFactory.createMOD274
% Exercises data/createMOD274Artillery.json, which overrides CD0/CDA2 with
% a "Mach a0 a1" per-breakpoint polynomial table (BaseAero.evalCoeff's
% PiecewisePolynomial case) instead of the hardcoded scalar defaults in
% MunitionFactory.createMOD274. The table's numbers are illustrative test
% values, not real aero data - CD0 comes out much higher than a real
% shell's, so the resulting trajectory is short and steep on purpose; this
% runner is about confirming the data-file override wires through
% correctly end-to-end, not about a realistic MOD274 flight.
clear; clc; close all;

%% 1. Confirm the data file parses as expected
data = FactoryData.load('createMOD274Artillery');
fprintf('=== Parsed data/createMOD274Artillery.json ===\n');
disp(data);
fprintf('CDA  (-> CD0)  : Type=%s, Mach breakpoints=[%s]\n', data.CDA.Type,  num2str(data.CDA.Mach));
fprintf('CDA2 (-> CDA2) : Type=%s, Mach breakpoints=[%s]\n', data.CDA2.Type, num2str(data.CDA2.Mach));

fprintf('\n=== Mach sweep through the piecewise-polynomial tables ===\n');
fprintf('%8s %10s %10s\n', 'Mach', 'CD0', 'CDA2');
for mach = [0.0 0.4 0.45 0.5 0.55 0.6 0.65 0.7 0.9]
    fprintf('%8.2f %10.4f %10.4f\n', mach, ...
        BaseAero.evalCoeff(data.CDA, mach), BaseAero.evalCoeff(data.CDA2, mach));
end

%% 2. Confirm MunitionFactory.createMOD274 actually picks the override up
shell = MunitionFactory.createMOD274('6M');
c = shell.Aero.Tables.Clean;
assert(isequal(c.CD0, data.CDA),   'CD0 was not overridden from data.CDA');
assert(isequal(c.CDA2, data.CDA2), 'CDA2 was not overridden from data.CDA2');
fprintf('\nMunitionFactory.createMOD274 coefficients confirmed overridden by FactoryData.\n');

%% 3. Fire it (fixed QE - the test drag values are too high for the
%    FireControlComputer's normal 25 km shooting solution to converge)
env = Environment(1.225, 9.81, [0; 0; 0]);
target = TargetStatic([3000; 0; 0]); % nominal only - not solved for
launcher_pos = [0; 0; 0];
v0 = shell.Charge.MuzzleVelocity;
elevation = deg2rad(45);
azimuth = 0;

shell.Dynamics.InitialState = shell.buildLaunchState(launcher_pos, v0, elevation, azimuth);

sim = Simulation(shell, target, env, [0 60], 0.005);
fprintf('\nFiring MOD274 with the data-file drag override at QE=45 deg, v0=%.0f m/s...\n', v0);
telemetry = sim.run();

[apogee, apogee_idx] = max(telemetry.pos(:, 3));
final_pos = telemetry.pos(end, :)';
fprintf('Termination Reason : %s\n', sim.TerminationReason);
fprintf('Flight Time        : %.2f s\n', telemetry.time(end));
fprintf('Apogee             : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Ground Range        : %.1f m\n', final_pos(1));

plotTrajectoryProfile(telemetry, target.Position, 'MOD274 - FactoryData Table Override Test');
