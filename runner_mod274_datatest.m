%% Test: MunitionData .mat override for MunitionFactory.createMOD274
% Exercises the new standardized per-munition data file (see
% MunitionData): data/<Munition.Name>.mat overrides CD0/CDA2 (and Mass/
% Inertia) instead of MunitionFactory.createMOD274's hardcoded scalar
% defaults. Since these files are produced independently (not by this
% framework), this script builds and saves a small illustrative one
% itself first, then confirms the factory preset actually picks it up -
% same "Mach a0 a1" per-breakpoint polynomial shape the old (now removed)
% data/createMOD274.json test used, just as a plain numeric matrix
% instead of parsed table text (see MunitionData's class comment). The
% numbers are illustrative test values, not real aero data - CD0 comes
% out much higher than a real shell's, so the resulting trajectory is
% short and steep on purpose; this runner is about confirming the
% override wires through correctly end-to-end, not a realistic MOD274
% flight.
clear; clc; clf;

%% 1. Build and save a small illustrative data/MOD274_155mm_BB.mat
% Columns: [Mach, a0, a1] - row R's local polynomial coeff = a0 + a1*mach,
% valid from that row's Mach breakpoint onward (see MunitionData).
S.aero.phase.CD0  = [0.4 1.7 0.1; 0.5 1.8 0.2; 0.6 1.9 0.115; 0.7 2.2 0.117];
S.aero.phase.CDA2 = [0.4 3.7 1.0; 0.5 3.8 1.1; 0.6 3.9 1.15;  0.7 4.2 1.17];
S.inertial.mass = 45.0;
% inertia_matrix demonstrated here (full 3x3), vs. the axial/transverse
% scalar pair used in the other example generators - both are valid,
% see MunitionData.inertia.
S.inertial.inertia_matrix = diag([0.15, 1.7, 1.7]);

here = fileparts(mfilename('fullpath'));
matPath = fullfile(here, 'data', 'MOD274_155mm_BB.mat');
save(matPath, 'S');
fprintf('Wrote test fixture: %s\n', matPath);

%% 2. Confirm the file round-trips through MunitionData as expected
mdata = MunitionData.load('MOD274_155mm_BB');
aero_over = MunitionData.aeroCoeffs(mdata);
fprintf('\n=== Mach sweep through the piecewise-polynomial tables ===\n');
fprintf('%8s %10s %10s\n', 'Mach', 'CD0', 'CDA2');
for mach = [0.0 0.4 0.45 0.5 0.55 0.6 0.65 0.7 0.9]
    fprintf('%8.2f %10.4f %10.4f\n', mach, ...
        BaseAero.evalCoeff(aero_over.CD0, mach), BaseAero.evalCoeff(aero_over.CDA2, mach));
end

%% 3. Confirm MunitionFactory.createMOD274 actually picks the override up
shell = MunitionFactory.createMOD274('6M');
c = shell.Aero.Tables.Clean;
assert(isequal(c.CD0, aero_over.CD0),   'CD0 was not overridden from data/MOD274_155mm_BB.mat');
assert(isequal(c.CDA2, aero_over.CDA2), 'CDA2 was not overridden from data/MOD274_155mm_BB.mat');
assert(isequal(shell.MassProperties.Mass, S.inertial.mass), 'Mass was not overridden from data/MOD274_155mm_BB.mat');
assert(isequal(shell.MassProperties.Inertia, S.inertial.inertia_matrix), 'Inertia was not overridden from data/MOD274_155mm_BB.mat');
fprintf('\nMunitionFactory.createMOD274 coefficients confirmed overridden by MunitionData.\n');

%% 4. Fire it (fixed QE - the test drag values are too high for the
%    FireControlComputer's normal 25 km shooting solution to converge)
env = Environment();
target = TargetStatic([3000; 0; 0], [], 'FDR'); % forward=3km, down=0, right=0 - nominal only, not solved for
launcher_pos = [0; 0; 0];
v0 = shell.Charge.MuzzleVelocity;
elevation = FireControlComputer.mil2rad(800); % 45 deg, expressed in NATO mils
azimuth = 0;

shell.Dynamics.InitialState = shell.buildLaunchState(launcher_pos, v0, elevation, azimuth);

sim = Simulation(shell, target, env, [0 60], 0.005);
sim.Frame = 'FDR';
fprintf('\nFiring MOD274 with the data-file drag override at QE=800 mils, v0=%.0f m/s...\n', v0);
telemetry = sim.run();

[apogee, apogee_idx] = max(CoordinateFrame.altitudeColumn(telemetry.pos(:,2), telemetry.Frame));
final_pos = telemetry.pos(end, :)';
fprintf('Termination Reason : %s\n', sim.TerminationReason);
fprintf('Flight Time        : %.2f s\n', telemetry.time(end));
fprintf('Apogee             : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Forward Range      : %.0f m\n', final_pos(1));
fprintf('Deflection         : %.0f m\n', final_pos(3));
fprintf('Total Range        : %.0f m\n', hypot(final_pos(1), final_pos(3)));

% telemetry.target_pos, not target.Position - the latter is the raw
% Z-Up-sense value TargetStatic was constructed with, never rotated into
% the sim's own frame; telemetry.target_pos is Simulation's own per-step
% resolveTargetBus() output, already correctly rotated to match final_pos
% (see Simulation.m's class comment on Frame).
plotTrajectoryProfile(telemetry, telemetry.target_pos(end, :)', 'MOD274 - MunitionData .mat Override Test');
