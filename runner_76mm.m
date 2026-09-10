%% Scenario: 76mm HE Land-to-Air Engagement (Detailed Mach-Dependent Aero Table)
clear; clc; close all;

% 1. Setup Environment (standard atmosphere, no wind)
env = Environment(1.225, 9.81, [0; 0; 0]);

% 2. Setup Target: same inbound target as runner_sam_engagement.m (the
%    "drone" used there), so the 76mm gun and the guided interceptor can
%    be compared against an identical threat.
target = TargetConstantVelocity([8000; 0; 3000], [-250; 0; 0]);

% 3. Instantiate Vehicle via MunitionFactory (uses AeroTabulated - see
%    MunitionFactory.create76mmHE for the full CD0/CDA2/CDA4/CLA/CLA3
%    coefficient set)
shell = MunitionFactory.create76mmHE();

% 4. Fire Control: analytical direct-lead solve. solveNumericalShooting
%    has no notion of target velocity (it aims at wherever the target is
%    at t=0), so it's the wrong tool now that the target is moving -
%    solveDirectLead accounts for target velocity plus ballistic drop.
fcc = FireControlComputer(env.Gravity, 'Z-Up');
v0 = shell.Charge.MuzzleVelocity; % see MunitionFactory.create76mmHE
launcher_pos = [0; 0; 0];

sol = fcc.solveFiringAngles(launcher_pos, v0, target.getTargetBus(), 'DirectLead');

if ~sol.IsValid
    error('No valid direct-lead solution (target out of envelope or geometry infeasible).');
end

fprintf('=== Direct-Lead Solution ===\n');
fprintf('Elevation                : %.1f mils\n', FireControlComputer.rad2mil(sol.Elevation));
fprintf('Azimuth                  : %.1f mils\n', FireControlComputer.rad2mil(sol.Azimuth));
fprintf('Predicted Intercept Time : %.3f s\n', sol.TimeOfFlight);

% 5. Apply Solved Initial State
shell.Dynamics.InitialState = shell.buildLaunchState(launcher_pos, v0, sol.Elevation, sol.Azimuth);

% 6. Assemble Simulation & Execute. Unguided direct-fire round with no
%    terminal correction - ImpactDistance is a large-HE proximity-effect
%    radius rather than a tight direct-hit tolerance, since
%    solveDirectLead ignores drag entirely: over this longer, higher-drag
%    trajectory than the 35mm case, the shell decelerates substantially
%    and arrives later than predicted, so expect a more noticeable lead
%    error here. TimeSpan is padded well past the predicted intercept
%    time to give the (slower-than-predicted) real shell room to arrive.
sim = Simulation(shell, target, env, [0 sol.TimeOfFlight * 2], 0.005);
sim.Termination.ImpactDistance = 5.0;
sim.Termination.MaxDownrange   = 200.0;

fprintf('Firing 76mm round...\n');
tic;
telemetry = sim.run();
fprintf('Sim finished in %.2f seconds (%.2f s simulated).\n', toc, telemetry.time(end));

% 7. Post-Engagement Analysis
final_pos = telemetry.pos(end, :)';
final_tgt = telemetry.target_pos(end, :)';
miss_dist = norm(final_pos - final_tgt);
[apogee, apogee_idx] = max(telemetry.pos(:,3));
speed_of_sound_series = arrayfun(@(h) env.getSpeedOfSound(h), telemetry.pos(:,3));
max_mach = max(vecnorm(telemetry.vel, 2, 2) ./ speed_of_sound_series);

fprintf('Termination Reason  : %s\n', sim.TerminationReason);
fprintf('Apogee              : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Peak Mach Number    : %.2f\n', max_mach);
fprintf('Final Miss Distance : %.2f m\n', miss_dist);

plotTrajectoryProfile(telemetry, final_tgt, '76mm Land-to-Air Engagement');
