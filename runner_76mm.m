%% Scenario: 76mm HE Land-to-Air Engagement (Detailed Mach-Dependent Aero Table)
clear; clc; clf;

% 1. Setup Atmosphere (standard atmosphere, no wind)
env = Atmosphere();

% 2. Setup Target: same inbound target as runner_sam_engagement.m (the
%    "drone" used there), so the 76mm gun and the guided interceptor can
%    be compared against an identical threat.
% forward=8000, down=-3000 (3000m above launch), right=0; vel forward=-250
% (closing), down=0, right=0. Same physical scenario as before
% (forward=8000,crossrange=0,altitude=3000), just re-authored directly
% in FDR (see BaseTarget's constructor comment).
target = TargetConstantVelocity([8000; -3000; 0], [-250; 0; 0], 'FDR');

% 3. Instantiate Vehicle via MunitionFactory (uses AeroMPMM - see
%    MunitionFactory.create76mmHE for the full CD0/CDA2/CDA4/CLA/CLA3
%    coefficient set)
shell = MunitionFactory.create76mmHE();

% 4. Fire Control: trajectory shooting. solveNumericalShooting has no
%    notion of target velocity (it aims at wherever the target is at
%    t=0), and the analytic direct-lead solve ignores drag - over this
%    long, high-drag trajectory the shell decelerates substantially and
%    would arrive well behind its lead point. solveTrajectoryShooting
%    iterates real Simulation flights (same dt/Frame as below) against
%    the moving target instead.
fcc = FireControlComputer(env.Gravity, 'Z-Up');
v0 = shell.Charge.MuzzleVelocity; % see MunitionFactory.create76mmHE
launcher_pos = [0; 0; 0];

sol = fcc.solveTrajectoryShooting(@MunitionFactory.create76mmHE, launcher_pos, v0, ...
    target, env, [0 30], 0.005, 'FDR');

if ~sol.IsValid
    error('No valid trajectory solution (target out of envelope or geometry infeasible).');
end

fprintf('=== Trajectory Shooting Solution ===\n');
fprintf('Elevation                : %.1f mils\n', sol.Elevation);
fprintf('Azimuth                  : %.1f mils\n', sol.Azimuth);
fprintf('Predicted Intercept Time : %.3f s\n', sol.TimeOfFlight);
fprintf('Predicted Miss Distance  : %.3f m (%d iterations)\n', sol.MissDistance, sol.Iterations);

% 5. Apply Solved Initial State
shell.Dynamics.InitialState = shell.buildLaunchState(launcher_pos, v0, ...
    FireControlComputer.mil2rad(sol.Elevation), FireControlComputer.mil2rad(sol.Azimuth));

% 6. Assemble Simulation & Execute. Unguided direct-fire round with no
%    terminal correction - ImpactDistance is a large-HE proximity-effect
%    radius.
sim = Simulation(shell, target, env, [0 sol.TimeOfFlight * 1.2], 0.005);
sim.Frame = 'FDR';
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
altitude_series = CoordinateFrame.altitudeColumn(telemetry.pos(:,2), telemetry.Frame);
[apogee, apogee_idx] = max(altitude_series);
speed_of_sound_series = arrayfun(@(h) env.getSpeedOfSound(h), altitude_series);
max_mach = max(vecnorm(telemetry.vel, 2, 2) ./ speed_of_sound_series);

fprintf('Termination Reason  : %s\n', sim.TerminationReason);
fprintf('Apogee              : %.1f m at t=%.2f s\n', apogee, telemetry.time(apogee_idx));
fprintf('Peak Mach Number    : %.2f\n', max_mach);
fprintf('Final Miss Distance : %.2f m\n', miss_dist);

plotTrajectoryProfile(telemetry, final_tgt, '76mm Land-to-Air Engagement');
