%% Scenario: 35mm Anti-Drone Burst Engagement (Multiple Rounds, Realistic Timing)
clear; clc; close all;

% 1. Setup Environment
env = Environment(1.225, 9.81, [0; 0; 0]);

% 2. Setup Target: same DJI-class drone as runner_35mm_antidrone.m
target = ConstantVelocityTarget([1000; 200; 120], [-12; 8; 0]);

% 3. Weapon Platform: 35mm twin AA mount (muzzle velocity dispersion,
%    aiming jitter, traverse limits, cyclic rate all come from here)
platform = WeaponPlatformFactory.create35mmAAMount();

% 4. Fire Mission: burst of rounds with realistic inter-round timing -
%    each round's lead solve uses the drone's actual position at that
%    round's fire time, not a single lead frozen at t=0.
mission = FireMission(platform, @MunitionFactory.create35mmHE, target, env);
mission.LauncherPos = [0; 0; 0];
mission.NumRounds   = 10;

fprintf('Firing %d-round burst from %s (cyclic rate %.0f rpm)...\n', ...
    mission.NumRounds, platform.Name, platform.RateOfFire);
tic;
results = mission.run();
fprintf('Mission finished in %.2f seconds.\n', toc);

% 5. Per-Round Report
fprintf('\n%4s %8s %8s %8s %8s %10s %s\n', 'Rnd', 'Fire(s)', 'Elev', 'Azim', 'V0', 'Miss(m)', 'Outcome');
hits = 0;
for i = 1:numel(results)
    r = results(i);
    if ~r.IsValid
        fprintf('%4d %8.3f %8s %8s %8.1f %10s %s\n', r.RoundNum, r.FireTime, '-', '-', r.MuzzleVelocity, '-', 'NO SOLUTION');
        continue;
    end
    is_hit = strcmp(r.TerminationReason, 'ImpactDistance');
    if is_hit, hits = hits + 1; end
    fprintf('%4d %8.3f %8.2f %8.2f %8.1f %10.2f %s\n', r.RoundNum, r.FireTime, ...
        rad2deg(r.Elevation), rad2deg(r.Azimuth), r.MuzzleVelocity, r.MissDistance, r.TerminationReason);
end
fprintf('\nHits: %d / %d rounds\n', hits, mission.NumRounds);

% 6. Impact Scatter Plot
impact_pts = [];
target_pts = [];
for i = 1:numel(results)
    r = results(i);
    if ~r.IsValid || isempty(r.Telemetry), continue; end
    impact_pts = [impact_pts; r.Telemetry.pos(end, :)]; %#ok<AGROW>
    target_pts = [target_pts; r.Telemetry.target_pos(end, :)]; %#ok<AGROW>
end

figure('Name', 'Burst Impact Dispersion', 'Color', 'w');
hold on; grid on; box on;
if ~isempty(impact_pts)
    plot(impact_pts(:,1), impact_pts(:,3), 'bx', 'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', 'Impact');
end
if ~isempty(target_pts)
    plot(target_pts(:,1), target_pts(:,3), 'r.', 'MarkerSize', 14, 'DisplayName', 'Target at impact');
end
legend('show', 'Location', 'best');
xlabel('Downrange X (m)'); ylabel('Altitude Z (m)');
title(sprintf('%s vs %s - %d Rounds', platform.Name, target.Name, mission.NumRounds), 'Interpreter', 'none');
