%% Scenario: 35mm Anti-Drone Burst Engagement (Multiple Rounds, Realistic Timing)
clear; clc; clf;

% 1. Setup Atmosphere
env = Atmosphere();

% 2. Setup Target: same DJI-class drone as runner_35mm_antidrone.m
% forward=1000, down=-120 (120m above launch), right=-200 (200m left);
% vel forward=-12 (closing), down=0 (level), right=-8. Same physical
% scenario as before (forward=1000,crossrange=200,altitude=120), just
% re-authored directly in FDR (see BaseTarget's constructor comment).
target = TargetConstantVelocity([1000; -120; -200], [-12; 0; -8], 'FDR');

% 3. Weapon Platform: 35mm twin AA mount (muzzle velocity dispersion,
%    aiming jitter, traverse limits, cyclic rate all come from here)
platform = WeaponPlatformFactory.create35mmAAMount();

% 4. Fire Mission: burst of rounds with realistic inter-round timing -
%    each round's lead solve uses the drone's actual position at that
%    round's fire time, not a single lead frozen at t=0.
mission = FireMission(platform, @MunitionFactory.create35mmHE, target, env);
mission.Frame = 'FDR';
mission.LauncherPos = [0; 0; 0];
mission.NumRounds   = 10;

fprintf('Firing %d-round burst from %s (cyclic rate %.0f rpm)...\n', ...
    mission.NumRounds, platform.Name, platform.RateOfFire);
tic;
results = mission.run();
fprintf('Mission finished in %.2f seconds.\n', toc);

% 5. Per-Round Report
fprintf('\n%4s %8s %8s %8s %8s %10s %s\n', 'Rnd', 'Fire(s)', 'Elev(mil)', 'Azim(mil)', 'V0', 'Miss(m)', 'Outcome');
hits = 0;
for i = 1:numel(results)
    r = results(i);
    if ~r.IsValid
        fprintf('%4d %8.3f %8s %8s %8.1f %10s %s\n', r.RoundNum, r.FireTime, '-', '-', r.MuzzleVelocity, '-', 'NO SOLUTION');
        continue;
    end
    is_hit = strcmp(r.TerminationReason, 'ImpactDistance');
    if is_hit, hits = hits + 1; end
    fprintf('%4d %8.3f %8.1f %8.1f %8.1f %10.2f %s\n', r.RoundNum, r.FireTime, ...
        r.Elevation, r.Azimuth, ...
        r.MuzzleVelocity, r.MissDistance, r.TerminationReason);
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
    plot(impact_pts(:,1), CoordinateFrame.altitudeColumn(impact_pts(:,2), mission.Frame), ...
        'bx', 'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', 'Impact');
end
if ~isempty(target_pts)
    plot(target_pts(:,1), CoordinateFrame.altitudeColumn(target_pts(:,2), mission.Frame), ...
        'r.', 'MarkerSize', 14, 'DisplayName', 'Target at impact');
end
legend('show', 'Location', 'best');
xlabel('Downrange X (m)'); ylabel('Altitude (m)');
title(sprintf('%s vs %s - %d Rounds', platform.Name, target.Name, mission.NumRounds), 'Interpreter', 'none');
