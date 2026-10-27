function result = runner_uav_seed_planting(makePlots)
%RUNNER_UAV_SEED_PLANTING Civilian UAV glide, wing deployment, and seed drop.
% Follows the setup -> flight -> analysis structure of the other runners.
% Synthetic aero/seed parameters demonstrate software only. This is an
% unpowered, open-loop glide, not a trimmed aircraft or an autonomous mission.
% Run runner_uav_seed_planting, or result=runner_uav_seed_planting(false).
    if nargin < 1, makePlots = true; end

    % 1. Editable scenario settings (positions below use Z-Up, meters).
    initialPosition = [0; 0; 40];
    initialSpeed = 18;          % m/s, initially level flight along +X
    plantingCenter = [45; 0; 0];
    plantingRadius = 15;        % m, circular planting zone on flat ground
    deploymentTime = 0.5;      % s since start of flight
    deploymentDuration = 0.5;  % s, fully deployed at t=1
    releaseTime = 1.5;         % s, fixed dispenser trigger
    flightDuration = 8;        % s, or until aircraft reaches ground
    dt = 0.02;
    seed.Count = 40;
    seed.Mass = 2e-5;          % kg per representative seed (illustrative)
    seed.Area = pi * 0.0015^2; % m^2, fixed presented cross-section
    seed.CD = 0.8;
    seed.VelocityStd = 0.3;    % m/s per-axis dispenser spread
    seed.MaxFallTime = 30;     % s after release
    seed.RandomSeed = 7;

    % 2. Environment and target: a stationary planting location, Z-Up.
    env = Atmosphere();
    plantingTarget = TargetStatic(plantingCenter);
    plantingTarget.Name = 'Seed planting zone';

    % 3. Assemble the UAV with the shared 6-DoF model and test aero data.
    stowed = FactoryData.load('UAV_Test_Stowed');
    deployed = FactoryData.load('UAV_Test_Deployed');
    if isempty(stowed) || isempty(deployed)
        error('SeedRunner:MissingData', 'Run generate_uav_deployment_data first.');
    end
    uav = Munition('Civilian_UAV_Test'); % generic existing vehicle container
    uav.Type = 'Civilian UAV';
    uav.Dynamics = Dynamics6DoF();
    uav.Aero = Aero6DoF();
    uav.Aero.Frame = 'NED';
    uav.Aero.RefArea = stowed.RefArea;
    uav.Aero.RefLength = stowed.RefLength;
    uav.Aero.setConfiguration('Stowed', stowed);
    uav.Aero.setConfiguration('WingsOpen', deployed);
    uav.Aero.ActiveTableKey = 'Stowed';
    uav.Aero.DeployedTableKey = 'WingsOpen';
    uav.Aero.DeploymentTime = deploymentTime;
    uav.Aero.DeploymentDuration = deploymentDuration;
    % Constant aircraft mass/inertia, common CG and moment reference.
    inertia = diag([1 2 3]);
    uav.MassProperties = struct('Mass', 12, 'MassEmpty', 12, ...
        'Inertia', inertia, 'InertiaFull', inertia, 'InertiaEmpty', inertia, ...
        'CGFull', zeros(3,1), 'CGEmpty', zeros(3,1), 'CGRef', zeros(3,1));
    uav.Dynamics.InitialState = uav.buildLaunchState(initialPosition, initialSpeed, 0, 0);

    % 4. Flight uses the normal Simulation loop, explicitly ground=0.
    sim = Simulation(uav, plantingTarget, env, [0 flightDuration], dt);
    sim.Frame = 'NED';
    sim.Termination.MinAltitude = 0;
    fprintf('Civilian UAV: wings deploy at %.2f s; seeds release at %.2f s.\n', ...
        deploymentTime, releaseTime);
    % Suppress the existing loop's per-step debug prints in this example.
    evalc('telemetry = sim.run();');
    if any(~isfinite(telemetry.pos(:))) || any(~isfinite(telemetry.vel(:)))
        error('SeedRunner:NonfiniteFlight', 'Synthetic flight diverged.');
    end
    if releaseTime < telemetry.time(1) || releaseTime > telemetry.time(end)
        error('SeedRunner:ReleaseNotReached', 'Flight ended before the configured seed release.');
    end

    % 5. Interpolate flight telemetry at the exact dispenser time.
    % Convert once from NED to Z-Up; seed dynamics and plots use Z-Up.
    releasePosNED = interp1(telemetry.time, telemetry.pos, releaseTime, 'linear')';
    releaseVelNED = interp1(telemetry.time, telemetry.vel, releaseTime, 'linear')';
    releasePosition = CoordinateFrame.rotateVector(releasePosNED, 'NED');
    releaseVelocity = CoordinateFrame.rotateVector(releaseVelNED, 'NED');
    if releasePosition(3) <= 0
        error('SeedRunner:ReleaseBelowGround', 'Seed release must occur above ground.');
    end
    seeds = flySeeds(releasePosition, releaseVelocity, releaseTime, env, seed, dt);

    % 6. Evaluate planting coverage using horizontal ground coordinates.
    distance = hypot(seeds.landingPositions(:,1) - plantingCenter(1), ...
                     seeds.landingPositions(:,2) - plantingCenter(2));
    inZone = seeds.landed & distance <= plantingRadius;
    result = struct('telemetry', telemetry, 'target', plantingTarget, ...
        'zoneRadius', plantingRadius, 'releaseTime', releaseTime, ...
        'releasePosition', releasePosition, 'releaseVelocity', releaseVelocity, ...
        'seeds', seeds, 'inZone', inZone, 'landingDistance', distance, ...
        'deploymentTime', deploymentTime, 'deploymentDuration', deploymentDuration);
    fprintf('Release position (Z-Up): [%.1f, %.1f, %.1f] m\n', releasePosition);
    fprintf('Seeds landed: %d/%d; within planting zone: %d/%d (%.1f%%).\n', ...
        nnz(seeds.landed), seed.Count, nnz(inZone), seed.Count, 100*nnz(inZone)/seed.Count);
    if any(~seeds.landed)
        fprintf('%d seeds still airborne at the seed time limit.\n', nnz(~seeds.landed));
    end
    if makePlots, plotPlanting(result); end
end

function seeds = flySeeds(position, velocity, releaseTime, env, cfg, dt)
    % Independent seed point masses, quadratic drag and gravity in Z-Up.
    % Ground-level density/wind are held constant over the short drop.
    % Seed mass loss is negligible here and is not fed back into the UAV.
    stream = RandStream('mt19937ar', 'Seed', cfg.RandomSeed);
    X = [repmat(position, 1, cfg.Count); ...
         repmat(velocity, 1, cfg.Count) + cfg.VelocityStd * randn(stream, 3, cfg.Count)];
    rho = env.getDensity(0);
    wind = CoordinateFrame.rotateVector(env.getWind(0), 'NED');
    dragFactor = 0.5 * rho * cfg.CD * cfg.Area / cfg.Mass;
    stepper = RK4FixedStep();
    time = (0:dt:cfg.MaxFallTime)';
    trace = NaN(numel(time), 3, cfg.Count);
    trace(1,:,:) = reshape(X(1:3,:), 1, 3, cfg.Count);
    alive = true(1, cfg.Count);
    landingPositions = NaN(cfg.Count, 3);
    landingTimes = NaN(cfg.Count, 1);
    last = 1;
    for k = 2:numel(time)
        next = stepper.step(@(~, state) seedDerivatives(state, wind, dragFactor, env.Gravity), ...
            time(k-1), X, time(k)-time(k-1));
        next(:, ~alive) = X(:, ~alive);
        if any(~isfinite(next(:)))
            error('SeedRunner:NonfiniteSeeds', 'Seed states diverged.');
        end
        touchingGround = alive & next(3,:) <= 0;
        for j = find(touchingGround)
            % Resolve the ground crossing between integration samples.
            fraction = X(3,j) / (X(3,j) - next(3,j));
            landing = X(1:3,j) + fraction * (next(1:3,j) - X(1:3,j));
            landing(3) = 0;
            landingPositions(j,:) = landing';
            landingTimes(j) = releaseTime + time(k-1) + fraction*(time(k)-time(k-1));
            next(:,j) = [landing; zeros(3,1)];
        end
        alive(touchingGround) = false;
        X = next;
        trace(k,:,:) = reshape(X(1:3,:), 1, 3, cfg.Count);
        last = k;
        if ~any(alive), break; end
    end
    seeds = struct('time', releaseTime + time(1:last), ...
        'positions', trace(1:last,:,:), 'landingPositions', landingPositions, ...
        'landingTimes', landingTimes, 'landed', ~alive(:));
end

function dX = seedDerivatives(X, wind, dragFactor, gravity)
    relative = X(4:6,:) - wind;
    speed = sqrt(sum(relative.^2, 1));
    acceleration = -dragFactor * relative .* speed + [0; 0; -gravity];
    dX = [X(4:6,:); acceleration];
end

function plotPlanting(r)
    flight = r.telemetry.pos .* [1 -1 -1]; % NED -> Z-Up rows
    center = r.target.Position;
    angle = linspace(0, 2*pi, 100);
    circleX = center(1) + r.zoneRadius*cos(angle);
    circleY = center(2) + r.zoneRadius*sin(angle);
    figure('Name', 'Civilian UAV seed planting', 'Color', 'w');
    subplot(1,2,1);
    plot3(flight(:,1), flight(:,2), flight(:,3), 'b-', 'DisplayName', 'UAV');
    hold on; grid on;
    plot3(circleX, circleY, zeros(size(angle)), 'g-', 'DisplayName', 'Planting zone');
    plot3(center(1), center(2), center(3), 'gp', 'MarkerFaceColor', 'g', ...
        'MarkerSize', 12, 'DisplayName', 'Planting center');
    plot3(r.releasePosition(1), r.releasePosition(2), r.releasePosition(3), ...
        'ko', 'MarkerFaceColor', 'k', 'DisplayName', 'Seed release');
    for j = 1:size(r.seeds.positions,3)
        p = r.seeds.positions(:,:,j);
        plot3(p(:,1), p(:,2), p(:,3), '-', 'Color', [0.7 0.5 0.2], 'HandleVisibility', 'off');
    end
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude (m)');
    title('Open-loop glide and seed trajectories'); view(3); legend('Location', 'best');

    subplot(1,2,2);
    plot(circleX, circleY, 'g-', 'DisplayName', 'Planting zone'); hold on; grid on;
    plot(center(1), center(2), 'gp', 'MarkerFaceColor', 'g', 'DisplayName', 'Planting center');
    landed = r.seeds.landingPositions;
    plot(landed(r.inZone,1), landed(r.inZone,2), 'go', 'DisplayName', 'Seeds inside');
    outside = r.seeds.landed & ~r.inZone;
    plot(landed(outside,1), landed(outside,2), 'rx', 'DisplayName', 'Seeds outside');
    plot(flight(:,1), flight(:,2), 'b:', 'DisplayName', 'UAV ground track');
    axis equal; xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('Planting coverage: %d/%d seeds', nnz(r.inZone), numel(r.inZone)));
    legend('Location', 'best');
end
