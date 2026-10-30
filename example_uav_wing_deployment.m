function samples = example_uav_wing_deployment(makePlots)
%EXAMPLE_UAV_WING_DEPLOYMENT Inspect switching/blending using synthetic data.
% Run from the repository root. Pass false for headless coefficient checks.
% This demonstrates coefficient lookup only, not a complete UAV simulation.
    if nargin < 1, makePlots = true; end
    stowed = FactoryData.load('UAV_Test_Stowed');
    deployed = FactoryData.load('UAV_Test_Deployed');
    if isempty(stowed) || isempty(deployed)
        error('UAVExample:MissingData', 'Run generate_uav_deployment_data first.');
    end
    aero = Aero6DoF();
    aero.RefArea = stowed.RefArea;
    aero.RefLength = stowed.RefLength;
    aero.setConfiguration('Stowed', stowed);
    aero.setConfiguration('WingsOpen', deployed);
    aero.ActiveTableKey = 'Stowed';
    aero.DeployedTableKey = 'WingsOpen';
    aero.DeploymentTime = 5;

    time = [0; 4.99; 5; 6; 7; 8];
    mach = 0.1; alpha = deg2rad(5);
    stepCD = zeros(size(time)); stepCL = stepCD;
    blendCD = stepCD; blendCL = stepCD; ratio = stepCD;
    for k = 1:numel(time)
        aero.DeploymentDuration = 0;
        c = aero.evaluateCoefficients(mach, alpha, time(k));
        stepCD(k) = c.CD; stepCL(k) = c.CL;
        aero.DeploymentDuration = 2;
        c = aero.evaluateCoefficients(mach, alpha, time(k));
        blendCD(k) = c.CD; blendCL(k) = c.CL;
        ratio(k) = aero.deploymentRatioAt(time(k));
    end
    samples = table(time, ratio, stepCD, blendCD, stepCL, blendCL);
    disp(samples);
    % Check the switch boundary and the halfway blend, using independently
    % queried endpoints so different dataset grids are respected.
    assert(abs(stepCD(3) - stepCD(end)) < 1e-12);
    assert(abs(blendCD(3) - stepCD(1)) < 1e-12);
    assert(abs(blendCD(4) - (stepCD(1) + stepCD(end))/2) < 1e-12);
    assert(abs(blendCL(4) - (stepCL(1) + stepCL(end))/2) < 1e-12);
    if makePlots
        figure('Name', 'Synthetic UAV wing deployment');
        subplot(2,1,1);
        plot(time, stepCD, '-o', time, blendCD, '-s');
        ylabel('CD'); grid on;
        legend('Instant switch', 'Two-second blend', 'Location', 'best');
        title('Synthetic coefficients at Mach 0.1, alpha = 5 deg');
        subplot(2,1,2);
        plot(time, stepCL, '-o', time, blendCL, '-s');
        xlabel('Time (s)'); ylabel('CL'); grid on;
    end
end
