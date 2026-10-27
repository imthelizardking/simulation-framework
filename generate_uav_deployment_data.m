function [stowed, deployed] = generate_uav_deployment_data()
%GENERATE_UAV_DEPLOYMENT_DATA Save two synthetic deployable-wing fixtures.
% These arbitrary coefficients exercise software only; they are not a
% validated UAV model and must not be used to predict aircraft performance.
    stowed = makeDataset([0 0.1 0.2], [-10 0 10], false);
    deployed = makeDataset([0 0.05 0.1 0.15 0.2], [-10 -5 0 5 10], true);
    folder = fullfile(fileparts(mfilename('fullpath')), 'data');
    save(fullfile(folder, 'UAV_Test_Stowed.mat'), 'stowed', '-v7');
    save(fullfile(folder, 'UAV_Test_Deployed.mat'), 'deployed', '-v7');
end

function data = makeDataset(mach, alphaDegrees, wingsOpen)
    data.Mach = mach;
    data.Alpha = deg2rad(alphaDegrees);
    data.RefArea = 1.0;   % same arbitrary reference for BOTH configurations
    data.RefLength = 0.5;
    data.Description = 'Synthetic software-test fixture; not validated flight data.';
    data.MomentReference = 'Common fixed origin in both configurations';
    [alpha, m] = ndgrid(data.Alpha, data.Mach);
    if wingsOpen
        data.CD = 0.10 + 0.03 * m + 0.8 * alpha.^2;
        data.CL = 4.0 * alpha;
        data.CM = -0.4 * alpha;
        data.CMQ = -3.0 * ones(size(m));
        data.CLLP = -0.3 * ones(size(m));
        data.CLNR = -0.3 * ones(size(m));
    else
        data.CD = 0.06 + 0.02 * m + 0.2 * alpha.^2;
        data.CL = 1.0 * alpha;
        data.CM = -0.1 * alpha;
        data.CMQ = -1.0 * ones(size(m));
        data.CLLP = -0.1 * ones(size(m));
        data.CLNR = -0.1 * ones(size(m));
    end
    % Body-axis fields used by Aero6DoF's force path. Both configurations
    % use the same convention: Fx = -CA*qS, Fz = -CN*qS at beta=0.
    data.CA = data.CD .* cos(alpha) - data.CL .* sin(alpha);
    data.CN = data.CD .* sin(alpha) + data.CL .* cos(alpha);
end
