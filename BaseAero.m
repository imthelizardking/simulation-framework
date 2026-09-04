classdef (Abstract) BaseAero < handle
    properties
        RefArea         (1,1) double = 0.01966 % S_ref (m^2)
        CD0             (1,1) double = 0.35    % Zero-lift drag coefficient
        RefLength       (1,1) double = 0.155   % d_ref (m)
        Tables          struct = struct()
        ActiveTableKey  (1,:) char = 'Clean'
        DeploymentRatio (1,1) double = 0.0
    end
    methods (Abstract)
        [F_aero, M_aero] = getForcesAndMoments(obj, X, env, fin_deflections)
    end
end