classdef DynamicsMPMM < BaseDynamics
    % Blueprint: Modified Point-Mass Model (STANAG 4355-style). Meant to
    % integrate only position/velocity/spin; IDX_EULER is meant to be
    % derived algebraically (yaw of repose) rather than integrated.
    % computeDerivatives is an interface-only stub, not yet implemented.
    methods
        function obj = DynamicsMPMM()
            obj.StateDimension = BaseDynamics.FULL_STATE_DIM;
        end

        function dX = computeDerivatives(~, ~, ~, ~, ~, ~, ~)
            dX = zeros(BaseDynamics.FULL_STATE_DIM, 1);
        end
    end
end
