classdef RK4AdaptiveNearEncounter < RK4FixedStep
    % Blueprint: reuses RK4FixedStep's stepper but shrinks dt as range-to-
    % target closes inside EngageRange, down to a floor of MinDt. This is a
    % placeholder linear policy to establish the interface — not yet wired
    % into Simulation.run(), whose main loop still walks a precomputed
    % fixed-dt time array. Swapping in real per-step-size integration needs
    % that loop restructured (growable telemetry, dynamic time stepping),
    % which is follow-up work once a real step-size policy is chosen.
    properties
        EngageRange (1,1) double = 500.0  % range (m) where shrinking begins
        MinDt       (1,1) double = 1e-4   % step-size floor near encounter
    end

    methods
        function dt_k = computeStepSize(obj, ~, X, target_pos, dt_nominal)
            range = norm(target_pos - X(BaseDynamics.IDX_POS));
            if range >= obj.EngageRange
                dt_k = dt_nominal;
            else
                ratio = max(range, 0) / obj.EngageRange;
                dt_k = obj.MinDt + (dt_nominal - obj.MinDt) * ratio;
            end
        end
    end
end
