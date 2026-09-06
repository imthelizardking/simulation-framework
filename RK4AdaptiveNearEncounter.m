classdef RK4AdaptiveNearEncounter < RK4FixedStep
    % Reuses RK4FixedStep's stepper but shrinks dt as range-to-target
    % closes inside EngageRange, down to a floor of MinDt - a placeholder
    % linear policy, not an error-controlled one. Wired into
    % ParticleSwarmSimulation.run() (set via Simulation.ParticleIntegrator),
    % whose loop walks a dynamic while-t<t_end structure with growable
    % telemetry; Simulation.run()'s own single-body loop still walks a
    % precomputed fixed-dt time array and would need the same restructuring
    % to use this for the parent munition's own flight.
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
