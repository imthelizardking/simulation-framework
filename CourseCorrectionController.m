classdef CourseCorrectionController < BaseController
    % Course-correction fuze controller: waits for a valid muzzle-velocity
    % reading (guid_bus.MVR - see MuzzleVelocityRadar/PassthroughGuidance),
    % then runs FireControlComputer.solveDeployTimeForRange exactly once,
    % using the round's OWN real current state (nav_state/t) as the recon
    % starting point, and sets veh_cfg.Aero.airbrake_ADT to the result.
    % Never re-solves after that (Solved latches, same one-shot spirit as
    % AeroMPMM's own Phase2Active) - a no-op every step thereafter.
    %
    % guid_bus.MVR.MuzzleVelocity is used only for the printed report
    % below, not as an input to the solve itself: nav_state IS ground
    % truth throughout this framework (see LongitudinalFinController,
    % which reads nav_state the same way) - there's no separate state-
    % estimation layer to model here, so the solve already has the real
    % current state directly and doesn't need to be driven off a noisy
    % derived value.
    properties
        ReconBuilder             = []    % function handle: () -> fresh Munition replicating the real round's instance settings exactly (see FireControlComputer.solveDeployTimeForRange)
        DesiredRange (1,1) double = 0.0  % m - the range the airbrake deploy should bring the round down to
        SolveInfo    struct       = struct()  % populated once solved - see FireControlComputer.solveDeployTimeForRange's info output
    end

    properties (Access = private)
        Solved (1,1) logical = false
    end

    methods
        function ctrl_bus = step(obj, t, guid_bus, nav_state, veh_cfg, env)
            if ~obj.Solved && isfield(guid_bus, 'MVR') && isfield(guid_bus.MVR, 'Valid') && guid_bus.MVR.Valid
                [adt, info] = FireControlComputer.solveDeployTimeForRange( ...
                    obj.ReconBuilder, nav_state, t, obj.DesiredRange, env);
                veh_cfg.Aero.airbrake_ADT = adt;
                obj.SolveInfo = info;
                obj.Solved = true;
                fprintf(['[CourseCorrectionController] t=%.3f s: measured muzzle velocity %.2f m/s ' ...
                    '(noisy reading) -> solved airbrake_ADT = %.4f s (edge case: %s), ' ...
                    'recon-predicted range = %.1f m (desired %.1f m).\n'], ...
                    t, guid_bus.MVR.MuzzleVelocity, adt, info.EdgeCase, info.AchievedRange, obj.DesiredRange);
            end
            ctrl_bus = struct('FinDeflection', 0.0);
        end
    end
end
