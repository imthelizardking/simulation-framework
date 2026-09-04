classdef (Abstract) BaseIntegrator < handle
    methods (Abstract)
        X_next = step(obj, deriv_fn, t, X, dt)
        dt_k = computeStepSize(obj, t, X, target_pos, dt_nominal)
    end
end
