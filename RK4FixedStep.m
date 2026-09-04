classdef RK4FixedStep < BaseIntegrator
    methods
        function X_next = step(~, deriv_fn, t, X, dt)
            k1 = deriv_fn(t, X);
            k2 = deriv_fn(t + 0.5*dt, X + 0.5*dt*k1);
            k3 = deriv_fn(t + 0.5*dt, X + 0.5*dt*k2);
            k4 = deriv_fn(t + dt, X + dt*k3);
            X_next = X + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
        end

        function dt_k = computeStepSize(~, ~, ~, ~, dt_nominal)
            dt_k = dt_nominal;
        end
    end
end
