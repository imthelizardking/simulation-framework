classdef UAVMassModel
    % One fixed airframe and one uniformly depleted, fixed-shape container.
    % SI units. CGs are positions in a common, right-handed body datum.
    % Tensors are about each component's OWN CG, expressed in body axes.
    % Pure time lookup permits out-of-order RK4 evaluations and exact
    % scheduled releases without integrating below the empty mass.
    properties
        DryMass = 12
        DryCG = [0; 0; 0]
        DryInertia = diag([1 2 3])
        InitialPayloadMass = 0
        PayloadCG = [0; 0; 0]
        PayloadInertiaPerKg = zeros(3)
        ReleaseTime = Inf
        ReleaseDuration = 0
    end

    methods
        function s = evaluate(obj, t)
            validateattributes(t, {'double'}, {'scalar', 'real', 'finite'});
            obj.validate();
            fraction = 1;
            rate = 0;
            if isfinite(obj.ReleaseTime) && t >= obj.ReleaseTime
                if obj.ReleaseDuration == 0
                    fraction = 0; % discrete release; state projection handles the jump
                else
                    fraction = max(0, 1 - (t - obj.ReleaseTime) / obj.ReleaseDuration);
                    if t < obj.ReleaseTime + obj.ReleaseDuration
                        rate = -obj.InitialPayloadMass / obj.ReleaseDuration;
                    end
                end
            end
            payload = obj.InitialPayloadMass * fraction;
            mass = obj.DryMass + payload;
            cg = (obj.DryMass * obj.DryCG + payload * obj.PayloadCG) / mass;
            inertia = obj.DryInertia + obj.shift(obj.DryMass, obj.DryCG - cg) ...
                + payload * obj.PayloadInertiaPerKg + obj.shift(payload, obj.PayloadCG - cg);
            s = struct('Mass', mass, 'CG', cg, 'Inertia', inertia, ...
                'PayloadMass', payload, 'MassRate', rate);
        end

        function validate(obj)
            validateattributes(obj.DryMass, {'double'}, {'scalar', 'real', 'finite', 'positive'});
            validateattributes(obj.InitialPayloadMass, {'double'}, {'scalar', 'real', 'finite', 'nonnegative'});
            validateattributes(obj.DryCG, {'double'}, {'size', [3 1], 'real', 'finite'});
            validateattributes(obj.PayloadCG, {'double'}, {'size', [3 1], 'real', 'finite'});
            validateattributes(obj.ReleaseTime, {'double'}, {'scalar', 'real', 'nonnan', 'nonnegative'});
            validateattributes(obj.ReleaseDuration, {'double'}, {'scalar', 'real', 'finite', 'nonnegative'});
            obj.checkTensor(obj.DryInertia, false);
            obj.checkTensor(obj.PayloadInertiaPerKg, true);
        end
    end

    methods (Static, Access = private)
        function I = shift(mass, r)
            I = mass * (dot(r, r) * eye(3) - r * r');
        end

    end

    methods (Static)
        function checkTensor(I, allowZero)
            validateattributes(I, {'double'}, {'size', [3 3], 'real', 'finite'});
            tolerance = 1e-12 * max(1, norm(I, 'fro'));
            if norm(I - I', 'fro') > tolerance
                error('UAVMassModel:Inertia', 'Inertia tensor must be symmetric.');
            end
            principal = eig((I + I')/2);
            if (allowZero && min(principal) < -tolerance) || ...
                    (~allowZero && min(principal) <= 0) || ...
                    2 * max(principal) > sum(principal) + tolerance
                error('UAVMassModel:Inertia', 'Inertia tensor must have physically valid principal moments.');
            end
        end
    end
end
