classdef MassPropertiesEvaluator
    % Shared adapter for existing MassProperties structs. All tensors refer
    % to the component's own CG in common body axes; positions are meters.
    methods (Static)
        function s = evaluate(config, mass)
            validateattributes(config.Mass, {'double'}, {'scalar','real','finite','positive'});
            validateattributes(mass, {'double'}, {'scalar','real','finite','positive'});
            full = config.Mass;
            empty = MassPropertiesEvaluator.value(config, 'MassEmpty', full);
            validateattributes(empty, {'double'}, {'scalar','real','finite','nonnegative'});
            if empty > full
                error('MassProperties:MassBounds', 'Empty mass cannot exceed full mass.');
            end
            % Without an explicit empty endpoint, retain the configured
            % tensor/CG while allowing a propulsion model to reduce mass.
            lower = 0;
            if isfield(config, 'MassEmpty'), lower = empty; end
            if mass < lower - 1e-9*max(1,full) || mass > full + 1e-9*max(1,full)
                error('MassProperties:MassBounds', 'Current mass is outside configured empty/full bounds.');
            end
            mass = min(full, max(lower, mass));
            cgFull = MassPropertiesEvaluator.value(config, 'CGFull', ...
                MassPropertiesEvaluator.value(config, 'CG', zeros(3,1)));
            cgEmpty = MassPropertiesEvaluator.value(config, 'CGEmpty', cgFull);
            validateattributes(cgFull, {'double'}, {'size',[3 1],'real','finite'});
            validateattributes(cgEmpty, {'double'}, {'size',[3 1],'real','finite'});
            IFull = MassPropertiesEvaluator.value(config, 'InertiaFull', config.Inertia);
            IEmpty = MassPropertiesEvaluator.value(config, 'InertiaEmpty', config.Inertia);
            IFull = MassPropertiesEvaluator.tensor(IFull);
            IEmpty = MassPropertiesEvaluator.tensor(IEmpty);
            if isfield(config,'MassEmpty') && full == empty && ...
                    (norm(cgFull-cgEmpty) > 1e-12 || norm(IFull-IEmpty,'fro') > 1e-12)
                error('MassProperties:Endpoints', 'Equal full/empty masses require equal CG/inertia endpoints.');
            end
            if full - empty > 1e-12*max(1,full)
                fraction = (mass - empty)/(full - empty);
                cg = (empty*cgEmpty + (mass-empty) * ...
                    ((full*cgFull - empty*cgEmpty)/(full-empty)))/mass;
                % Interpolate tensors about a COMMON datum, then transfer
                % back to current CG. Direct CG-to-CG interpolation is wrong.
                IFullDatum = IFull + MassPropertiesEvaluator.shift(full, cgFull);
                IEmptyDatum = IEmpty + MassPropertiesEvaluator.shift(empty, cgEmpty);
                I = (1-fraction)*IEmptyDatum + fraction*IFullDatum ...
                    - MassPropertiesEvaluator.shift(mass, cg);
            else
                cg = cgFull;
                I = IFull; % preserve configured fixed tensor or explicit full endpoint
            end
            UAVMassModel.checkTensor(I, false);
            s = struct('Mass',mass,'CG',cg,'Inertia',I,'PayloadMass',max(0,mass-empty),'MassRate',0);
        end

        function result = shift(mass, r)
            result = mass*(dot(r,r)*eye(3) - r*r');
        end
    end
    methods (Static, Access = private)
        function I = tensor(I)
            if isvector(I) && numel(I) == 3, I = diag(I); end
            UAVMassModel.checkTensor(I, false);
        end

        function result = value(s, field, fallback)
            result = fallback;
            if isfield(s,field), result = s.(field); end
        end
    end
end
