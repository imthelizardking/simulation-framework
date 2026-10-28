classdef AeroCivilianUAV < BaseAero
    % Standalone low-speed fixed-wing UAV aero, independent of DATCOM data.
    % Uses a coefficient-derivative model, not full-alpha static tables.
    % Body axes: forward/right/down. Angles and controls are radians.
    % getForcesAndMoments matches the current framework's body-force path;
    % bodyLoads can also be used without any framework state conventions.
    properties
        Span (1,1) double {mustBeReal,mustBePositive,mustBeFinite} = 2
        Coefficients struct = struct('CD0',0.06)
        DeployedTableKey (1,:) char = ''
        DeploymentTime (1,1) double {mustBeReal,mustBeNonnegative} = Inf
        DeploymentDuration (1,1) double {mustBeReal,mustBeFinite,mustBeNonnegative} = 0
    end

    methods
        function obj = AeroCivilianUAV()
            obj.RefArea = 1;
            obj.RefLength = 0.5; % mean aerodynamic chord
            obj.Frame = 'NED';
        end

        function setConfiguration(obj, key, coefficients)
            key = char(key);
            if ~isvarname(key)
                error('AeroCivilianUAV:Key','Configuration key must be a valid field name.');
            end
            obj.validateCoefficients(coefficients);
            obj.Tables.(key) = coefficients;
        end

        function [force, moment, info] = getForcesAndMoments(obj, t, X, env, controls)
            if nargin < 5, controls = []; end
            if ~strcmpi(obj.Frame,'NED')
                error('AeroCivilianUAV:Frame','The framework adapter requires NED.');
            end
            validateattributes(X,{'double'},{'column','real','finite','numel',21});
            q = X(Dynamics6DoF.IDX_QUAT);
            if norm(q) < 1e-12
                error('AeroCivilianUAV:Quaternion','Quaternion must have nonzero norm.');
            end
            % Match the existing Dynamics6DoF force-path matrix convention.
            % This does not change the legacy quaternion/rate conventions.
            inertialToBody = Dynamics6DoF.quatToDCM(q/norm(q));
            altitude = CoordinateFrame.altitude(X,'NED');
            airVelocity = X(BaseDynamics.IDX_VEL) - env.getWind(altitude);
            velocityBody = inertialToBody * airVelocity;
            [force,moment,info] = obj.bodyLoads(t,velocityBody, ...
                X(BaseDynamics.IDX_BODYRATE),env.getDensity(altitude), ...
                env.getSpeedOfSound(altitude),controls);
        end

        function [force, moment, info] = bodyLoads(obj,t,velocity,rates,density,soundSpeed,controls)
            % Pure instantaneous lookup. No history/finite-difference state.
            % AlphaDot, when needed, is supplied by an external flight model.
            if nargin < 7, controls = []; end
            validateattributes(t,{'double'},{'scalar','real','finite'});
            validateattributes(velocity,{'double'},{'size',[3 1],'real','finite'});
            validateattributes(rates,{'double'},{'size',[3 1],'real','finite'});
            validateattributes(density,{'double'},{'scalar','real','finite','nonnegative'});
            validateattributes(soundSpeed,{'double'},{'scalar','real','finite','positive'});
            validateattributes(obj.RefArea,{'double'},{'scalar','real','finite','positive'});
            validateattributes(obj.RefLength,{'double'},{'scalar','real','finite','positive'});
            if obj.IForm ~= 1 || obj.FL ~= 1
                error('AeroCivilianUAV:FormFactors', ...
                    'Keep inherited IForm/FL at 1; configure drag/lift with the civilian coefficients.');
            end
            input = obj.controlInputs(controls);
            speed = norm(velocity);
            mach = speed/soundSpeed;
            c = obj.coefficientsAt(t,mach);
            alpha = 0; beta = 0;
            rateHat = zeros(3,1); alphaDotHat = 0;
            if speed >= 1e-3
                alpha = atan2(velocity(3),velocity(1));
                beta = asin(max(-1,min(1,velocity(2)/speed)));
                rateHat = rates .* [obj.Span;obj.RefLength;obj.Span]/(2*speed);
                if ~input.HasAlphaDot && (c.CL_alphaDot ~= 0 || c.Cm_alphaDot ~= 0)
                    error('AeroCivilianUAV:AlphaDot', ...
                        'Nonzero alpha-dot derivatives require controls.AlphaDot in rad/s.');
                end
                alphaDotHat = input.AlphaDot*obj.RefLength/(2*speed);
            end
            p = rateHat(1); q = rateHat(2); r = rateHat(3);
            de = input.Elevator; da = input.Aileron; dr = input.Rudder;
            lift = c.CL0 + c.CL_alpha*alpha + c.CL_q*q ...
                + c.CL_alphaDot*alphaDotHat + c.CL_elevator*de;
            drag = c.CD0 + c.CD_alpha2*alpha^2 + c.InducedDrag*lift^2 ...
                + c.CD_beta2*beta^2 + c.CD_elevator2*de^2;
            side = c.CY_beta*beta + c.CY_p*p + c.CY_r*r + c.CY_rudder*dr;
            roll = c.Cl_beta*beta + c.Cl_p*p + c.Cl_r*r + c.Cl_aileron*da + c.Cl_rudder*dr;
            pitch = c.Cm0 + c.Cm_alpha*alpha + c.Cm_q*q ...
                + c.Cm_alphaDot*alphaDotHat + c.Cm_elevator*de;
            yaw = c.Cn_beta*beta + c.Cn_p*p + c.Cn_r*r + c.Cn_aileron*da + c.Cn_rudder*dr;
            if drag < 0
                error('AeroCivilianUAV:NegativeDrag','Coefficient model produced negative drag.');
            end
            % Orthonormal wind basis expressed in body axes. Drag opposes
            % air-relative velocity; lift and side force are perpendicular.
            ca = cos(alpha); sa = sin(alpha); cb = cos(beta); sb = sin(beta);
            windToBody = [ca*cb,-ca*sb,-sa; sb,cb,0; sa*cb,-sa*sb,ca];
            pressureArea = 0.5*density*speed^2*obj.RefArea;
            force = pressureArea * windToBody * [-drag;side;-lift];
            moment = pressureArea * [obj.Span*roll;obj.RefLength*pitch;obj.Span*yaw];
            if speed < 1e-3
                force = zeros(3,1); moment = zeros(3,1);
            end
            info = struct('Alpha',alpha,'Beta',beta,'Mach',mach,'RateHat',rateHat, ...
                'AlphaDotHat',alphaDotHat,'ForceCoefficients',[drag;side;lift], ...
                'MomentCoefficients',[roll;pitch;yaw]);
        end

        function c = coefficientsAt(obj,t,mach)
            validateattributes(t,{'double'},{'scalar','real','finite'});
            validateattributes(mach,{'double'},{'scalar','real','finite','nonnegative'});
            ratio = obj.deploymentRatioAt(t);
            if isempty(obj.DeployedTableKey), ratio = 0; end
            if ratio < 1, c = obj.evaluateSet(obj.configuration(obj.ActiveTableKey),mach); end
            if ratio > 0
                deployed = obj.evaluateSet(obj.configuration(obj.DeployedTableKey),mach);
                if ratio == 1
                    c = deployed;
                else
                    for field = fieldnames(c)'
                        f = field{1}; c.(f) = (1-ratio)*c.(f) + ratio*deployed.(f);
                    end
                end
            end
        end

        function ratio = deploymentRatioAt(obj,t)
            validateattributes(t,{'double'},{'scalar','real','finite'});
            if isinf(obj.DeploymentTime)
                validateattributes(obj.DeploymentRatio,{'double'},{'scalar','real','finite','>=',0,'<=',1});
                ratio = obj.DeploymentRatio;
            elseif obj.DeploymentDuration == 0
                ratio = double(t >= obj.DeploymentTime);
            else
                ratio = max(0,min(1,(t-obj.DeploymentTime)/obj.DeploymentDuration));
            end
        end
    end

    methods (Access = private)
        function data = configuration(obj,key)
            if isfield(obj.Tables,key)
                data = obj.Tables.(key);
            elseif strcmp(key,'Clean')
                data = obj.Coefficients;
            else
                error('AeroCivilianUAV:Configuration','Unknown configuration: %s.',key);
            end
        end
    end

    methods (Static)
        function c = zeroCoefficients()
            % Explicit derivative representation. No DATCOM field aliases.
            names = {'CD0','CD_alpha2','InducedDrag','CD_beta2','CD_elevator2', ...
                'CL0','CL_alpha','CL_q','CL_alphaDot','CL_elevator', ...
                'CY_beta','CY_p','CY_r','CY_rudder', ...
                'Cl_beta','Cl_p','Cl_r','Cl_aileron','Cl_rudder', ...
                'Cm0','Cm_alpha','Cm_q','Cm_alphaDot','Cm_elevator', ...
                'Cn_beta','Cn_p','Cn_r','Cn_aileron','Cn_rudder'};
            c = cell2struct(num2cell(zeros(size(names))),names,2);
        end
    end

    methods (Static, Access = private)
        function validateCoefficients(data)
            if ~isstruct(data) || ~isscalar(data)
                error('AeroCivilianUAV:Coefficients','Coefficients must be a scalar struct.');
            end
            known = fieldnames(AeroCivilianUAV.zeroCoefficients());
            unknown = setdiff(fieldnames(data),known);
            if ~isempty(unknown)
                error('AeroCivilianUAV:Coefficients','Unknown civilian coefficient: %s.',unknown{1});
            end
        end

        function c = evaluateSet(data,mach)
            AeroCivilianUAV.validateCoefficients(data);
            c = AeroCivilianUAV.zeroCoefficients();
            for field = fieldnames(data)'
                f = field{1};
                c.(f) = BaseAero.evalCoeff(data.(f),mach);
                validateattributes(c.(f),{'double'},{'scalar','real','finite'});
            end
        end

        function input = controlInputs(data)
            input = struct('Elevator',0,'Aileron',0,'Rudder',0,'AlphaDot',0,'HasAlphaDot',false);
            if isempty(data), return; end
            if isnumeric(data)
                validateattributes(data,{'double'},{'scalar','real','finite'});
                input.Elevator = data;
                return;
            end
            if ~isstruct(data) || ~isscalar(data) || ...
                    ~isempty(setdiff(fieldnames(data),{'Elevator','Aileron','Rudder','AlphaDot'}))
                error('AeroCivilianUAV:Controls','Expected Elevator/Aileron/Rudder/AlphaDot fields or a scalar elevator.');
            end
            for field = fieldnames(data)'
                f = field{1};
                validateattributes(data.(f),{'double'},{'scalar','real','finite'});
                input.(f) = data.(f);
            end
            input.HasAlphaDot = isfield(data,'AlphaDot');
        end
    end
end
