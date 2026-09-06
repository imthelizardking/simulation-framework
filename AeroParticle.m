%% File: AeroParticle.m
classdef AeroParticle < BaseAero
    % Drag-only aero for a fragment/sub-munition particle: no lift, no
    % moments. A fragment tumbles with no attitude tracked (Particle uses
    % Dynamics3Dof, which has no real attitude state), so rather than model
    % a time-varying presented area we resolve one fixed RefArea up front
    % from Shape/Dimensions/ReferenceAreaMode - see computeRefArea.
    properties
        Shape             (1,:) char = 'Sphere' % 'Cylinder' | 'Sphere' | 'Cube' (alias 'Square')
        Dimensions        struct     = struct()
        ReferenceAreaMode (1,:) char = 'Max'     % 'Min' | 'Max'
    end

    methods
        function obj = AeroParticle(cd, shape, dims, ref_mode)
            obj.CD0 = cd;
            obj.Shape = shape;
            obj.Dimensions = dims;
            if nargin > 3 && ~isempty(ref_mode)
                obj.ReferenceAreaMode = ref_mode;
            end
            obj.RefArea = AeroParticle.computeRefArea(obj.Shape, obj.Dimensions, obj.ReferenceAreaMode);
        end

        function [F_aero, M_aero] = getForcesAndMoments(obj, ~, X, env, ~)
            flow = BaseAero.computeFlowState(X, env);
            if flow.v_mag < 1e-3
                F_aero = [0; 0; 0];
                M_aero = [0; 0; 0];
                return;
            end
            cd_val = BaseAero.evalCoeff(obj.CD0, flow.mach);
            F_aero = -flow.q_bar * cd_val * obj.RefArea * flow.vel_dir;
            M_aero = [0; 0; 0];
        end
    end

    methods (Static)
        function area = computeRefArea(shape, dims, mode)
            % Resolves one fixed reference area for a tumbling, non-attitude-
            % tracked fragment: 'Min' is the smallest cross-section the
            % shape can present (nose/face-on), 'Max' the largest (broadside/
            % edge-on). Sphere has no min/max distinction.
            switch lower(shape)
                case 'sphere'
                    area = pi * dims.Radius^2;
                case 'cylinder'
                    area_min = pi * (dims.Diameter/2)^2;      % end-on (frontal circle)
                    area_max = dims.Diameter * dims.Length;   % broadside (rectangular projection)
                    area = AeroParticle.pickArea(area_min, area_max, mode);
                case {'cube', 'square'}
                    area_min = dims.Side^2;                   % face-on
                    area_max = dims.Side^2 * sqrt(2);         % edge-on (rotated 45 deg)
                    area = AeroParticle.pickArea(area_min, area_max, mode);
                otherwise
                    error('AeroParticle:computeRefArea', 'Unknown particle Shape "%s".', shape);
            end
        end
    end

    methods (Static, Access = private)
        function area = pickArea(area_min, area_max, mode)
            switch lower(mode)
                case 'min'
                    area = area_min;
                case 'max'
                    area = area_max;
                otherwise
                    error('AeroParticle:computeRefArea', 'ReferenceAreaMode must be ''Min'' or ''Max'', got "%s".', mode);
            end
        end
    end
end
