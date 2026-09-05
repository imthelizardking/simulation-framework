classdef (Abstract) BaseAero < handle
    properties
        RefArea         (1,1) double = 0.01966 % S_ref (m^2)
        CD0                           = 0.35    % Zero-lift drag: scalar, or a Mach-dependent struct (see evalCoeff)
        RefLength       (1,1) double = 0.155   % d_ref (m)
        Tables          struct = struct()      % Named coefficient sets, e.g. Tables.Clean.CD0/.CDA2/.CDA4/.CLA/.CLA3
        ActiveTableKey  (1,:) char = 'Clean'
        DeploymentRatio (1,1) double = 0.0
    end
    methods (Abstract)
        [F_aero, M_aero] = getForcesAndMoments(obj, t, X, env, fin_deflections)
    end
    methods (Static)
        function val = evalCoeff(coeff, mach)
            % Evaluates one aero coefficient at a given Mach number. coeff is:
            %   - a plain scalar          -> returned unchanged (Mach-independent)
            %   - struct('Type','Table', 'Mach',[...], 'Value',[...])
            %       -> linearly interpolated, clamped to the table's Mach range
            %   - struct('Type','Polynomial', 'a0',.., 'a1',.., 'a2',.., ...)
            %       -> value = sum(a_i * mach^i) over whichever a-fields are present
            if ~isstruct(coeff)
                val = coeff;
                return;
            end
            switch coeff.Type
                case 'Table'
                    m_clamped = min(max(mach, min(coeff.Mach)), max(coeff.Mach));
                    val = interp1(coeff.Mach, coeff.Value, m_clamped, 'linear');
                case 'Polynomial'
                    val = 0;
                    fn = fieldnames(coeff);
                    for i = 1:numel(fn)
                        tok = regexp(fn{i}, '^a(\d+)$', 'tokens', 'once');
                        if ~isempty(tok)
                            order = str2double(tok{1});
                            val = val + coeff.(fn{i}) * mach^order;
                        end
                    end
                otherwise
                    error('BaseAero:evalCoeff', 'Unknown aero coefficient Type: %s', coeff.Type);
            end
        end

        function flow = computeFlowState(X, env)
            % Shared velocity/Mach/dynamic-pressure calculation every
            % concrete Aero (and DynamicsMPMM's yaw-of-repose) class needs -
            % reads density/speed-of-sound at the current altitude (X(3),
            % Z-up) from env rather than duplicating this per class.
            % vel_dir/gamma are zeroed when v_mag is ~0 rather than
            % producing NaN; callers still decide their own "too slow to
            % matter" early-return threshold.
            vel   = X(4:6);
            v_mag = norm(vel);
            altitude = X(3);

            flow.vel   = vel;
            flow.v_mag = v_mag;
            flow.mach  = v_mag / env.getSpeedOfSound(altitude);
            flow.q_bar = 0.5 * env.getDensity(altitude) * v_mag^2;

            if v_mag > 1e-3
                flow.vel_dir = vel / v_mag;
                flow.gamma   = atan2(vel(3), vel(1));
            else
                flow.vel_dir = [0; 0; 0];
                flow.gamma   = 0;
            end
        end
    end
end
