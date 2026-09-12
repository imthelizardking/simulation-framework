classdef AeroDatcom < BaseAero
    % Aero model driven directly by Missile DATCOM's printed "CHARACTERISTICS
    % AT ANGLE OF ATTACK" tables: CD and CL, each a 2D Alpha-by-Mach grid,
    % taken straight from DATCOM's own output rather than the CD0/CDA2/
    % CDA4/CLA/CLA3 polynomial-in-alpha buildup AeroTabulated uses.
    %
    % Earlier versions of this class stored DATCOM's body-axis CN/CA and
    % converted to wind-axis CD/CL here via the standard body->wind
    % transform. That conversion is no longer needed: DATCOM's real output
    % (confirmed against data/ex1_in.txt, a genuine Digital DATCOM run)
    % already lists CD and CL as their own columns, computed by DATCOM
    % itself - reading them directly avoids re-deriving something DATCOM
    % already did, and sidesteps any risk of a sign/convention mismatch in
    % that transform. See DatcomReader for how these tables are parsed out
    % of DATCOM's printed listing.
    properties
        Mach  (1,:) double = []  % Mach breakpoints
        Alpha (1,:) double = []  % Alpha breakpoints, RADIANS (DatcomReader converts from DATCOM's native degrees)
        CD    double = []        % numel(Alpha)-by-numel(Mach), drag coefficient (wind axis, as tabulated by DATCOM)
        CL    double = []        % numel(Alpha)-by-numel(Mach), lift coefficient (wind axis, as tabulated by DATCOM)
        % Approximate linear lift-curve slope (per rad) - used only by
        % controllers (e.g. LongitudinalFinController) that need a single
        % representative gain for their control law. The force computation
        % in getForcesAndMoments always uses the full nonlinear CD/CL
        % table above; this is a separate, simplified value for closed-
        % loop control design only. DatcomReader.load sets this from the
        % table's low-alpha slope - override directly for a different
        % representative value.
        CL_alpha (1,1) double = 3.0
    end

    methods
        function [F_aero, M_aero] = getForcesAndMoments(obj, t, X, env, fin_deflection)
            pitch = X(7);

            flow = BaseAero.computeFlowState(X, env);
            if flow.v_mag < 1e-3
                F_aero = [0; 0; 0];
                M_aero = [0; 0; 0];
                return;
            end
            vel_dir = flow.vel_dir;
            mach    = flow.mach;
            q_bar   = flow.q_bar;
            alpha   = (pitch - flow.gamma) + fin_deflection;

            if isempty(obj.Mach) || isempty(obj.Alpha)
                error('AeroDatcom:NoData', ...
                    '%s has no DATCOM table loaded - see DatcomReader.load.', class(obj));
            end

            % Clamped, not extrapolated, same philosophy as
            % BaseAero.evalCoeff's Table case and Environment's METCM
            % interpolation - this is measured/tabulated data, not a
            % formula, so holding the boundary value is safer than
            % extrapolating past the table's actual coverage.
            mach_c  = min(max(mach, min(obj.Mach)), max(obj.Mach));
            alpha_c = min(max(alpha, min(obj.Alpha)), max(obj.Alpha));

            CD = interp2(obj.Mach, obj.Alpha, obj.CD, mach_c, alpha_c, 'linear');
            CL = interp2(obj.Mach, obj.Alpha, obj.CL, mach_c, alpha_c, 'linear');

            F_drag = -(q_bar * CD * obj.RefArea) * vel_dir;

            lift_dir = [-vel_dir(3); 0; vel_dir(1)];
            if norm(lift_dir) > 0, lift_dir = lift_dir / norm(lift_dir); end
            F_lift = (q_bar * CL * obj.RefArea) * lift_dir;

            F_aero = F_drag + F_lift;
            M_aero = [0; 0; 0];
        end
    end
end
