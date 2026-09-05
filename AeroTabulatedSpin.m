classdef AeroTabulatedSpin < AeroTabulated
    % AeroTabulated plus the two extra effects a spin-stabilized shell
    % needs that a fin-stabilized planar model doesn't: a Magnus (spin x
    % angle-of-attack) side force, and the spin-damping moment that slows
    % the shell's roll rate over the flight. Add Cl_p and CMagnus_pa to
    % the same Tables.(ActiveTableKey) coefficient struct AeroTabulated
    % already reads CD0/CDA2/CDA4/CLA/CLA3 from - each may be a plain
    % scalar or a Mach-dependent Table/Polynomial struct, per
    % BaseAero.evalCoeff. Reads spin rate from X(9), consistent with
    % Dynamics3DofSpin's 9-state layout - its usual pairing.
    %
    % The overturning-moment / yaw-of-repose coupling a full
    % spin-stabilized treatment also has is NOT modeled here - that needs
    % a coupled attitude state (see DynamicsMPMM instead); this class only
    % adds the two effects that are self-contained given the pitch-plane
    % alpha it inherits from AeroTabulated.
    methods
        function [F_aero, M_aero] = getForcesAndMoments(obj, t, X, env, fin_deflection)
            [F_aero, M_aero] = getForcesAndMoments@AeroTabulated(obj, t, X, env, fin_deflection);

            flow = BaseAero.computeFlowState(X, env);
            if flow.v_mag < 1e-3
                return;
            end
            v_mag   = flow.v_mag;
            vel_dir = flow.vel_dir;
            mach    = flow.mach;
            q_bar   = flow.q_bar;

            pitch   = X(7);
            spin    = X(9); % rad/s, roll rate p - see Dynamics3DofSpin
            alpha   = (pitch - flow.gamma) + fin_deflection;

            coeffs    = obj.Tables.(obj.ActiveTableKey);
            Clp_v     = BaseAero.evalCoeff(coeffs.Cl_p,       mach);
            CMagnus_v = BaseAero.evalCoeff(coeffs.CMagnus_pa, mach);

            % Magnus force: acts out of the pitch plane, along
            % spin_axis x lift_dir, proportional to spin rate, alpha, and
            % dynamic pressure (McCoy, "Modern Exterior Ballistics", ch. 9)
            lift_dir = [-vel_dir(3); 0; vel_dir(1)];
            if norm(lift_dir) > 0, lift_dir = lift_dir / norm(lift_dir); end
            magnus_dir = cross(vel_dir, lift_dir);
            if norm(magnus_dir) > 0, magnus_dir = magnus_dir / norm(magnus_dir); end

            spin_nd  = spin * obj.RefLength / v_mag; % nondimensional spin rate, p*d/V
            F_magnus = q_bar * CMagnus_v * obj.RefArea * spin_nd * alpha * magnus_dir;

            % Spin-damping moment about the roll axis (Cl_p < 0 decelerates spin)
            M_roll = q_bar * obj.RefArea * obj.RefLength * Clp_v * spin_nd / 2;

            F_aero = F_aero + F_magnus;
            M_aero = M_aero + [M_roll; 0; 0];
        end
    end
end
