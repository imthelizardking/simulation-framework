classdef AeroTabulatedSpin < AeroTabulated
    % AeroTabulated plus the extra effects a spin-stabilized shell needs
    % that a fin-stabilized planar model doesn't: a pitch-axis overturning
    % moment (CMA/CMA3, alpha-dependent - McCoy's convention: POSITIVE for
    % a spin-stabilized shell, destabilizing without spin, the gyroscopic
    % effect is what keeps it stable), a Magnus (spin x angle-of-attack)
    % side force (CMAG), and the spin-damping moment that slows the
    % shell's roll rate over the flight (CSPIN). Standard field vocabulary
    % across every artillery preset in MunitionFactory (25/35/76/155mm):
    % CD0, CDA2, CDA4, CLA, CLA3 (read by the parent AeroTabulated) plus
    % CSPIN, CMA, CMA3, CMAG (read here) in Tables.(ActiveTableKey) - each
    % may be a plain scalar or a Mach-dependent Table/Polynomial struct,
    % per BaseAero.evalCoeff. Reads spin rate from X(9), consistent with
    % Dynamics3DofSpin's 9-state layout - its usual pairing.
    %
    % CMA/CMA3 are computed and returned in M_aero(2) but Dynamics3DofSpin
    % doesn't consume that slot yet (only M_aero(1), for spin) - pitch
    % itself is still the same first-order pitch-follower heuristic as the
    % parent chain, not a real moment-driven attitude state. The full
    % yaw-of-repose coupling this would feed (a coupled attitude state)
    % lives in DynamicsMPMM instead, also not yet wired to a preset.
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

            coeffs   = obj.Tables.(obj.ActiveTableKey);
            CSPIN_v  = BaseAero.evalCoeff(coeffs.CSPIN, mach);
            CMAG_v   = BaseAero.evalCoeff(coeffs.CMAG,  mach);
            CMA_v    = BaseAero.evalCoeff(coeffs.CMA,   mach);
            CMA3_v   = BaseAero.evalCoeff(coeffs.CMA3,  mach);

            % Magnus force: acts out of the pitch plane, along
            % spin_axis x lift_dir, proportional to spin rate, alpha, and
            % dynamic pressure (McCoy, "Modern Exterior Ballistics", ch. 9)
            lift_dir = [-vel_dir(3); 0; vel_dir(1)];
            if norm(lift_dir) > 0, lift_dir = lift_dir / norm(lift_dir); end
            magnus_dir = cross(vel_dir, lift_dir);
            if norm(magnus_dir) > 0, magnus_dir = magnus_dir / norm(magnus_dir); end

            spin_nd  = spin * obj.RefLength / v_mag; % nondimensional spin rate, p*d/V
            F_magnus = q_bar * CMAG_v * obj.RefArea * spin_nd * alpha * magnus_dir;

            % Spin-damping moment about the roll axis (CSPIN < 0 decelerates spin)
            M_roll = q_bar * obj.RefArea * obj.RefLength * CSPIN_v * spin_nd / 2;

            % Overturning moment about the pitch axis, same CMA*alpha +
            % CMA3*alpha^3 buildup as the CLA/CLA3 lift curve.
            CM = CMA_v * alpha + CMA3_v * alpha^3;
            M_pitch = q_bar * obj.RefArea * obj.RefLength * CM;

            F_aero = F_aero + F_magnus;
            M_aero = M_aero + [M_roll; M_pitch; 0];
        end
    end
end
