classdef AeroTabulated < BaseAero
    % Aero model driven by a full Mach-dependent coefficient set (standard
    % McCoy-style buildup):
    %   CD(alpha, M) = CD0(M) + CDA2(M)*alpha^2 + CDA4(M)*alpha^4
    %   CL(alpha, M) = CLA(M)*alpha + CLA3(M)*alpha^3
    % Populate Tables.(ActiveTableKey) with CD0/CDA2/CDA4/CLA/CLA3, each
    % either a plain scalar or a Mach-dependent Table/Polynomial struct
    % (see BaseAero.evalCoeff). alpha here is the 2D pitch-plane angle of
    % attack, consistent with the planar Dynamics3Dof state this pairs
    % with - not a full 3D total-yaw angle.
    properties
        BaseBleed = [] % optional Propulsion_DYU - reduces CD0 while burning
    end
    methods
        function [F_aero, M_aero] = getForcesAndMoments(obj, t, X, env, fin_deflection)
            pitch = X(7);

            flow = BaseAero.computeFlowState(X, env, obj.Frame);
            if flow.v_mag < 1e-3
                F_aero = [0; 0; 0];
                M_aero = [0; 0; 0];
                return;
            end
            vel_dir = flow.vel_dir;
            mach    = flow.mach;
            q_bar   = flow.q_bar;
            alpha   = (pitch - flow.gamma) + fin_deflection;

            coeffs = obj.Tables.(obj.ActiveTableKey);
            CD0_v  = BaseAero.evalCoeff(coeffs.CD0,  mach);
            if ~isempty(obj.BaseBleed)
                CD0_v = CD0_v * obj.BaseBleed.dragMultiplier(t);
            end
            CDA2_v = BaseAero.evalCoeff(coeffs.CDA2, mach);
            CDA4_v = BaseAero.evalCoeff(coeffs.CDA4, mach);
            CLA_v  = BaseAero.evalCoeff(coeffs.CLA,  mach);
            CLA3_v = BaseAero.evalCoeff(coeffs.CLA3, mach);

            CD = CD0_v + CDA2_v * alpha^2 + CDA4_v * alpha^4;
            CL = CLA_v * alpha + CLA3_v * alpha^3;

            F_drag = -(q_bar * CD * obj.RefArea) * vel_dir;

            lift_dir = cross(vel_dir, CoordinateFrame.lateralAxis(obj.Frame));
            if norm(lift_dir) > 0, lift_dir = lift_dir / norm(lift_dir); end
            F_lift = (q_bar * CL * obj.RefArea) * lift_dir;

            F_aero = F_drag + F_lift;
            M_aero = [0; 0; 0];
        end
    end
end
