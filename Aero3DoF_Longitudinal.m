classdef Aero3DoF_Longitudinal < BaseAero
    properties
        CL_alpha = 4.0 % scalar, or a Mach-dependent struct (see BaseAero.evalCoeff)
        BaseBleed = [] % optional Propulsion_DYU - reduces CD0 while burning
    end

    methods
        function obj = Aero3DoF_Longitudinal(cd0, cl_alpha, ref_area)
            obj.CD0 = cd0;
            obj.CL_alpha = cl_alpha;
            obj.RefArea = ref_area;
        end

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
            alpha   = pitch - flow.gamma;

            % Drag
            cd0_val = BaseAero.evalCoeff(obj.CD0, mach);
            if ~isempty(obj.BaseBleed)
                cd0_val = cd0_val * obj.BaseBleed.dragMultiplier(t);
            end
            f_drag = q_bar * cd0_val * obj.RefArea;
            F_drag = -f_drag * vel_dir;

            % Lift
            effective_alpha = alpha + fin_deflection;
            cl_alpha_val = BaseAero.evalCoeff(obj.CL_alpha, mach);
            f_lift = q_bar * cl_alpha_val * obj.RefArea * effective_alpha;
            
            lift_dir = cross(vel_dir, CoordinateFrame.lateralAxis(obj.Frame));
            if norm(lift_dir) > 0, lift_dir = lift_dir / norm(lift_dir); end
            F_lift = f_lift * lift_dir;
            
            F_aero = F_drag + F_lift;
            M_aero = [0; 0; 0];
        end
    end
end