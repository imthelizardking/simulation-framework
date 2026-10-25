classdef Aero3DoF < BaseAero
    properties
        CL_alpha = 4.0 % scalar, or a Mach-dependent struct (see BaseAero.evalCoeff)
        BaseBleed = [] % optional Propulsion_DYU - reduces CD0 while burning
    end

    methods
        function obj = Aero3DoF(cd0, cl_alpha, ref_area)
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
            alpha = pitch - flow.gamma;

            coeffs = obj.evaluateCoefficients(flow.mach);

            F_drag = obj.dragEquation(coeffs, t, flow);
            F_lift = obj.liftEquation(coeffs, alpha, fin_deflection, flow);

            F_aero = F_drag + F_lift;
            M_aero = [0; 0; 0];
        end

        function coeffs = evaluateCoefficients(obj, mach)
            coeffs.CD0      = BaseAero.evalCoeff(obj.CD0, mach);
            coeffs.CL_alpha = BaseAero.evalCoeff(obj.CL_alpha, mach);
        end

        function F_drag = dragEquation(obj, coeffs, t, flow)
            cd0_val = coeffs.CD0;
            if ~isempty(obj.BaseBleed)
                cd0_val = cd0_val * obj.BaseBleed.dragMultiplier(t);
            end
            F_drag = -(flow.q_bar * cd0_val * obj.RefArea) * flow.vel_dir;
        end

        function F_lift = liftEquation(obj, coeffs, alpha, fin_deflection, flow)
            effective_alpha = alpha + fin_deflection;
            f_lift = flow.q_bar * coeffs.CL_alpha * obj.RefArea * effective_alpha;

            lift_dir = cross(flow.vel_dir, CoordinateFrame.lateralAxis(obj.Frame));
            if norm(lift_dir) > 0, lift_dir = lift_dir / norm(lift_dir); end
            F_lift = f_lift * lift_dir;
        end
    end
end
