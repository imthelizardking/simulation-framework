classdef Aero3DoF_Longitudinal < BaseAero
    properties
        CL_alpha = 4.0 % scalar, or a Mach-dependent struct (see BaseAero.evalCoeff)
    end
    
    methods
        function obj = Aero3DoF_Longitudinal(cd0, cl_alpha, ref_area)
            obj.CD0 = cd0;
            obj.CL_alpha = cl_alpha;
            obj.RefArea = ref_area;
        end
        
        function [F_aero, M_aero] = getForcesAndMoments(obj, X, env, fin_deflection)
            vel   = X(4:6);
            pitch = X(7);
            v_mag = norm(vel);
            
            if v_mag < 1e-3
                F_aero = [0; 0; 0];
                M_aero = [0; 0; 0];
                return;
            end
            
            vel_dir = vel / v_mag;
            gamma   = atan2(vel(3), vel(1));
            alpha   = pitch - gamma;
            mach    = v_mag / env.SpeedOfSound;

            q_bar = 0.5 * env.Density * v_mag^2;

            % Drag
            cd0_val = BaseAero.evalCoeff(obj.CD0, mach);
            f_drag = q_bar * cd0_val * obj.RefArea;
            F_drag = -f_drag * vel_dir;

            % Lift
            effective_alpha = alpha + fin_deflection;
            cl_alpha_val = BaseAero.evalCoeff(obj.CL_alpha, mach);
            f_lift = q_bar * cl_alpha_val * obj.RefArea * effective_alpha;
            
            lift_dir = [-vel_dir(3); 0; vel_dir(1)];
            if norm(lift_dir) > 0, lift_dir = lift_dir / norm(lift_dir); end
            F_lift = f_lift * lift_dir;
            
            F_aero = F_drag + F_lift;
            M_aero = [0; 0; 0];
        end
    end
end