classdef Controller_LongitudinalFin < BaseController
    methods
        function ctrl_bus = step(obj, ~, guid_bus, nav_state, veh_cfg, env)
            vel   = nav_state(4:6);
            pitch = nav_state(7);
            m     = nav_state(8);
            
            v_mag = norm(vel);
            gamma = atan2(vel(3), vel(1));
            alpha = pitch - gamma;
            
            q_bar = 0.5 * env.getDensity(nav_state(3)) * v_mag^2;
            denom = q_bar * veh_cfg.Aero.CL_alpha * veh_cfg.Aero.RefArea;
            
            if denom < 1e-6
                fin_delta = 0.0;
            else
                req_eff_alpha = guid_bus.AccelCmd * m / denom;
                fin_delta = req_eff_alpha - alpha;
            end
            
            fin_delta = max(min(fin_delta, obj.MaxDeflection), -obj.MaxDeflection);
            ctrl_bus = struct('FinDeflection', fin_delta);
        end
    end
end