classdef Seeker_LOAL < BaseSensor
    properties
        MaxFOV      (1,1) double = deg2rad(20)
        T_SeekerOn  (1,1) double = 2.0
    end
    
    methods
        function obj = Seeker_LOAL(max_fov, t_seeker_on)
            obj.Name = 'Seeker_LOAL';
            obj.MaxFOV = max_fov;
            obj.T_SeekerOn = t_seeker_on;
        end
        
        function meas = step(obj, t, truth_state, target, ~)
            pos   = truth_state(1:3);
            vel   = truth_state(4:6);
            pitch = truth_state(7);
            
            r_vec = target.pos - pos;
            dist  = norm(r_vec);
            LOS   = atan2(r_vec(3), r_vec(1));
            
            v_rel = target.vel - vel;
            los_rate = (r_vec(1)*v_rel(3) - r_vec(3)*v_rel(1)) / max(dist^2, 1e-3);
            look_angle = abs(LOS - pitch);
            
            % Blindness & LOAL Mode Transition
            if t < obj.T_SeekerOn
                is_blind = false;
                meas_los_rate = los_rate; % Virtual INS LOS
                is_locked = false;
            else
                if look_angle > obj.MaxFOV
                    is_blind = true;
                    meas_los_rate = 0.0;  % Seeker Blind
                    is_locked = false;
                else
                    is_blind = false;
                    meas_los_rate = los_rate;
                    is_locked = true;
                end
            end
            
            V_c = -dot(v_rel, r_vec / max(dist, 1e-3));
            
            meas = struct('Range', dist, 'LOS', LOS, 'LOS_Rate', meas_los_rate, ...
                          'LookAngle', look_angle, 'V_closure', max(V_c, 10.0), ...
                          'IsBlind', is_blind, 'IsLocked', is_locked);
        end
    end
end