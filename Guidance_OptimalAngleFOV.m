classdef Guidance_OptimalAngleFOV < BaseGuidance
    properties
        UseTerminalAngle (1,1) logical = false
        UseFOV           (1,1) logical = true
        MaxFOV           (1,1) double = deg2rad(20)
        FOVMargin        (1,1) double = deg2rad(15)
        FOVHysteresis    (1,1) double = deg2rad(3)
        DesiredAngle     (1,1) double = deg2rad(50)
        T_LOAL           (1,1) double = 0.0
        T_SeekerOn       (1,1) double = 2.0
        FOVActiveFlag    (1,1) logical = false
    end
    
    methods
        function obj = Guidance_OptimalAngleFOV()
            obj.MaxAccel = 40 * 9.81;
        end
        
        function guid_bus = step(obj, t, nav_state, ~, sensor_bus)
            seeker = sensor_bus.Seeker_LOAL;
            vel    = nav_state(4:6);
            pitch  = nav_state(7);
            
            R        = seeker.Range;
            V_c      = seeker.V_closure;
            los_rate = seeker.LOS_Rate;
            LOS      = seeker.LOS;
            gamma    = atan2(vel(3), vel(1));
            t_go     = max(R / V_c, 0.1);
            
            is_loal   = (t < obj.T_LOAL);
            is_funnel = (t >= obj.T_LOAL) && (t < obj.T_SeekerOn);
            
            % 1. Classical PN Component
            acc_pn = 3.5 * V_c * los_rate;
            
            % 2. FOV Constraint Protection
            acc_fov = 0.0;
            if obj.UseFOV && ~is_loal
                look_err = LOS - pitch;
                abs_look = abs(look_err);
                
                trig_on  = obj.MaxFOV - obj.FOVMargin;
                trig_off = trig_on - obj.FOVHysteresis;
                
                if ~obj.FOVActiveFlag && (abs_look > trig_on)
                    obj.FOVActiveFlag = true;
                elseif obj.FOVActiveFlag && (abs_look < trig_off)
                    obj.FOVActiveFlag = false;
                end
                
                if obj.FOVActiveFlag
                    ratio = max(0.0, min(1.0, (abs_look - trig_off) / (obj.MaxFOV - trig_off)));
                    acc_fov = obj.MaxAccel * ratio * sign(look_err);
                end
            else
                obj.FOVActiveFlag = false;
            end
            
            % 3. Terminal Angle Shaping Component
            acc_angle = 0.0;
            if obj.UseTerminalAngle
                ang_err = obj.DesiredAngle - gamma;
                base_shaping = -500.0 * ang_err / (max(t_go, 1.0)^1.5);
                
                funnel_mult = 1.0;
                if is_funnel
                    funnel_mult = max(0.0, 1.0 - (t - obj.T_LOAL) / (obj.T_SeekerOn - obj.T_LOAL));
                end
                if R < 100.0, funnel_mult = funnel_mult * max(0.0, R / 100.0); end
                acc_angle = base_shaping * funnel_mult;
            end
            
            % Arbitration: Prioritize FOV over Angle Shaping
            if obj.FOVActiveFlag
                raw_acc = acc_pn + acc_fov;
            else
                raw_acc = acc_pn + acc_angle;
            end
            
            a_cmd = max(min(raw_acc, obj.MaxAccel), -obj.MaxAccel);
            
            guid_bus = struct('AccelCmd', a_cmd, 'AccPN', acc_pn, ...
                              'AccAngle', acc_angle, 'AccFOV', acc_fov, ...
                              'FOVActive', obj.FOVActiveFlag);
        end
    end
end