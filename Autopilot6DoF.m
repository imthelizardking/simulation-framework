classdef Autopilot6DoF < BaseController
    % AUTOPILOT6DOF Generates virtual commands (roll, pitch, yaw) 
    % based on guidance demands and body rates.
    
    properties
        % Gains
        K_p_pitch, K_d_pitch
        K_p_yaw,   K_d_yaw
        K_p_roll,  K_d_roll
    end
    
    methods
        function obj = Autopilot6DoF()
            % Default gains
            obj.K_p_pitch = 0.5; obj.K_d_pitch = 0.1;
            obj.K_p_yaw   = 0.5; obj.K_d_yaw   = 0.1;
            obj.K_p_roll  = 1.0; obj.K_d_roll  = 0.2;
        end
        
        % BaseController'ın zorunlu kıldığı abstract metod
        function ctrl_bus = step(obj, t, guid_bus, nav_state, veh_cfg, env)
            
            % Girdi olarak struct veya mock array gelmesine karşı koruma
            if isnumeric(guid_bus)
                guidance_cmd = guid_bus;
            elseif isfield(guid_bus, 'AccelCmd_Inertial')
                guidance_cmd = guid_bus.AccelCmd_Inertial;
            elseif isfield(guid_bus, 'AccelCmd')
                guidance_cmd = guid_bus.AccelCmd;
            else
                guidance_cmd = [0; 0; 0];
            end
            
            % Extract body rates
            p = nav_state(10); q = nav_state(11); r = nav_state(12);
            
            ay_cmd = guidance_cmd(2);
            az_cmd = guidance_cmd(3);
            
            % Pitch ve Yaw kanalları (P-D kontrol)
            pitch_cmd = obj.K_p_pitch * az_cmd - obj.K_d_pitch * q;
            yaw_cmd   = obj.K_p_yaw * ay_cmd - obj.K_d_yaw * r;
            
            % Roll kanalı (0 dereceye regüle etmeye çalışır)
            roll_cmd  = -obj.K_p_roll * nav_state(7) - obj.K_d_roll * p; 
            
            % Mixer'a gidecek sanal komutları ctrl_bus içine paketliyoruz
            virtual_cmds = [roll_cmd; pitch_cmd; yaw_cmd];
            ctrl_bus = struct('VirtualCmds', virtual_cmds);
        end
    end
end