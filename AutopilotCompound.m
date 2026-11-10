classdef AutopilotCompound < BaseController
    properties
        Mixer
        K_p_pitch, K_d_pitch
        K_p_yaw,   K_d_yaw
        K_p_roll,  K_d_roll
    end
    
    methods
        function obj = AutopilotCompound()
            obj.Mixer = Mixer6DoF(); % '+' konfigürasyon mikseri
            obj.K_p_pitch = 0.5; obj.K_d_pitch = 0.1;
            obj.K_p_yaw   = 0.5; obj.K_d_yaw   = 0.1;
            obj.K_p_roll  = 1.0; obj.K_d_roll  = 0.2;
        end
        
        function ctrl_bus = step(obj, t, guid_bus, nav_state, veh_cfg, env)
            if isfield(guid_bus, 'AccelCmd_Inertial')
                cmd = guid_bus.AccelCmd_Inertial;
            elseif isfield(guid_bus, 'AccelCmd')
                cmd = guid_bus.AccelCmd;
            else
                cmd = [0;0;0];
            end
            
            p = nav_state(10); q = nav_state(11); r = nav_state(12);
            
            pitch_cmd = obj.K_p_pitch * cmd(3) - obj.K_d_pitch * q;
            yaw_cmd   = obj.K_p_yaw * cmd(2) - obj.K_d_yaw * r;
            roll_cmd  = -obj.K_p_roll * nav_state(7) - obj.K_d_roll * p; 
            
            % Mikser 4 kanal komut üretir
            virtual_cmds = [roll_cmd; pitch_cmd; yaw_cmd];
            delta_cmds = obj.Mixer.allocate(virtual_cmds); 
            
            % 12 kanala pad'le (gelecek için)
            full_cmds = zeros(12,1);
            full_cmds(1:length(delta_cmds)) = delta_cmds;
            
            % Framework bu değişkeni Munition.getDerivatives'de okur
            ctrl_bus = struct('FinDeflection', full_cmds);
        end
    end
end