classdef Mixer6DoF < handle
    % MIXER6DOF Distributes virtual control commands (roll, pitch, yaw) 
    % to physical aerodynamic control surfaces (canards/fins).
    
    properties
        allocation_matrix % 3xN or similar matrix mapping virtual cmds to physical deflections
        num_surfaces      % Total number of control surfaces (e.g., 4, 8, 12)
    end
    
    methods
        function obj = Mixer6DoF(alloc_mat, num_surf)
            if nargin > 0
                obj.allocation_matrix = alloc_mat;
                obj.num_surfaces = num_surf;
            else
                % Default allocation for a '+' configuration canards
                % cmd = [d_roll; d_pitch; d_yaw]
                % Canards: 1(Top), 2(Right), 3(Bottom), 4(Left)
                % Roll: all surfaces deflect in same direction to create roll torque
                % Pitch: surfaces 2 & 4 deflect
                % Yaw: surfaces 1 & 3 deflect
                obj.allocation_matrix = [
                    1,  0,  1;  % Fin 1 (Top)    - affects Roll, Yaw
                    1, -1,  0;  % Fin 2 (Right)  - affects Roll, Pitch
                    1,  0,  1;  % Fin 3 (Bottom) - affects Roll, Yaw
                    1,  1,  0;  % Fin 4 (Left)   - affects Roll, Pitch
                ];
                obj.num_surfaces = 4;
            end
        end
        
        function delta_cmds = allocate(obj, virtual_cmds)
            % virtual_cmds: [roll_cmd; pitch_cmd; yaw_cmd] in radians
            % delta_cmds: column vector of physical fin commands
            delta_cmds = obj.allocation_matrix * virtual_cmds;
        end
    end
end
