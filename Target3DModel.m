classdef Target3DModel < BaseTarget
    properties
        Faces            % N x 3 array of vertex indices for each triangle
        Vertices_Body    % N x 3 array of local vertices from the STL
        Vertices_Global  % N x 3 array of dynamically transformed vertices in NED
        BoundingRadius   % Maximum vertex distance from CG for fast-pass collision checks
    end
    
    methods
        function obj = Target3DModel(initial_state, stl_filename)
            % Initialize the parent kinematic target class
            obj = obj@BaseTarget(initial_state);
            
            % Read the 3D model geometry
            tr = stlread(stl_filename);
            obj.Faces = tr.ConnectivityList;
            obj.Vertices_Body = tr.Points;
            
            % Preallocate global vertices to prevent memory reallocation in loops
            obj.Vertices_Global = zeros(size(obj.Vertices_Body));
            
            % Calculate a bounding sphere for preliminary collision culling
            obj.BoundingRadius = max(vecnorm(obj.Vertices_Body, 2, 2));
            
            % Perform the initial coordinate transformation
            obj = obj.updateGeometry();
        end
        
        function obj = update(obj, dt)
            % Update the kinematic state (position and attitude) using the parent method
            obj = update@BaseTarget(obj, dt);
            
            % Transform the 3D mesh into the new global position and orientation
            obj = obj.updateGeometry();
        end
        
        function obj = updateGeometry(obj)
            % Map local vertices to global NED frame: V_global = V_body * R_bi^T + Pos
            % Assumes obj.R_bi is the body-to-inertial rotation matrix and obj.Pos is a 3x1 vector
            
            rotated_vertices = obj.Vertices_Body * obj.R_bi';
            obj.Vertices_Global = rotated_vertices + obj.Pos';
        end
    end
end