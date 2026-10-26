classdef Visualizer_Basic < BaseVisualizer
    properties
        h_main_ax
        h_traj_line
        h_missile_head
        h_time_speed_text
        traj_history double = []
        AxisLimits   double = [] 
    end
    
    methods
        function obj = Visualizer_Basic(decimation, limits)
            if nargin > 0, obj.UpdateDecimation = decimation; end
            if nargin > 1, obj.AxisLimits = limits; end
        end
        
        function init(obj, x0, ~, ~)
            old_fig = findobj('Type', 'figure', 'Name', 'Basic Trajectory');
            if ~isempty(old_fig), close(old_fig); end
            
            obj.FigureHandle = figure('WindowStyle', 'docked', ...
                                      'Name', 'Basic Trajectory', 'Color', 'w');
            
            obj.h_main_ax = axes(obj.FigureHandle);
            hold(obj.h_main_ax, 'on'); grid(obj.h_main_ax, 'on'); box(obj.h_main_ax, 'on');
            xlabel(obj.h_main_ax, 'Downrange X (m)'); ylabel(obj.h_main_ax, 'Altitude (m)');
            title(obj.h_main_ax, 'Live Trajectory');
            
            if ~isempty(obj.AxisLimits)
                axis(obj.h_main_ax, obj.AxisLimits);
            end
            
            obj.h_traj_line = plot(obj.h_main_ax, NaN, NaN, 'b-', 'LineWidth', 2);
            obj.h_missile_head = plot(obj.h_main_ax, x0(1), CoordinateFrame.altitude(x0, obj.Frame), 'bo', 'MarkerFaceColor', 'b');
            
            obj.h_time_speed_text = text(obj.h_main_ax, 0.05, 0.90, 'Time: 0.000 s\nSpeed: 0 m/s', ...
                'Units', 'normalized', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue', ...
                'BackgroundColor', 'white', 'EdgeColor', 'black');
        end
        
        function updatePlot(obj, t, X, ~, ~)
            pos   = X(1:3);
            vel   = X(4:6);
            speed = norm(vel);
            
            obj.traj_history = [obj.traj_history, pos];

            is_fdr = strcmpi(obj.Frame, 'FDR');
            alt_row = 3; if is_fdr, alt_row = 2; end
            is_ned_or_fdr = strcmpi(obj.Frame, 'NED') || is_fdr;
            
            traj_alt = obj.traj_history(alt_row,:);
            if is_ned_or_fdr, traj_alt = -traj_alt; end
            
            alt = CoordinateFrame.altitude(pos, obj.Frame);

            set(obj.h_traj_line, 'XData', obj.traj_history(1,:), 'YData', traj_alt);
            set(obj.h_missile_head, 'XData', pos(1), 'YData', alt);
            set(obj.h_time_speed_text, 'String', sprintf('Time: %.3f s\nSpeed: %d m/s', t, round(speed)));
        end
        
        function close(obj)
            if isvalid(obj.FigureHandle), close(obj.FigureHandle); end
        end
    end
end