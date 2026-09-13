classdef LiveEngagementPlotter < BaseVisualizer
    properties
        h_main_ax
        h_bar_ax
        h_traj_line
        h_target_line
        h_missile_head
        h_target_head
        h_fov_upper
        h_fov_lower
        h_bar_pn
        h_bar_ang
        h_bar_fov
        h_txt_pn
        h_txt_ang
        h_txt_fov
        h_phase_text
        h_fov_text
        h_time_speed_text
        traj_history   double = []
        target_history double = []
    end
    
    methods
        function obj = LiveEngagementPlotter(decimation)
            if nargin > 0, obj.UpdateDecimation = decimation; end
        end
        
        function init(obj, x0, target0, bounds)
            old_fig = findobj('Type', 'figure', 'Name', 'Online Engagement Animation');
            if ~isempty(old_fig), close(old_fig); end
            
            obj.FigureHandle = figure('WindowStyle', 'docked', ...
                                      'Name', 'Online Engagement Animation', 'Color', 'w');
            
            obj.h_main_ax = axes(obj.FigureHandle);
            hold(obj.h_main_ax, 'on'); grid(obj.h_main_ax, 'on'); box(obj.h_main_ax, 'on');
            xlabel(obj.h_main_ax, 'Range X (m)'); ylabel(obj.h_main_ax, 'Altitude (m)');
            title(obj.h_main_ax, 'Engagement Trajectory');
            axis(obj.h_main_ax, bounds);
            
            obj.h_fov_upper = plot(obj.h_main_ax, NaN, NaN, '--', 'Color', [0 0.8 0], 'LineWidth', 1.2);
            obj.h_fov_lower = plot(obj.h_main_ax, NaN, NaN, '--', 'Color', [0 0.8 0], 'LineWidth', 1.2);
            
            obj.h_traj_line   = plot(obj.h_main_ax, NaN, NaN, 'b-', 'LineWidth', 2, 'DisplayName', 'Optimal Missile');
            obj.h_target_line = plot(obj.h_main_ax, NaN, NaN, 'r--', 'LineWidth', 2, 'DisplayName', 'Target');
            
            obj.h_missile_head = plot(obj.h_main_ax, x0(1), CoordinateFrame.altitude(x0, obj.Frame), 'bo', 'MarkerFaceColor', 'b');
            obj.h_target_head  = plot(obj.h_main_ax, target0.pos(1), CoordinateFrame.altitude(target0.pos, obj.Frame), 'ro', 'MarkerFaceColor', 'r');
            
            obj.h_phase_text = text(obj.h_main_ax, 0.45, 0.95, 'PHASE: LOAL (INS)', 'Units', 'normalized', ...
                'FontSize', 11, 'FontWeight', 'bold', 'Color', 'white', ...
                'BackgroundColor', [0.2 0.2 0.2], 'EdgeColor', 'black');
            obj.h_fov_text = text(obj.h_main_ax, 0.75, 0.14, 'FOV: 0.0 deg', 'Units', 'normalized', ...
                'FontSize', 10, 'FontWeight', 'bold', 'Color', 'black', ...
                'BackgroundColor', 'white', 'EdgeColor', 'black');
            obj.h_time_speed_text = text(obj.h_main_ax, 0.75, 0.05, 'Time: 0.000 s\nSpeed: 0 m/s', 'Units', 'normalized', ...
                'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue', ...
                'BackgroundColor', 'white', 'EdgeColor', 'black');
            
            obj.h_bar_ax = axes(obj.FigureHandle, 'Position', [0.72, 0.40, 0.18, 0.15]);
            hold(obj.h_bar_ax, 'on'); grid(obj.h_bar_ax, 'on');
            obj.h_bar_pn  = fill(obj.h_bar_ax, [0 0 0 0], [2.6 3.4 3.4 2.6], [0 0.5 0]);
            obj.h_bar_ang = fill(obj.h_bar_ax, [0 0 0 0], [1.6 2.4 2.4 1.6], [0.2 0.6 0.9]);
            obj.h_bar_fov = fill(obj.h_bar_ax, [0 0 0 0], [0.6 1.4 1.4 0.6], [0.9 0.2 0.2]);
            set(obj.h_bar_ax, 'YTick', [1, 2, 3], 'YTickLabel', {'FOV', 'Angle', 'PN'}, 'XTick', 0:25:100);
            xlim(obj.h_bar_ax, [0 120]); ylim(obj.h_bar_ax, [0.2 3.8]);
            title(obj.h_bar_ax, 'Command Breakdown', 'FontSize', 8, 'FontWeight', 'bold');
            
            obj.h_txt_pn  = text(obj.h_bar_ax, 105, 3, '0%', 'FontSize', 8, 'FontWeight', 'bold');
            obj.h_txt_ang = text(obj.h_bar_ax, 105, 2, '0%', 'FontSize', 8, 'FontWeight', 'bold');
            obj.h_txt_fov = text(obj.h_bar_ax, 105, 1, '0%', 'FontSize', 8, 'FontWeight', 'bold');
        end
        
        function updatePlot(obj, t, X, target_state, gnc_bus)
            pos   = X(1:3);
            vel   = X(4:6);
            pitch = X(7);
            speed = norm(vel);
            
            obj.traj_history   = [obj.traj_history, pos];
            obj.target_history = [obj.target_history, target_state.pos];

            is_ned = strcmpi(obj.Frame, 'NED');
            traj_alt   = obj.traj_history(3,:);   if is_ned, traj_alt   = -traj_alt;   end
            target_alt = obj.target_history(3,:); if is_ned, target_alt = -target_alt; end
            alt = CoordinateFrame.altitude(pos, obj.Frame);
            target_alt_now = CoordinateFrame.altitude(target_state.pos, obj.Frame);

            set(obj.h_traj_line, 'XData', obj.traj_history(1,:), 'YData', traj_alt);
            set(obj.h_target_line, 'XData', obj.target_history(1,:), 'YData', target_alt);
            set(obj.h_missile_head, 'XData', pos(1), 'YData', alt);
            set(obj.h_target_head, 'XData', target_state.pos(1), 'YData', target_alt_now);

            % Seeker/FOV cone and PN-Angle-FOV command breakdown only apply
            % to a seeker-guided munition; an unguided direct-fire round
            % has neither, so both sections degrade gracefully instead of
            % erroring on a missing field. NOTE: this cone's trig is only
            % verified for 'Z-Up' - no guided/seeker scenario in this
            % framework currently opts into 'NED', so it hasn't been
            % exercised or adapted for it.
            has_seeker = isfield(gnc_bus, 'Sensors') && isfield(gnc_bus.Sensors, 'SeekerLOAL');
            if has_seeker
                seeker = gnc_bus.Sensors.SeekerLOAL;
                fov_len = seeker.Range + 100;
                max_fov = deg2rad(20);
                x_up = pos(1) + fov_len * cos(pitch + max_fov);
                z_up = pos(3) + fov_len * sin(pitch + max_fov);
                x_dn = pos(1) + fov_len * cos(pitch - max_fov);
                z_dn = pos(3) + fov_len * sin(pitch - max_fov);

                fov_color = [0 0.8 0];
                if seeker.IsBlind, fov_color = [1 0 0]; end
                set(obj.h_fov_upper, 'XData', [pos(1), x_up], 'YData', [pos(3), z_up], 'Color', fov_color);
                set(obj.h_fov_lower, 'XData', [pos(1), x_dn], 'YData', [pos(3), z_dn], 'Color', fov_color);
                set(obj.h_fov_text, 'String', sprintf('FOV: %.1f deg', rad2deg(seeker.LookAngle)));
            else
                set(obj.h_fov_upper, 'XData', NaN, 'YData', NaN);
                set(obj.h_fov_lower, 'XData', NaN, 'YData', NaN);
                set(obj.h_fov_text, 'String', 'FOV: n/a (unguided)');
            end

            has_breakdown = isfield(gnc_bus, 'Guidance') && all(isfield(gnc_bus.Guidance, {'AccPN','AccAngle','AccFOV'}));
            if has_breakdown
                guid = gnc_bus.Guidance;
                tot_cmd = abs(guid.AccPN) + abs(guid.AccAngle) + abs(guid.AccFOV);
                if tot_cmd > 0
                    pct_pn  = round(100 * abs(guid.AccPN) / tot_cmd);
                    pct_ang = round(100 * abs(guid.AccAngle) / tot_cmd);
                    pct_fov = round(100 * abs(guid.AccFOV) / tot_cmd);
                else
                    pct_pn = 0; pct_ang = 0; pct_fov = 0;
                end
                set(obj.h_txt_pn,  'String', sprintf('%d%%', pct_pn));
                set(obj.h_txt_ang, 'String', sprintf('%d%%', pct_ang));
                set(obj.h_txt_fov, 'String', sprintf('%d%%', pct_fov));
            else
                pct_pn = 0; pct_ang = 0; pct_fov = 0;
                set(obj.h_txt_pn,  'String', 'n/a');
                set(obj.h_txt_ang, 'String', 'n/a');
                set(obj.h_txt_fov, 'String', 'n/a');
            end
            set(obj.h_bar_pn,  'XData', [0 pct_pn pct_pn 0]);
            set(obj.h_bar_ang, 'XData', [0 pct_ang pct_ang 0]);
            set(obj.h_bar_fov, 'XData', [0 pct_fov pct_fov 0]);

            set(obj.h_time_speed_text, 'String', sprintf('Time: %.3f s\nSpeed: %d m/s', t, round(speed)));
        end
        
        function close(obj)
            if isvalid(obj.FigureHandle), close(obj.FigureHandle); end
        end
    end
end