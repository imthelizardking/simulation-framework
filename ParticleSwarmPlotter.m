%% File: ParticleSwarmPlotter.m
classdef ParticleSwarmPlotter < BaseVisualizer
    % Live view of the post-burst fragment swarm, in its own docked figure
    % zoomed in on the burst locus (not the full engagement bounds the main
    % Visualizer_LiveEngagement uses) - separate from and much tighter than the
    % parent munition's own plot, per "create new live plot and zoom into
    % there, show particles motion".
    %
    % ParticleSwarmSimulation calls init/step with a cell array of N
    % particle state vectors in the X/target_state slots BaseVisualizer's
    % single-munition signature was written for (bounds/gnc_bus aren't
    % used the same way here - the 3rd/4th init args are unused, and the
    % 4th step arg is the Alive mask instead of a gnc_bus struct); this is
    % a deliberate reuse of the same init/step/close shape, not a literal
    % single-munition visualizer.
    properties
        h_ax
        h_trail   cell = {}
        h_head    cell = {}
        h_target_line
        h_target_head
        h_info_text
        history   cell = {}   % 2-by-K [downrange; altitude] trail per particle
        target_history double = []
        NumParticles (1,1) double = 0
    end

    methods
        function obj = ParticleSwarmPlotter(decimation)
            if nargin > 0, obj.UpdateDecimation = decimation; end
        end

        function init(obj, states0, target0, ~)
            old_fig = findobj('Type', 'figure', 'Name', 'Particle Swarm Burst');
            if ~isempty(old_fig), close(old_fig); end

            obj.FigureHandle = figure('WindowStyle', 'docked', ...
                                      'Name', 'Particle Swarm Burst', 'Color', 'w');
            obj.h_ax = axes(obj.FigureHandle);
            hold(obj.h_ax, 'on'); grid(obj.h_ax, 'on'); box(obj.h_ax, 'on');
            xlabel(obj.h_ax, 'Downrange X (m)'); ylabel(obj.h_ax, 'Altitude (m)');
            title(obj.h_ax, 'Fragment Swarm (post-burst)');

            n = numel(states0);
            obj.NumParticles = n;
            obj.history = cell(n, 1);
            obj.h_trail = cell(n, 1);
            obj.h_head  = cell(n, 1);
            colors = lines(max(n, 1));
            for i = 1:n
                pos = states0{i}(1:3);
                alt = CoordinateFrame.altitude(pos, obj.Frame);
                obj.history{i} = [pos(1); alt];
                obj.h_trail{i} = plot(obj.h_ax, pos(1), alt, '-', 'Color', colors(i,:), 'LineWidth', 1.0);
                obj.h_head{i}  = plot(obj.h_ax, pos(1), alt, 'o', 'MarkerSize', 4, ...
                                      'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', colors(i,:));
            end

            target_alt = CoordinateFrame.altitude(target0.pos, obj.Frame);
            obj.target_history = target0.pos;
            obj.h_target_line = plot(obj.h_ax, target0.pos(1), target_alt, 'r--', 'LineWidth', 1.5);
            obj.h_target_head = plot(obj.h_ax, target0.pos(1), target_alt, 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 8);

            obj.h_info_text = text(obj.h_ax, 0.02, 0.95, '', 'Units', 'normalized', ...
                'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', 'white', 'EdgeColor', 'black');

            obj.zoomToSwarm(states0, target0.pos, true);
        end

        function updatePlot(obj, t, states, target_state, alive)
            n_alive = 0;
            for i = 1:obj.NumParticles
                pos = states{i}(1:3);
                alt = CoordinateFrame.altitude(pos, obj.Frame);
                obj.history{i} = [obj.history{i}, [pos(1); alt]];
                set(obj.h_trail{i}, 'XData', obj.history{i}(1,:), 'YData', obj.history{i}(2,:));
                set(obj.h_head{i}, 'XData', pos(1), 'YData', alt);
                if alive(i)
                    n_alive = n_alive + 1;
                else
                    set(obj.h_head{i}, 'MarkerFaceColor', [0.7 0.7 0.7], 'MarkerEdgeColor', [0.7 0.7 0.7]);
                end
            end

            obj.target_history = [obj.target_history, target_state.pos];
            % 'FDR' keeps altitude at row 2 (down) instead of 'NED''s row
            % 3 - see CoordinateFrame.m's class comment.
            is_fdr = strcmpi(obj.Frame, 'FDR');
            alt_row = 3; if is_fdr, alt_row = 2; end
            is_ned_or_fdr = strcmpi(obj.Frame, 'NED') || is_fdr;
            target_alt_hist = obj.target_history(alt_row,:); if is_ned_or_fdr, target_alt_hist = -target_alt_hist; end
            target_alt_now  = CoordinateFrame.altitude(target_state.pos, obj.Frame);
            set(obj.h_target_line, 'XData', obj.target_history(1,:), 'YData', target_alt_hist);
            set(obj.h_target_head, 'XData', target_state.pos(1), 'YData', target_alt_now);

            set(obj.h_info_text, 'String', sprintf('t = %.3f s\nAlive: %d / %d', t, n_alive, obj.NumParticles));

            obj.zoomToSwarm(states, target_state.pos, false);
        end

        function close(obj)
            if isvalid(obj.FigureHandle), close(obj.FigureHandle); end
        end
    end

    methods (Access = private)
        function zoomToSwarm(obj, states, target_pos, force)
            % Zoomed to the burst/swarm locus rather than the full
            % engagement: bounds start tight around the burst point and
            % expand only when a live particle actually approaches the
            % current edge, so the view stays "zoomed in" instead of
            % snapping out to the full trajectory scale on the first step.
            xs = zeros(numel(states), 1);
            zs = zeros(numel(states), 1);
            for i = 1:numel(states)
                xs(i) = states{i}(1);
                zs(i) = CoordinateFrame.altitude(states{i}, obj.Frame);
            end
            xs(end+1) = target_pos(1); %#ok<AGROW>
            zs(end+1) = CoordinateFrame.altitude(target_pos, obj.Frame); %#ok<AGROW>

            x_lo = min(xs); x_hi = max(xs);
            z_lo = min(zs); z_hi = max(zs);
            margin_x = max(10.0, 0.25 * (x_hi - x_lo));
            margin_z = max(10.0, 0.25 * (z_hi - z_lo));
            new_bounds = [x_lo - margin_x, x_hi + margin_x, min(0, z_lo - margin_z), z_hi + margin_z];

            cur = axis(obj.h_ax);
            needs_expand = force || any([x_lo, z_lo] < cur([1 3])) || any([x_hi, z_hi] > cur([2 4]));
            if needs_expand
                axis(obj.h_ax, new_bounds);
            end
        end
    end
end
