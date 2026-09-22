function plotTrajectoryProfile(tel, target_pos, fig_title)
% Side-view (range vs altitude) and top-down (range vs crossrange) panels
% from Simulation telemetry. target_pos ([]/omitted to skip marker).
    if nargin < 2, target_pos = []; end
    if nargin < 3, fig_title = 'Trajectory'; end

    frame = 'Z-Up';
    if isfield(tel, 'Frame'), frame = tel.Frame; end
    % 'FDR' keeps altitude at column 2 (down) and crossrange at column 3
    % (right) instead of 'NED'/'Z-Up''s column 3/column 2 - see
    % CoordinateFrame.m's class comment.
    if strcmpi(frame, 'FDR')
        alt_col   = 2;
        cross_col = 3;
    else
        alt_col   = 3;
        cross_col = 2;
    end
    if strcmpi(frame, 'NED') || strcmpi(frame, 'FDR')
        alt = -tel.pos(:,alt_col);
    else
        alt = tel.pos(:,alt_col);
    end
    if ~isempty(target_pos)
        target_alt = CoordinateFrame.altitude(target_pos, frame);
    end

    figure('Name', fig_title, 'Color', 'w');

    subplot(1, 2, 1);
    plot(tel.pos(:,1), alt, 'b-', 'LineWidth', 1.5); hold on; grid on;
    if ~isempty(target_pos)
        plot(target_pos(1), target_alt, 'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    end
    xlabel('Downrange X (m)'); ylabel('Altitude (m)'); title('Side Profile');

    subplot(1, 2, 2);
    plot(tel.pos(:,1), tel.pos(:,cross_col), 'b-', 'LineWidth', 1.5); hold on; grid on;
    if ~isempty(target_pos)
        plot(target_pos(1), target_pos(cross_col), 'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    end
    xlabel('Downrange X (m)'); ylabel('Crossrange Y (m)'); title('Top-Down View');

    sgtitle(fig_title);
end
