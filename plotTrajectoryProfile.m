function plotTrajectoryProfile(tel, target_pos, fig_title)
% Side-view (range vs altitude) and top-down (range vs crossrange) panels
% from Simulation telemetry. target_pos ([]/omitted to skip marker).
    if nargin < 2, target_pos = []; end
    if nargin < 3, fig_title = 'Trajectory'; end

    figure('Name', fig_title, 'Color', 'w');

    subplot(1, 2, 1);
    plot(tel.pos(:,1), tel.pos(:,3), 'b-', 'LineWidth', 1.5); hold on; grid on;
    if ~isempty(target_pos)
        plot(target_pos(1), target_pos(3), 'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    end
    xlabel('Downrange X (m)'); ylabel('Altitude Z (m)'); title('Side Profile');

    subplot(1, 2, 2);
    plot(tel.pos(:,1), tel.pos(:,2), 'b-', 'LineWidth', 1.5); hold on; grid on;
    if ~isempty(target_pos)
        plot(target_pos(1), target_pos(2), 'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    end
    xlabel('Downrange X (m)'); ylabel('Crossrange Y (m)'); title('Top-Down View');

    sgtitle(fig_title);
end
