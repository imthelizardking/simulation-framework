function [r, telemetry, sim] = runScenario(name)
%RUNSCENARIO Run an existing runner_<name>.m headless and return key metrics.
%   Reads the runner source unchanged, strips the lines that clear the
%   workspace, plot, or attach a live visualizer, evaluates the rest, and
%   pulls summary numbers out of the resulting telemetry. The runners
%   themselves are never modified.
    f = old_figs_off();
    cleanup = onCleanup(@() set(groot, 'DefaultFigureVisible', f)); %#ok<NASGU>

    path = which(['runner_' name '.m']);
    if isempty(path), error('runScenario:notFound', 'No runner_%s.m on the path.', name); end
    src = fileread(path);
    src = regexprep(src, '^\s*clear\b[^\n]*', '', 'lineanchors');
    src = regexprep(src, '^\s*plotTrajectoryProfile\([^\n]*', '', 'lineanchors');
    src = regexprep(src, '^\s*sim\.Visualizer\s*=[^\n]*', '', 'lineanchors');

    rng(0);
    evalc('eval(src)'); %#ok<EVLC> evalc swallows the runner's fprintf output

    r.reason = char(sim.TerminationReason); %#ok<NODEF>
    r.tof    = telemetry.time(end); %#ok<NODEF>
    r.finalPos = telemetry.pos(end, :);
    altitudeColumn = 3;
    if strcmpi(telemetry.Frame, 'FDR'), altitudeColumn = 2; end
    r.apogee = max(CoordinateFrame.altitudeColumn(telemetry.pos(:,altitudeColumn), telemetry.Frame));
    if isfield(telemetry, 'target_pos') && ~isempty(telemetry.target_pos)
        r.missDistance = norm(telemetry.pos(end,:) - telemetry.target_pos(end,:));
    end
end

function f = old_figs_off()
    f = get(groot, 'DefaultFigureVisible');
    set(groot, 'DefaultFigureVisible', 'off');
end
