classdef ScenarioBuilderApp < handle
    % Phase-1 scenario builder: pick a munition, launcher position, and
    % firing mode (Direct - fixed elevation/azimuth, or Indirect -
    % FireControlComputer solves onto a target point), hit Run, and this
    % writes a real, standalone runner script (see generateScenarioRunner)
    % into generated/ and executes it - the same script you'd get writing
    % it by hand, just generated instead of typed. This app is purely a
    % control panel; it does not embed a live dashboard or touch any
    % existing MunitionFactory/FireControlComputer/Simulation code - it
    % calls those exactly the way any hand-written runner already does.
    %
    % No sensors/guidance/controller in this phase - see munitionCatalog
    % for the current preset list (all unguided direct/indirect-fire
    % rounds). Usage: ScenarioBuilderApp() from the command line, or via
    % the "Scenario Builder" entry point.
    properties (Access = private)
        Fig
        MunitionDropdown
        FiringModeDropdown
        LauncherXField
        LauncherYField
        LauncherZField
        DirectPanel
        ElevationField
        AzimuthField
        IndirectPanel
        TargetXField
        TargetYField
        TargetZField
        BranchDropdown
        RunButton
        StatusArea
    end

    methods
        function obj = ScenarioBuilderApp()
            % run() changes the current folder to the SCRIPT's own
            % directory while it executes - since generated scripts live
            % in generated/ (a subfolder), that CD away from this project
            % folder makes Environment/MunitionFactory/etc unresolvable
            % unless this folder is explicitly on the path (not just
            % reachable via CWD). addpath is additive/non-destructive, so
            % doing this once here is safe regardless of the user's own
            % path state.
            addpath(fileparts(mfilename('fullpath')));
            obj.buildUI();
        end
    end

    methods (Access = private)
        function buildUI(obj)
            obj.Fig = uifigure('Name', 'Scenario Builder', 'Position', [100 100 480 560]);

            y = 520;
            rowH = 30;

            uilabel(obj.Fig, 'Text', 'Munition', 'Position', [20 y 120 22]);
            catalog = munitionCatalog();
            keys = fieldnames(catalog);
            labels = cellfun(@(k) catalog.(k).Label, keys, 'UniformOutput', false);
            obj.MunitionDropdown = uidropdown(obj.Fig, 'Position', [150 y 300 24], ...
                'Items', labels, 'ItemsData', keys);
            y = y - rowH;

            uilabel(obj.Fig, 'Text', 'Firing Mode', 'Position', [20 y 120 22]);
            obj.FiringModeDropdown = uidropdown(obj.Fig, 'Position', [150 y 300 24], ...
                'Items', {'Direct', 'Indirect'}, 'Value', 'Direct', ...
                'ValueChangedFcn', @(src, event) obj.onFiringModeChanged());
            y = y - rowH - 10;

            uilabel(obj.Fig, 'Text', 'Launcher Position (m)', 'Position', [20 y 200 22], 'FontWeight', 'bold');
            y = y - rowH;
            uilabel(obj.Fig, 'Text', 'Downrange X', 'Position', [40 y 120 22]);
            obj.LauncherXField = uieditfield(obj.Fig, 'numeric', 'Position', [170 y 100 24], 'Value', 0);
            y = y - rowH;
            uilabel(obj.Fig, 'Text', 'Crossrange Y', 'Position', [40 y 120 22]);
            obj.LauncherYField = uieditfield(obj.Fig, 'numeric', 'Position', [170 y 100 24], 'Value', 0);
            y = y - rowH;
            uilabel(obj.Fig, 'Text', 'Altitude Z', 'Position', [40 y 120 22]);
            obj.LauncherZField = uieditfield(obj.Fig, 'numeric', 'Position', [170 y 100 24], 'Value', 0);
            y = y - rowH - 10;

            panelY = y - 110;

            % Direct-mode controls
            obj.DirectPanel = uipanel(obj.Fig, 'Position', [10 panelY 460 110], ...
                'Title', 'Direct Fire Parameters');
            uilabel(obj.DirectPanel, 'Text', 'Elevation (mils)', 'Position', [20 55 120 22]);
            obj.ElevationField = uieditfield(obj.DirectPanel, 'numeric', 'Position', [160 55 100 24], 'Value', 800);
            uilabel(obj.DirectPanel, 'Text', 'Azimuth (mils)', 'Position', [20 20 120 22]);
            obj.AzimuthField = uieditfield(obj.DirectPanel, 'numeric', 'Position', [160 20 100 24], 'Value', 0);

            % Indirect-mode controls (same footprint, toggled by Visible)
            obj.IndirectPanel = uipanel(obj.Fig, 'Position', [10 panelY 460 110], ...
                'Title', 'Indirect Fire Parameters', 'Visible', 'off');
            uilabel(obj.IndirectPanel, 'Text', 'Target Downrange X', 'Position', [20 70 130 22]);
            obj.TargetXField = uieditfield(obj.IndirectPanel, 'numeric', 'Position', [170 70 100 24], 'Value', 7500);
            uilabel(obj.IndirectPanel, 'Text', 'Target Crossrange Y', 'Position', [20 40 130 22]);
            obj.TargetYField = uieditfield(obj.IndirectPanel, 'numeric', 'Position', [170 40 100 24], 'Value', 0);
            uilabel(obj.IndirectPanel, 'Text', 'Target Altitude Z', 'Position', [20 10 130 22]);
            obj.TargetZField = uieditfield(obj.IndirectPanel, 'numeric', 'Position', [170 10 100 24], 'Value', 0);
            uilabel(obj.IndirectPanel, 'Text', 'Branch', 'Position', [300 55 60 22]);
            obj.BranchDropdown = uidropdown(obj.IndirectPanel, 'Position', [300 25 130 24], ...
                'Items', {'Low', 'High'}, 'Value', 'Low');

            btnY = panelY - 45;
            obj.RunButton = uibutton(obj.Fig, 'Text', 'Run', 'Position', [20 btnY 100 32], ...
                'ButtonPushedFcn', @(src, event) obj.onRunPushed());

            statusY = 20;
            statusH = btnY - statusY - 10;
            obj.StatusArea = uitextarea(obj.Fig, 'Position', [20 statusY 440 statusH], ...
                'Editable', 'off', 'Value', {'Ready.'});
        end

        function onFiringModeChanged(obj)
            if strcmp(obj.FiringModeDropdown.Value, 'Direct')
                obj.DirectPanel.Visible = 'on';
                obj.IndirectPanel.Visible = 'off';
            else
                obj.DirectPanel.Visible = 'off';
                obj.IndirectPanel.Visible = 'on';
            end
        end

        function log(obj, msg)
            obj.StatusArea.Value = [obj.StatusArea.Value; {msg}];
            scroll(obj.StatusArea, 'bottom');
            drawnow;
        end

        function onRunPushed(obj)
            params.MunitionKey = obj.MunitionDropdown.Value;
            params.FiringMode  = obj.FiringModeDropdown.Value;
            params.LauncherPos = [obj.LauncherXField.Value; obj.LauncherYField.Value; obj.LauncherZField.Value];

            if strcmp(params.FiringMode, 'Direct')
                params.ElevationMil = obj.ElevationField.Value;
                params.AzimuthMil   = obj.AzimuthField.Value;
            else
                params.TargetPos = [obj.TargetXField.Value; obj.TargetYField.Value; obj.TargetZField.Value];
                params.Branch    = obj.BranchDropdown.Value;
            end

            try
                filepath = generateScenarioRunner(params);
                obj.log(sprintf('Generated: %s', filepath));
            catch err
                obj.log(sprintf('FAILED to generate script: %s', err.message));
                return;
            end

            obj.log('Running...');
            try
                % Run in the BASE workspace, not this method's own - the
                % generated script starts with `clear` (correct, matches
                % every hand-written runner), and run() executes a script
                % in the CALLER's workspace when called from a function/
                % method. Without this, `clear` wipes out this method's
                % own local `obj` variable (the app handle itself),
                % breaking every obj.log(...) call after it - including
                % the one in this catch block.
                escaped_path = strrep(filepath, '''', '''''');
                evalin('base', ['run(''' escaped_path ''')']);
                obj.log('Finished. See the Command Window / figure for full output.');
            catch err
                obj.log(sprintf('Simulation error: %s', err.message));
            end
        end
    end
end
