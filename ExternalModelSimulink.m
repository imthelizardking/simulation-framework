%% File: ExternalModelSimulink.m
classdef ExternalModelSimulink < BaseExternalModel
    % Drives a Simulink model whose tunable parameters live in a linked
    % .sldd data dictionary (Simulink loads that automatically from the
    % model's own configuration - this class doesn't read or write the
    % dictionary itself, only checks it's actually there) and whose
    % outputs are captured by one or more "To Workspace" blocks inside
    % the model. Per-run inputs are applied as Simulink.SimulationInput
    % variable overrides (prepareInputs) rather than written into the
    % dictionary file, so repeated/concurrent runs don't mutate shared
    % on-disk state or step on each other.
    %
    % Requires the model's Data Import/Export configuration to have "Save
    % simulation output as single object" enabled - that's what makes
    % each To Workspace variable retrievable off the returned
    % Simulink.SimulationOutput by name (parseOutputs) instead of landing
    % in the base workspace, which would have the same cross-run
    % interference problem the SimulationInput approach above avoids.
    properties
        ModelName    (1,:) char = ''  % Simulink model name (must be on the MATLAB path)
        SlddPath     (1,:) char = ''  % .sldd data dictionary this model is linked to
        OutputNames  cell       = {}  % "To Workspace" variable names to pull out of the result
    end

    methods
        function obj = ExternalModelSimulink(model_name, sldd_path, output_names)
            if nargin > 0, obj.ModelName   = model_name; end
            if nargin > 1, obj.SlddPath    = sldd_path; end
            if nargin > 2, obj.OutputNames = output_names; end
        end

        function simIn = prepareInputs(obj, inputs)
            % inputs: struct of workspace-variable overrides for this
            % run (e.g. struct('V0', 850, 'Elevation', 0.62)), layered on
            % top of whatever obj.SlddPath's dictionary already provides.
            if ~isempty(obj.SlddPath) && ~isfile(obj.SlddPath)
                error('ExternalModelSimulink:SlddNotFound', ...
                    'Data dictionary not found: %s', obj.SlddPath);
            end
            simIn = Simulink.SimulationInput(obj.ModelName);
            if nargin > 1 && isstruct(inputs)
                fn = fieldnames(inputs);
                for i = 1:numel(fn)
                    simIn = simIn.setVariable(fn{i}, inputs.(fn{i}));
                end
            end
        end

        function simOut = execute(~, simIn)
            simOut = sim(simIn);
        end

        function outputs = parseOutputs(obj, simOut)
            outputs = struct();
            for i = 1:numel(obj.OutputNames)
                name = obj.OutputNames{i};
                try
                    outputs.(name) = simOut.get(name);
                catch err
                    error('ExternalModelSimulink:OutputNotFound', ...
                        ['Simulink output "%s" not found (%s) - check the To ' ...
                         'Workspace block''s variable name and that "Save ' ...
                         'simulation output as single object" is enabled in ' ...
                         'the model''s configuration.'], name, err.message);
                end
            end
        end
    end
end
