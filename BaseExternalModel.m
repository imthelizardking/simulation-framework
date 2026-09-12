%% File: BaseExternalModel.m
classdef (Abstract) BaseExternalModel < handle
    % Adapter contract for driving a model that lives outside this
    % framework's own in-process types (a Simulink model, a third-party
    % tool, a bare .exe) and therefore needs its own input/output format
    % on each side of the call. run() is a template method: convert this
    % framework's inputs into whatever the external model needs
    % (prepareInputs), invoke it (execute), then convert its raw output
    % back into this framework's own types (parseOutputs) - so a caller
    % (e.g. a Munition/Propulsion/Aero implementation delegating physics
    % out to an external tool) sees the same run(inputs) -> outputs shape
    % no matter which concrete external tool is behind it.
    methods (Abstract)
        prepared = prepareInputs(obj, inputs)
        raw      = execute(obj, prepared)
        outputs  = parseOutputs(obj, raw)
    end

    methods
        function outputs = run(obj, inputs)
            prepared = obj.prepareInputs(inputs);
            raw      = obj.execute(prepared);
            outputs  = obj.parseOutputs(raw);
        end
    end
end
