%% File: ExternalModelExe.m
classdef ExternalModelExe < BaseExternalModel
    % Drives a third-party .exe (or any external command-line tool) by
    % shelling out - MATLAB's system() call, "bash commands" as opposed
    % to ExternalModelSimulink's sim() call.
    %
    % The exact input/output shape is completely tool-specific (a
    % command-line argument list vs. an input file it reads; stdout text
    % vs. an output file it writes) and not yet known, so every method
    % below is an intentional stub - fill in prepareInputs/execute/
    % parseOutputs once a real target tool's exact CLI/IO contract is
    % available. Guessing that shape now, without a real sample to check
    % it against, is exactly the mistake this project already made once
    % (see ParserDatcom's history - it had to be rebuilt once a genuine
    % sample file arrived) - not repeating it here.
    %
    % inputs (into run(), see Simulation.runExternalModel) is
    % struct('InitialState',X0,'TimeSpan',[t0 t1],'Dt',dt,
    % 'Environment',env,'Target',target_bus); outputs must come back as
    % struct('time',...,'pos',...,'vel',...) in the same shape
    % Simulation's own telemetry uses - that's the one part of the
    % contract fixed on this framework's side, independent of whatever
    % the .exe itself looks like.
    properties
        ExePath (1,:) char = '' % path to the executable
    end

    methods
        function obj = ExternalModelExe(exe_path)
            if nargin > 0, obj.ExePath = exe_path; end
        end

        function prepared = prepareInputs(obj, inputs) %#ok<INUSD>
            % TODO: translate `inputs` into whatever this .exe expects -
            % a command-line argument list, an input file, or both.
            error('ExternalModelExe:NotImplemented', ...
                '%s.prepareInputs is a stub - fill it in once this .exe''s input format is known.', class(obj));
        end

        function raw = execute(obj, prepared) %#ok<INUSD>
            % TODO: e.g. [status, cmdout] = system(sprintf('"%s" %s', obj.ExePath, prepared.Args));
            error('ExternalModelExe:NotImplemented', ...
                '%s.execute is a stub - fill it in once this .exe''s calling convention is known.', class(obj));
        end

        function outputs = parseOutputs(obj, raw) %#ok<INUSD>
            % TODO: parse raw (stdout text and/or an output file this
            % .exe wrote) into outputs.time/.pos/.vel - see this class'
            % header comment for the exact contract.
            error('ExternalModelExe:NotImplemented', ...
                '%s.parseOutputs is a stub - fill it in once this .exe''s output format is known.', class(obj));
        end
    end
end
