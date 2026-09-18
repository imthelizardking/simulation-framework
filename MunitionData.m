classdef MunitionData
    % Reads the standardized per-munition .mat data file - one file per
    % munition, named EXACTLY after its Munition.Name (e.g. Name =
    % '25mm_M793' -> data/25mm_M793.mat) - holding that munition's aero
    % coefficient tables and mass/inertia properties. This supersedes the
    % old per-MunitionFactory-preset-method JSON/XML override files
    % (FactoryData.load('createXXX')) for these two fields specifically;
    % FactoryData itself is still used for everything else a preset
    % reads (Name, Type, RefArea/RefLength, TwistRate, ...) and is
    % untouched by this class.
    %
    % Fixed layout (produced independently, outside this framework, and
    % just read here - not FactoryData's flexible multi-format/schema
    % convention, since this one is standardized going forward):
    %
    %   <Name>.aero.phase.<Coeff> - an Nx(1+K) numeric matrix. Column 1
    %       is the Mach breakpoint; columns 2..end are a0, a1, a2, ... -
    %       each row's LOCAL polynomial-in-Mach (coeff = a0 + a1*mach +
    %       a2*mach^2 + ...), valid from that row's Mach breakpoint up to
    %       the next row's (same "Lower"-boundary per-segment semantics
    %       as BaseAero.evalCoeff's PiecewisePolynomial case - see
    %       toCoeff below, which converts into exactly that struct shape
    %       so evalCoeff needs no changes to consume it). No explicit
    %       Type tag is stored on disk - every coefficient is this one
    %       shape now, or a bare scalar for a Mach-invariant constant.
    %   <Name>.inertial.Mass    - scalar, kg.
    %   <Name>.inertial.Inertia - the FULL inertia matrix, not just a
    %       diagonal vector: 3x3 for an ordinary rigid body, or 4x4 for a
    %       dual-spin body (e.g. a despun payload section). Passed
    %       through to MassProperties.Inertia as-is; nothing in this
    %       framework's Dynamics classes yet consumes anything past
    %       Inertia(1,1) (the axial/spin term), so a 4x4 matrix carries
    %       through the data pipeline correctly even though full
    %       dual-spin dynamics aren't wired up yet.
    methods (Static)
        function data = load(munition_name)
            % [] (not an error) if data/<munition_name>.mat doesn't
            % exist - every accessor below then falls back to its own
            % `default` argument, same "optional override" contract
            % FactoryData.load/getOr already has.
            here = fileparts(mfilename('fullpath'));
            path = fullfile(here, 'data', [munition_name '.mat']);
            if ~isfile(path)
                data = [];
                return;
            end
            raw = load(path);
            fn = fieldnames(raw);
            if isscalar(fn) && isstruct(raw.(fn{1}))
                data = raw.(fn{1});
            else
                data = raw;
            end
        end

        function coeffs = aeroCoeffs(data)
            % Every field under data.aero.phase, converted to a
            % BaseAero.evalCoeff-compatible value (see toCoeff) and
            % collected into one struct - so a caller can pull a specific
            % coefficient out with FactoryData.getOr(MunitionData.
            % aeroCoeffs(data), 'CD0', <hardcoded default>), reusing
            % getOr's existing per-field fallback rather than a new one.
            % Returns struct() (no fields - getOr's isfield check then
            % just falls through to its own default) if data has no
            % aero.phase section.
            coeffs = struct();
            if ~isstruct(data) || ~isfield(data, 'aero') || ~isfield(data.aero, 'phase')
                return;
            end
            phase = data.aero.phase;
            fn = fieldnames(phase);
            for i = 1:numel(fn)
                coeffs.(fn{i}) = MunitionData.toCoeff(phase.(fn{i}));
            end
        end

        function mass = mass(data, default)
            if isstruct(data) && isfield(data, 'inertial') && isfield(data.inertial, 'Mass')
                mass = data.inertial.Mass;
            else
                mass = default;
            end
        end

        function inertia = inertia(data, default)
            if isstruct(data) && isfield(data, 'inertial') && isfield(data.inertial, 'Inertia')
                inertia = data.inertial.Inertia;
            else
                inertia = default;
            end
        end
    end

    methods (Static, Access = private)
        function coeff = toCoeff(raw)
            if isscalar(raw)
                coeff = raw; % Mach-invariant constant
                return;
            end
            coeff = struct('Type', 'PiecewisePolynomial', ...
                'Mach',   raw(:, 1).', ...
                'Coeffs', raw(:, 2:end), ...
                'Order',  0:(size(raw, 2) - 2));
        end
    end
end
