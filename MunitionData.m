classdef MunitionData
    % Reads the standardized per-munition .mat data file - one file per
    % munition, named EXACTLY after its Munition.Name (e.g. Name =
    % '25mm_M793' -> data/25mm_M793.mat) - holding that munition's aero
    % coefficient tables, mass/inertia properties, and charge-dependent
    % form factors. This supersedes the old per-MunitionFactory-preset-
    % method JSON/XML override files (FactoryData.load('createXXX')) for
    % these fields specifically; FactoryData itself is still used for
    % everything else a preset reads (Name, Type, RefArea/RefLength,
    % TwistRate, ...) and is untouched by this class.
    %
    % Schema confirmed against a real prototype file (data/mun_test.mat) -
    % NOT the earlier guessed layout this class started with (that had an
    % extra always-present "phase" level with no room for more than one,
    % and a single full inertia matrix). Fixed layout:
    %
    %   <Name>.aero.<PhaseName>.<Coeff> - PhaseName is 'phase' by default
    %       (see aeroCoeffs), but a munition can have more than one -
    %       'phase2', 'phase3', ... - for genuinely different aero
    %       regimes (e.g. boost vs. coast) sharing one file. Each
    %       <Coeff> is an Nx(1+K) numeric matrix: column 1 = Mach
    %       breakpoint, columns 2..end = a0, a1, a2, ... (a row's LOCAL
    %       polynomial-in-Mach, valid from that row's breakpoint onward -
    %       same "Lower"-boundary per-segment semantics as
    %       BaseAero.evalCoeff's PiecewisePolynomial case, see toCoeff),
    %       or a bare scalar for a Mach-invariant constant. Coefficient
    %       field names may differ from this framework's own vocabulary
    %       (e.g. CMASPIN/CMAGF instead of CSPIN/CMAG) - callers use
    %       FactoryData.getOrAny with both spellings as candidates, same
    %       as any other spelling variation (see that method).
    %   <Name>.inertial.mass - scalar, kg.
    %   <Name>.inertial.inertia_matrix - the FULL inertia matrix if
    %       given directly: 3x3 for an ordinary rigid body, or 4x4 for a
    %       dual-spin body (e.g. a despun payload section). Takes
    %       priority over the pair below when present.
    %   <Name>.inertial.inertial_axial / .inertial_transverse - scalar
    %       Ix and Iy(=Iz) instead of a full matrix, for the common
    %       axisymmetric case - combined into diag([axial, transverse,
    %       transverse]) when inertia_matrix isn't given.
    %   <Name>.iform / <Name>.fl - STANAG 4355 MPMM fitting factors (see
    %       AeroMPMM.IForm/FL). EITHER a bare scalar (constant for the
    %       whole munition, charge-independent) OR a struct keyed by
    %       charge-zone code (e.g. 'CH7W', matching ChargeZone.Name),
    %       each holding a [a0,a1,a2,...] polynomial-in-QE (quadrant
    %       elevation, MILS): value = a0 + a1*QE + a2*QE^2 + ... - see
    %       formFactor, which evaluates whichever shape is present.
    %   <Name>.propulsion - reserved, not yet given a fixed sub-schema
    %       (empty in the prototype file) - see propulsion() below.
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

        function coeffs = aeroCoeffs(data, phase_name)
            % Every field under data.aero.(phase_name), converted to a
            % BaseAero.evalCoeff-compatible value (see toCoeff) and
            % collected into one struct - so a caller can pull a specific
            % coefficient out with FactoryData.getOr(MunitionData.
            % aeroCoeffs(data), 'CD0', <hardcoded default>), reusing
            % getOr's existing per-field fallback rather than a new one.
            % phase_name defaults to 'phase' - pass 'phase2' etc. for a
            % munition file that defines more than one aero regime.
            % Returns struct() (no fields - getOr's isfield check then
            % just falls through to its own default) if data has no
            % aero.(phase_name) section.
            if nargin < 2 || isempty(phase_name), phase_name = 'phase'; end
            coeffs = struct();
            if ~isstruct(data) || ~isfield(data, 'aero') || ~isfield(data.aero, phase_name)
                return;
            end
            phase = data.aero.(phase_name);
            fn = fieldnames(phase);
            for i = 1:numel(fn)
                coeffs.(fn{i}) = MunitionData.toCoeff(phase.(fn{i}));
            end
        end

        function mass = mass(data, default)
            if isstruct(data) && isfield(data, 'inertial') && isfield(data.inertial, 'mass')
                mass = double(data.inertial.mass);
            else
                mass = default;
            end
        end

        function inertia = inertia(data, default)
            % Prefers a full inertia_matrix if given; falls back to
            % diag([axial, transverse, transverse]) built from the
            % scalar pair; falls back to `default` if neither is present.
            if isstruct(data) && isfield(data, 'inertial')
                inertial = data.inertial;
                if isfield(inertial, 'inertia_matrix') && ~isempty(inertial.inertia_matrix)
                    inertia = double(inertial.inertia_matrix);
                    return;
                end
                if isfield(inertial, 'inertial_axial') && isfield(inertial, 'inertial_transverse')
                    a = double(inertial.inertial_axial);
                    t = double(inertial.inertial_transverse);
                    inertia = diag([a, t, t]);
                    return;
                end
            end
            inertia = default;
        end

        function val = formFactor(data, field_name, zone_name, qe_mils, default)
            % Evaluates data.(field_name) (field_name is 'iform' or
            % 'fl'), which is EITHER a bare scalar (constant, charge-
            % independent - returned as-is, zone_name/qe_mils ignored)
            % OR a struct keyed by charge-zone code (see this class'
            % header comment), each holding a [a0,a1,a2,...]
            % polynomial-in-QE evaluated at qe_mils. Falls back to
            % `default` if data lacks the field, or (struct case) lacks
            % that specific zone.
            if ~isstruct(data) || ~isfield(data, field_name)
                val = default;
                return;
            end
            raw = data.(field_name);
            if ~isstruct(raw)
                val = double(raw); % single scalar for the whole munition
                return;
            end
            if ~isfield(raw, zone_name)
                val = default;
                return;
            end
            coeffs = double(raw.(zone_name));
            coeffs = coeffs(:);
            val = 0;
            for k = 1:numel(coeffs)
                val = val + coeffs(k) * qe_mils^(k - 1);
            end
        end

        function p = propulsion(data, default)
            % Reserved field, no fixed sub-schema yet (empty in the
            % prototype file this class' layout was confirmed against) -
            % passed through as whatever's there, or `default` if absent/
            % empty.
            if isstruct(data) && isfield(data, 'propulsion') && ~isempty(data.propulsion)
                p = data.propulsion;
            else
                p = default;
            end
        end
    end

    methods (Static, Access = private)
        function coeff = toCoeff(raw)
            if isscalar(raw)
                coeff = double(raw); % Mach-invariant constant
                return;
            end
            raw = double(raw);
            coeff = struct('Type', 'PiecewisePolynomial', ...
                'Mach',   raw(:, 1).', ...
                'Coeffs', raw(:, 2:end), ...
                'Order',  0:(size(raw, 2) - 2));
        end
    end
end
