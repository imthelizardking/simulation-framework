classdef FactoryData
    % Optional data-file override for *Factory classes (WeaponPlatformFactory,
    % MunitionFactory, ChargeFactory): if data/<name>.mat exists alongside
    % this file, its fields are used via getOr()/getOrAny()/getOrPath()/
    % findAll(); otherwise the calling factory method keeps its own
    % hardcoded defaults. Lets real munition/weapon/propellant parameters
    % live in an editable, diffable data file instead of only inside a
    % classdef method body - without requiring one to exist for every
    % preset.
    %
    % .mat only - this framework standardized on one file format (see
    % MunitionData for the fixed per-munition schema it layers on top of
    % this for aero/mass/inertia specifically). Earlier versions of this
    % class also read .json/.xml/.fci/.xlsx/.txt/.sldd; that multi-format
    % support (and the accompanying text-table/FCI-segment parsing) is
    % removed, not just unused - do not resurrect it for a one-off file
    % instead of converting that file to .mat.
    methods (Static)
        function data = load(name)
            here = fileparts(mfilename('fullpath'));
            path = fullfile(here, 'data', [name '.mat']);
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

        function val = getOr(data, field, default)
            if isstruct(data) && isfield(data, field)
                val = data.(field);
            else
                val = default;
            end
        end

        function val = getOrAny(data, candidates, default)
            % Like getOr, but tries several candidate field-name
            % spellings and matches case- and underscore-insensitively -
            % for a field whose name varies across source files/tools
            % (e.g. 'IForm', 'IFORM', 'I_Form', 'i_form' should all match
            % the same value). candidates is a cellstr, tried in order;
            % the first one present in data (after normalizing both sides)
            % wins. Falls back to default if data isn't a struct or none
            % of the candidates match, same as getOr.
            val = default;
            if ~isstruct(data)
                return;
            end
            fn = fieldnames(data);
            normFn = cellfun(@FactoryData.normalizeFieldName, fn, 'UniformOutput', false);
            for i = 1:numel(candidates)
                idx = find(strcmp(normFn, FactoryData.normalizeFieldName(candidates{i})), 1);
                if ~isempty(idx)
                    val = data.(fn{idx});
                    return;
                end
            end
        end

        function val = getOrPath(data, path, default)
            % Like getOr, but walks an arbitrary chain of nested field
            % names instead of just one - for a source file whose fields
            % of interest sit inside nested groups (e.g. data.Group.Field
            % instead of a flat data.Field) rather than FactoryData's
            % usual flat top-level convention. path is a cellstr, e.g.
            % {'ChargePerformance','ReferenceV0'} for data.ChargePerformance.ReferenceV0.
            % Falls back to default at the first missing/non-struct hop,
            % same "no file/field -> just use the hardcoded default"
            % behavior as getOr - never errors on a missing path.
            val = data;
            for i = 1:numel(path)
                if isstruct(val) && isfield(val, path{i})
                    val = val.(path{i});
                else
                    val = default;
                    return;
                end
            end
        end

        function matches = findAll(data, field)
            % Recursively searches data - including nested structs and
            % struct arrays - for EVERY field named `field`, wherever it
            % sits in the tree. Unlike getOr/getOrAny (top-level only) or
            % getOrPath (needs the exact path already known), this finds
            % a field that might be nested under an arbitrary/unknown
            % subfield - e.g. data.CD0 AND data.Aero.CD0 would both be
            % found by findAll(data, 'CD0'). Matched case- and
            % underscore-insensitively, same rule as getOrAny.
            %
            % Returns every match (there may legitimately be zero, one,
            % or several - a field name is not assumed unique across the
            % file) as a struct array with one entry per match: .Path (a
            % cell array locating it - char field names, or a numeric
            % index where the path passed through a struct array) and
            % .Value. Check isempty(matches) rather than passing a
            % fallback default like getOr does, since "found nowhere"
            % and "found with this value" aren't the only two outcomes
            % here - "found in several places" also needs handling by
            % the caller.
            matches = struct('Path', {}, 'Value', {});
            matches = FactoryData.findAllRecurse(data, field, {}, matches);
        end
    end

    methods (Static, Access = private)
        function matches = findAllRecurse(node, field, path, matches)
            if isstruct(node)
                if ~isscalar(node)
                    for k = 1:numel(node)
                        matches = FactoryData.findAllRecurse(node(k), field, [path, {k}], matches);
                    end
                    return;
                end
                fn = fieldnames(node);
                for i = 1:numel(fn)
                    childPath = [path, {fn{i}}];
                    if strcmp(FactoryData.normalizeFieldName(fn{i}), FactoryData.normalizeFieldName(field))
                        matches(end + 1) = struct('Path', {childPath}, 'Value', {node.(fn{i})}); %#ok<AGROW>
                    end
                    matches = FactoryData.findAllRecurse(node.(fn{i}), field, childPath, matches);
                end
            elseif iscell(node)
                for k = 1:numel(node)
                    matches = FactoryData.findAllRecurse(node{k}, field, [path, {k}], matches);
                end
            end
        end

        function n = normalizeFieldName(name)
            % Case- and underscore-insensitive comparison key for
            % getOrAny - 'IForm', 'IFORM', 'I_Form', 'i_form' all reduce
            % to the same string.
            n = lower(strrep(char(name), '_', ''));
        end
    end
end
