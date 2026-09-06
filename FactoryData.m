classdef FactoryData
    % Optional data-file override for *Factory classes (WeaponPlatformFactory,
    % MunitionFactory, ChargeFactory): if data/<name>.<ext> exists alongside
    % this file, its fields are used via getOr(); otherwise the calling
    % factory method keeps its own hardcoded defaults. Lets real
    % munition/weapon/propellant parameters live in an editable, diffable
    % data file instead of only inside a classdef method body - without
    % requiring one to exist for every preset.
    %
    % Supported file formats (tried in this order, first match wins):
    % .json, .xml, .fci, .xlsx, .mat, .txt, .sldd. All but .sldd only need
    % base MATLAB; .sldd needs a Simulink license (Simulink.data.dictionary).
    % .fci is the NATO Fire Control Interface XML schema (STANAG/AC-326
    % ballistic data exchange format) - it is parsed exactly like .xml,
    % see the segmented-coefficient note below.
    %
    % Any field's value can be a plain scalar/array (a JSON number/array, an
    % XML element with plain text, an .xlsx cell, a .mat variable, ...) or a
    % Mach-dependent text table: a header line naming its columns - one of
    % which must be "Mach" (case-insensitive) - followed by whitespace- or
    % comma-separated numeric rows, e.g.:
    %   "Mach CD0 CDA2\n0 0.19 2.2\n0.7 0.20 2.1\n1.0 0.38 1.9"
    % Each non-Mach column becomes a top-level field, named after that
    % column's header, holding a BaseAero.evalCoeff-compatible
    % struct('Type','Table','Mach',...,'Value',...). The key/element/cell
    % that originally held the table text does not matter - only the
    % header row's column names do - so one table can populate several
    % coefficients (e.g. CD0 and CDA2) at once, and a repeated "Mach"
    % column lets later columns use a different Mach breakpoints than
    % earlier ones (e.g. "Mach CD0 Mach CLA").
    %
    % If instead every non-Mach column is named "a0"/"a1"/... (e.g.
    % "Mach a0 a1\n0.4 1.7 0.1\n0.5 1.8 0.2"), the whole table is treated
    % as ONE coefficient - a McCoy/PRODAS-style Mach-regime table where
    % each row's a0/a1/... is a local polynomial-in-Mach valid from that
    % row's Mach breakpoint to the next - and kept under the field's own
    % original key (there are no descriptive column names to use instead)
    % as a struct('Type','PiecewisePolynomial','Mach',...,'Coeffs',...,'Order',...).
    %
    % .xml/.fci files additionally recognize that same per-breakpoint
    % polynomial shape written the Fire-Control-Interface way: an element
    % whose only children are <Segment Maximum_Mach="..."> entries, each
    % holding one or more <Coeff Term="a0">value</Coeff> children, e.g.
    %   <CD0>
    %     <Segment Maximum_Mach="0.83"><Coeff Term="a0">0.124</Coeff></Segment>
    %     <Segment Maximum_Mach="0.9"><Coeff Term="a0">0.029</Coeff></Segment>
    %   </CD0>
    % becomes the same PiecewisePolynomial struct as above, with one
    % difference: FCI's Maximum_Mach is each segment's UPPER bound (the
    % segment applies up to and including that Mach), the opposite of the
    % lower-bound "Mach" breakpoints in the text-table convention, so
    % this struct also carries 'Boundary','Upper' and
    % BaseAero.evalCoeff picks the segment accordingly. Any other XML
    % element's attributes are folded into its struct as ordinary fields
    % (e.g. <Projectile Model="..." Name="..."/> becomes a struct with
    % Model/Name fields); repeated sibling tags (multiple <Projectile>
    % under <Projectiles>) become a cell array in encounter order.
    methods (Static)
        function data = load(name)
            here = fileparts(mfilename('fullpath'));
            exts = {'.json', '.xml', '.fci', '.xlsx', '.mat', '.txt', '.sldd'};
            data = [];
            for i = 1:numel(exts)
                p = fullfile(here, 'data', [name exts{i}]);
                if isfile(p)
                    data = FactoryData.loadFile(p, exts{i});
                    break;
                end
            end
            if isstruct(data)
                data = FactoryData.expandTables(data);
            end
        end

        function val = getOr(data, field, default)
            if isstruct(data) && isfield(data, field)
                val = data.(field);
            else
                val = default;
            end
        end
    end

    methods (Static, Access = private)
        function data = loadFile(path, ext)
            switch ext
                case '.json'
                    data = jsondecode(fileread(path));
                case {'.xml', '.fci'}
                    data = FactoryData.loadXml(path);
                case '.xlsx'
                    data = FactoryData.loadXlsx(path);
                case '.mat'
                    data = FactoryData.loadMat(path);
                case '.txt'
                    data = FactoryData.loadTxt(path);
                case '.sldd'
                    data = FactoryData.loadSldd(path);
                otherwise
                    error('FactoryData:load', 'Unsupported data file extension: %s', ext);
            end
        end

        function data = expandTables(data)
            % Replaces any char/string field whose text parses as a
            % Mach-header table. A "Mach CD0 CLA"-style header (arbitrary
            % names) becomes one field per non-Mach column, named after
            % its header - the original field's own key is discarded. A
            % "Mach a0 a1 ..."-style header instead describes ONE
            % per-breakpoint polynomial coefficient (see
            % BaseAero.evalCoeff's PiecewisePolynomial case), so the
            % result is kept under the ORIGINAL field key since a0/a1/...
            % are polynomial term indices, not names.
            fn = fieldnames(data);
            for i = 1:numel(fn)
                v = data.(fn{i});
                if isstring(v) && isscalar(v)
                    v = char(v);
                end
                if ~ischar(v)
                    continue;
                end
                [cols, single] = FactoryData.parseTableText(v);
                if isempty(cols) && isempty(single)
                    continue;
                end
                data = rmfield(data, fn{i});
                if ~isempty(single)
                    data.(fn{i}) = single;
                else
                    cn = fieldnames(cols);
                    for k = 1:numel(cn)
                        data.(cn{k}) = cols.(cn{k});
                    end
                end
            end
        end

        function [cols, single] = parseTableText(str)
            % Parses str as "header line" + numeric rows. Returns [] for
            % both outputs (not an error) when str simply isn't
            % table-shaped, e.g. a plain Name string or a single
            % number/word - only a string that does have a header row
            % containing "Mach" is treated as a table and held to strict
            % row-shape validation.
            cols = [];
            single = [];
            lines = regexp(str, '\r\n|\r|\n', 'split');
            lines = strtrim(lines);
            lines = lines(~cellfun(@isempty, lines));
            if numel(lines) < 2
                return;
            end
            headers = regexp(lines{1}, '\s+', 'split');
            headers = headers(~cellfun(@isempty, headers));
            if ~any(strcmpi(headers, 'mach'))
                return;
            end
            nCols = numel(headers);
            rows = zeros(numel(lines) - 1, nCols);
            for r = 2:numel(lines)
                toks = regexp(strrep(lines{r}, ',', ' '), '\s+', 'split');
                toks = toks(~cellfun(@isempty, toks));
                vals = str2double(toks);
                if numel(vals) ~= nCols || any(isnan(vals))
                    error('FactoryData:parseTable', ...
                        'Table row "%s" must have %d numeric values matching header "%s"', ...
                        lines{r}, nCols, lines{1});
                end
                rows(r - 1, :) = vals;
            end
            [cols, single] = FactoryData.parseTableFromRows(headers, rows);
        end

        function [cols, single] = parseTableFromRows(headers, rows)
            % headers: cellstr of column names. rows: numeric matrix,
            % columns(headers) x rows(data points). Returns either:
            %   cols   - a struct with one Table-typed field per
            %            arbitrarily-named non-Mach column (each paired
            %            with the nearest preceding "Mach" column, so a
            %            repeated "Mach" header lets later columns use
            %            different breakpoints), or
            %   single - one PiecewisePolynomial-typed struct, when every
            %            non-Mach header is "a0"/"a1"/... (a single
            %            coefficient's per-breakpoint polynomial, not
            %            several independent named coefficients).
            % Exactly one of the two is non-empty on a successful parse.
            cols = [];
            single = [];
            machIdxAll = find(strcmpi(headers, 'mach'));
            valueIdx = setdiff(1:numel(headers), machIdxAll, 'stable');
            valueIdx = valueIdx(~cellfun(@isempty, headers(valueIdx)));
            if isempty(valueIdx)
                return;
            end
            orders = nan(1, numel(valueIdx));
            for k = 1:numel(valueIdx)
                tok = regexpi(headers{valueIdx(k)}, '^a(\d+)$', 'tokens', 'once');
                if ~isempty(tok)
                    orders(k) = str2double(tok{1});
                end
            end
            if all(~isnan(orders))
                if numel(machIdxAll) ~= 1
                    error('FactoryData:parseTable', ...
                        'A per-breakpoint polynomial table ("Mach a0 a1 ...") must have exactly one "Mach" column');
                end
                [orders, order] = sort(orders);
                single = struct('Type', 'PiecewisePolynomial', ...
                    'Mach', rows(:, machIdxAll).', ...
                    'Coeffs', rows(:, valueIdx(order)), ...
                    'Order', orders);
                return;
            end
            if any(~isnan(orders))
                error('FactoryData:parseTable', ...
                    'Cannot mix "a0"/"a1"/... polynomial columns with named coefficient columns in one table');
            end
            % Named-coefficient shape: pair each value column with the
            % nearest preceding Mach column.
            cols = struct();
            machCol = [];
            for j = 1:numel(headers)
                if isempty(headers{j})
                    continue;
                elseif strcmpi(headers{j}, 'mach')
                    machCol = j;
                else
                    if isempty(machCol)
                        error('FactoryData:parseTable', ...
                            'Column "%s" has no preceding "Mach" column to pair with', headers{j});
                    end
                    cols.(headers{j}) = struct('Type', 'Table', ...
                        'Mach', rows(:, machCol).', 'Value', rows(:, j).');
                end
            end
            if isempty(fieldnames(cols))
                cols = [];
            end
        end

        function val = parseScalarOrVector(txt)
            % Best-effort match to JSON's native number/array typing for
            % formats (XML, txt, xlsx) whose cells only carry text: a
            % single-line, purely-numeric, whitespace/comma-separated
            % piece of text becomes a double scalar/row vector; anything
            % else (a name, a unit suffix, ...) stays a plain string.
            txt = strtrim(txt);
            if isempty(txt)
                val = '';
                return;
            end
            toks = regexp(strrep(txt, ',', ' '), '\s+', 'split');
            toks = toks(~cellfun(@isempty, toks));
            nums = str2double(toks);
            if ~isempty(nums) && ~any(isnan(nums))
                val = nums;
            else
                val = txt;
            end
        end

        function data = loadXml(path)
            doc = xmlread(path);
            data = FactoryData.xmlElementToStruct(doc.getDocumentElement());
        end

        function s = xmlElementToStruct(elem)
            % Builds a struct from elem's attributes and element children.
            % A repeated child tag becomes a cell array (in document
            % order) instead of silently keeping only the last one.
            s = struct();
            names = {};
            values = {};
            kids = elem.getChildNodes();
            for i = 0:kids.getLength() - 1
                child = kids.item(i);
                if child.getNodeType() ~= 1 % not an ELEMENT_NODE
                    continue;
                end
                names{end + 1} = char(child.getNodeName()); %#ok<AGROW>
                values{end + 1} = FactoryData.xmlChildValue(child); %#ok<AGROW>
            end
            uniqueNames = unique(names, 'stable');
            for u = 1:numel(uniqueNames)
                idx = find(strcmp(names, uniqueNames{u}));
                if isscalar(idx)
                    s.(uniqueNames{u}) = values{idx};
                else
                    s.(uniqueNames{u}) = values(idx);
                end
            end
            attrs = FactoryData.xmlAttrsToStruct(elem);
            attrNames = fieldnames(attrs);
            for k = 1:numel(attrNames)
                if ~isfield(s, attrNames{k})
                    s.(attrNames{k}) = attrs.(attrNames{k});
                end
            end
        end

        function val = xmlChildValue(elem)
            % A Fire-Control-Interface-style segmented Mach table (see
            % tryParseSegmentedCoeff) is recognized before the generic
            % struct/attribute handling below would otherwise turn its
            % repeated <Segment> children into an awkward cell array.
            poly = FactoryData.tryParseSegmentedCoeff(elem);
            if ~isempty(poly)
                val = poly;
                return;
            end
            hasElementChild = false;
            kids = elem.getChildNodes();
            for k = 0:kids.getLength() - 1
                if kids.item(k).getNodeType() == 1
                    hasElementChild = true;
                    break;
                end
            end
            if hasElementChild
                val = FactoryData.xmlElementToStruct(elem);
                return;
            end
            attrs = FactoryData.xmlAttrsToStruct(elem);
            txt = char(elem.getTextContent());
            if contains(txt, sprintf('\n')) || contains(txt, sprintf('\r'))
                textVal = txt;
            else
                textVal = FactoryData.parseScalarOrVector(txt);
            end
            if isempty(fieldnames(attrs))
                val = textVal; % plain leaf, e.g. <Mass>43.2</Mass>
            elseif isempty(strtrim(txt))
                val = attrs; % attributes-only leaf, e.g. <Projectile Model="..."/>
            else
                val = attrs;
                val.Value = textVal; % both, e.g. <Coeff Term="a0">0.124</Coeff>
            end
        end

        function attrs = xmlAttrsToStruct(elem)
            attrs = struct();
            if ~elem.hasAttributes()
                return;
            end
            attrMap = elem.getAttributes();
            for i = 0:attrMap.getLength() - 1
                a = attrMap.item(i);
                attrs.(char(a.getNodeName())) = FactoryData.parseScalarOrVector(char(a.getNodeValue()));
            end
        end

        function poly = tryParseSegmentedCoeff(elem)
            % Recognizes elem as a Fire-Control-Interface-style
            % Mach-regime table: every element child is a <Segment
            % Maximum_Mach="..."> holding one or more <Coeff
            % Term="a0">value</Coeff> children. Returns [] (not an error)
            % for anything that doesn't match this exact shape, so the
            % generic struct/attribute path in xmlChildValue handles it
            % instead. Maximum_Mach is each segment's UPPER bound, so the
            % resulting struct carries 'Boundary','Upper' for
            % BaseAero.evalCoeff.
            poly = [];
            segElems = {};
            kids = elem.getChildNodes();
            for i = 0:kids.getLength() - 1
                c = kids.item(i);
                if c.getNodeType() ~= 1
                    continue;
                end
                if ~strcmp(char(c.getNodeName()), 'Segment') || ~c.hasAttribute('Maximum_Mach')
                    return;
                end
                segElems{end + 1} = c; %#ok<AGROW>
            end
            if isempty(segElems)
                return;
            end
            nSeg = numel(segElems);
            machVals = nan(1, nSeg);
            termNames = {};
            coeffMaps = cell(1, nSeg);
            for s = 1:nSeg
                machVals(s) = str2double(char(segElems{s}.getAttribute('Maximum_Mach')));
                local = struct();
                ck = segElems{s}.getChildNodes();
                for i = 0:ck.getLength() - 1
                    cc = ck.item(i);
                    if cc.getNodeType() ~= 1
                        continue;
                    end
                    if ~strcmp(char(cc.getNodeName()), 'Coeff') || ~cc.hasAttribute('Term')
                        return;
                    end
                    term = char(cc.getAttribute('Term'));
                    local.(term) = str2double(char(cc.getTextContent()));
                    if ~any(strcmp(termNames, term))
                        termNames{end + 1} = term; %#ok<AGROW>
                    end
                end
                coeffMaps{s} = local;
            end
            if any(isnan(machVals)) || isempty(termNames)
                return;
            end
            orders = nan(1, numel(termNames));
            for k = 1:numel(termNames)
                tok = regexpi(termNames{k}, '^a(\d+)$', 'tokens', 'once');
                if isempty(tok)
                    return;
                end
                orders(k) = str2double(tok{1});
            end
            [orders, ord] = sort(orders);
            termNames = termNames(ord);
            [machVals, msort] = sort(machVals);
            coeffs = zeros(nSeg, numel(termNames));
            for s = 1:nSeg
                local = coeffMaps{msort(s)};
                for k = 1:numel(termNames)
                    if isfield(local, termNames{k})
                        coeffs(s, k) = local.(termNames{k});
                    end
                end
            end
            poly = struct('Type', 'PiecewisePolynomial', 'Mach', machVals, ...
                'Coeffs', coeffs, 'Order', orders, 'Boundary', 'Upper');
        end

        function data = loadTxt(path)
            % "Key = value" (or "Key: value") lines; a value can continue
            % on following lines (e.g. a Mach table) until a blank line
            % or the next "Key =" line.
            rawLines = regexp(fileread(path), '\r\n|\r|\n', 'split');
            data = struct();
            curName = '';
            curLines = {};
            keyPattern = '^\s*([A-Za-z_]\w*)\s*[:=]\s*(.*)$';
            for i = 1:numel(rawLines)
                line = rawLines{i};
                m = regexp(line, keyPattern, 'tokens', 'once');
                if ~isempty(m)
                    data = FactoryData.flushTxtField(data, curName, curLines);
                    curName = m{1};
                    curLines = {m{2}};
                elseif isempty(strtrim(line))
                    data = FactoryData.flushTxtField(data, curName, curLines);
                    curName = '';
                    curLines = {};
                else
                    curLines{end + 1} = line; %#ok<AGROW>
                end
            end
            data = FactoryData.flushTxtField(data, curName, curLines);
        end

        function data = flushTxtField(data, name, valueLines)
            if isempty(name)
                return;
            end
            valueLines = valueLines(~cellfun(@(l) isempty(strtrim(l)), valueLines));
            if isempty(valueLines)
                data.(name) = '';
            elseif isscalar(valueLines)
                data.(name) = FactoryData.parseScalarOrVector(valueLines{1});
            else
                data.(name) = strjoin(valueLines, newline);
            end
        end

        function data = loadXlsx(path)
            raw = readcell(path);
            data = struct();
            if isempty(raw)
                return;
            end
            headerStr = cellfun(@FactoryData.cellToStr, raw(1, :), 'UniformOutput', false);
            if size(raw, 2) > 1 && any(strcmpi(headerStr, 'mach'))
                nRows = size(raw, 1) - 1;
                nCols = size(raw, 2);
                rows = nan(nRows, nCols);
                for r = 1:nRows
                    for c = 1:nCols
                        v = raw{r + 1, c};
                        if ischar(v) || isstring(v)
                            v = str2double(v);
                        end
                        if isnumeric(v) && isscalar(v)
                            rows(r, c) = v;
                        end
                    end
                end
                [cols, single] = FactoryData.parseTableFromRows(headerStr, rows);
                if ~isempty(single)
                    error('FactoryData:loadXlsx', ...
                        'A per-breakpoint polynomial sheet ("Mach a0 a1 ...") has no field name to attach to - use the two-column Field/Value convention instead so the coefficient has a name.');
                elseif ~isempty(cols)
                    data = cols;
                end
                return;
            end
            % Two-column Field/Value convention otherwise: row 1 is always
            % a header row (e.g. "Field","Value") and is skipped
            % unconditionally - unlike the Mach-table branch above, a data
            % row's own key is never compared against header text, so a
            % field legitimately named "Name" or "Field" still comes
            % through.
            if size(raw, 2) < 2
                return;
            end
            for r = 2:size(raw, 1)
                name = FactoryData.cellToStr(raw{r, 1});
                if isempty(name)
                    continue;
                end
                v = raw{r, 2};
                if ischar(v) || isstring(v)
                    v = char(v);
                    if contains(v, sprintf('\n')) || contains(v, sprintf('\r'))
                        data.(name) = v;
                    else
                        data.(name) = FactoryData.parseScalarOrVector(v);
                    end
                else
                    data.(name) = v;
                end
            end
        end

        function s = cellToStr(v)
            if ischar(v)
                s = v;
            elseif isstring(v)
                s = char(v);
            elseif isnumeric(v) && (isempty(v) || any(ismissing(v)))
                s = '';
            else
                s = num2str(v);
            end
        end

        function data = loadMat(path)
            raw = load(path);
            fn = fieldnames(raw);
            if isscalar(fn) && isstruct(raw.(fn{1}))
                data = raw.(fn{1});
            else
                data = raw;
            end
        end

        function data = loadSldd(path)
            % Requires a Simulink license - unlike every other format
            % here, .sldd is not readable with base MATLAB alone.
            try
                dd = Simulink.data.dictionary.open(path);
            catch err
                error('FactoryData:loadSldd', ...
                    'Reading .sldd files requires Simulink (failed to open "%s": %s)', ...
                    path, err.message);
            end
            cleanupObj = onCleanup(@() dd.close()); %#ok<NASGU>
            sec = getSection(dd, 'Design Data');
            entries = find(sec);
            data = struct();
            for i = 1:numel(entries)
                e = entries(i);
                v = getValue(e);
                if isa(v, 'Simulink.Parameter')
                    v = v.Value;
                end
                data.(e.Name) = v;
            end
        end
    end
end
