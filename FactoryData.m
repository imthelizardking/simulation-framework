classdef FactoryData
    % Optional data-file override for *Factory classes (MunitionFactory,
    % WeaponPlatformFactory, ChargeFactory): if data/<name>.json exists
    % alongside this file, its fields are used via getOr(); otherwise the
    % calling factory method keeps its own hardcoded defaults. Lets real
    % munition/weapon/propellant parameters live in an editable, diffable
    % JSON file instead of only inside a classdef method body - without
    % requiring one to exist for every preset.
    methods (Static)
        function data = load(name)
            here = fileparts(mfilename('fullpath'));
            json_path = fullfile(here, 'data', [name '.json']);
            if isfile(json_path)
                data = jsondecode(fileread(json_path));
            else
                data = [];
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
end
