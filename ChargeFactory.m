classdef ChargeFactory
    % Propellant/charge-zone presets. DM92 zone names and the 6M=943 m/s
    % point are as given; the other DM92 zones and all M119A2 zones are
    % illustrative (a plausible monotonic progression), not transcribed
    % from a real firing table - same caveat as MunitionFactory's other
    % illustrative presets. Fixed-cartridge propellant names for the
    % small/medium-caliber rounds are generic placeholders, not real
    % manufacturer designations - unlike separate-loading artillery
    % charges, fixed-cartridge propellant formulations are proprietary
    % per round and not something to state as fact without a source.
    methods (Static)

        %% DM92 modular propelling charge (155mm/52-cal) - MunitionFactory.createMOD274
        function c = getDM92(zone)
            v0_table = containers.Map({'3M', '4M', '5M', '6M'}, {670.0, 770.0, 860.0, 943.0});
            sd_table = containers.Map({'3M', '4M', '5M', '6M'}, {4.0, 4.0, 3.5, 3.0});
            if ~isKey(v0_table, zone)
                error('ChargeFactory:UnknownZone', 'DM92 has no charge zone "%s".', zone);
            end
            c = ChargeZone(zone, 'DM92', v0_table(zone), sd_table(zone));
        end

        %% M119A2 modular propelling charge (155mm) - MunitionFactory.createM107Artillery
        function c = getM119A2(zone)
            % Charge7 = 827 m/s preserves the value used throughout the
            % existing M107 runners/scenarios.
            v0_table = containers.Map({'Charge5', 'Charge6', 'Charge7', 'Charge8'}, {650.0, 740.0, 827.0, 890.0});
            sd_table = containers.Map({'Charge5', 'Charge6', 'Charge7', 'Charge8'}, {4.0, 5.0, 6.0, 7.0});
            if ~isKey(v0_table, zone)
                error('ChargeFactory:UnknownZone', 'M119A2 has no charge zone "%s".', zone);
            end
            c = ChargeZone(zone, 'M119A2', v0_table(zone), sd_table(zone));
        end

        %% Single fixed-cartridge charge (no selectable zones)
        function c = getFixed(propellant_name, v0, stddev)
            if nargin < 1, propellant_name = 'Fixed'; end
            if nargin < 2, v0 = 0.0; end
            if nargin < 3, stddev = 0.0; end
            c = ChargeZone('Fixed', propellant_name, v0, stddev);
        end

    end
end
