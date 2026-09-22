classdef ChargeFactory
    % Propellant/charge-zone presets. DM92 zone names and the 6M=943 m/s
    % point are as given; the other DM92 zones and all M4A2/M119A2 zones
    % are illustrative (a plausible monotonic progression), not
    % transcribed from a real firing table - same caveat as MunitionFactory's other
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

        %% M4A2 modular propelling charge (155mm) - MunitionFactory.createM107Artillery
        function c = getM4A2(zone)
            % CH-prefix zones (per data/<Munition>.mat's iform/fl charge-
            % zone keys, see MunitionData): CH<N>W/CH<N>R, N = zone
            % number, W/R = White/Red bag. CH and CX prefixes are two
            % different propellant/charge SYSTEMS - CH is M4A2 (this
            % method), CX is M119A2 (see getM119A2 below), as given - not
            % independently verified against a source. CH5W/CH6W/CH7W =
            % 415/497/580 m/s per the earlier source data this table
            % already had; CH3W/CH4W (lower zones) and CH7R (zone-7
            % supercharge) are new additions, illustrative like the rest
            % of this file - not transcribed from a real firing table.
            v0_table = containers.Map( ...
                {'CH3W', 'CH4W', 'CH5W', 'CH6W', 'CH7W', 'CH7R'}, ...
                {300.0,  360.0,  415.0,  497.0,  580.0,  650.0});
            sd_table = containers.Map( ...
                {'CH3W', 'CH4W', 'CH5W', 'CH6W', 'CH7W', 'CH7R'}, ...
                {3.0,    3.5,    4.0,    5.0,    6.0,    7.0});
            if ~isKey(v0_table, zone)
                error('ChargeFactory:UnknownZone', 'M4A2 has no charge zone "%s".', zone);
            end
            c = ChargeZone(zone, 'M4A2', v0_table(zone), sd_table(zone));
        end

        %% M119A2 modular propelling charge (155mm, extended-range) - not yet paired with a MunitionFactory preset
        function c = getM119A2(zone)
            % CX-prefix zones (see getM4A2's comment on the CH/CX
            % distinction) - M119A2 tops out one zone higher (CX8R) than
            % M4A2 (CH7R), matching the real M119-series' role as the
            % newer, higher-zone successor to the M4-series. Entirely
            % illustrative (a plausible monotonic progression, scaled up
            % a bit from M4A2's) - not transcribed from a real firing
            % table, and no current MunitionFactory preset uses this yet.
            v0_table = containers.Map( ...
                {'CX3W', 'CX4W', 'CX5W', 'CX6W', 'CX7W', 'CX8R'}, ...
                {320.0,  385.0,  445.0,  530.0,  615.0,  700.0});
            sd_table = containers.Map( ...
                {'CX3W', 'CX4W', 'CX5W', 'CX6W', 'CX7W', 'CX8R'}, ...
                {3.0,    3.5,    4.0,    5.0,    6.0,    7.0});
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
