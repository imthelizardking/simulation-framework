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
            ChargeFactory.applyFormFactors(c, 'getDM92', zone);
        end

        %% M119A2 modular propelling charge (155mm) - MunitionFactory.createM107Artillery
        function c = getM119A2(zone)
            % Charge5-7 updated to 415/497/580 m/s per newer source data.
            % Charge8 removed - not part of this charge system.
            v0_table = containers.Map({'Charge5', 'Charge6', 'Charge7'}, {415.0, 497.0, 580.0});
            sd_table = containers.Map({'Charge5', 'Charge6', 'Charge7'}, {4.0, 5.0, 6.0});
            if ~isKey(v0_table, zone)
                error('ChargeFactory:UnknownZone', 'M119A2 has no charge zone "%s".', zone);
            end
            c = ChargeZone(zone, 'M119A2', v0_table(zone), sd_table(zone));
            ChargeFactory.applyFormFactors(c, 'getM119A2', zone);
        end

        %% Single fixed-cartridge charge (no selectable zones)
        function c = getFixed(propellant_name, v0, stddev)
            if nargin < 1, propellant_name = 'Fixed'; end
            if nargin < 2, v0 = 0.0; end
            if nargin < 3, stddev = 0.0; end
            c = ChargeZone('Fixed', propellant_name, v0, stddev);
            ChargeFactory.applyFormFactors(c, 'getFixed', propellant_name);
        end

    end

    methods (Static, Access = private)
        function applyFormFactors(chargeZone, method_name, zone)
            % Optional data/<method_name>_<zone>.<ext> file overrides this
            % specific propelling charge's STANAG 4355 MPMM form factors
            % (IForm/FL/QD/QM - see ChargeZone/AeroMPMM); ChargeZone
            % already default to 1.0 (no correction) when absent.
            % ChargeZone is a handle, so mutating it here is visible to
            % the caller without needing to return it.
            %
            % Best-effort file-naming convention - no real sample file
            % was available to confirm the exact layout real firing-table
            % data uses for these, unlike e.g. DatcomReader, which was
            % reverse-engineered against a genuine sample (see that
            % class). Field NAME spelling itself (IForm vs IFORM vs
            % I_Form vs ...) is handled leniently by
            % FactoryData.getOrAny, since that's the part explicitly
            % expected to vary; if your real files also organize/name
            % the FILES differently (e.g. one file per propellant system
            % covering every zone, rather than one per zone), adjust the
            % FactoryData.load() call below accordingly.
            data = FactoryData.load(sprintf('%s_%s', method_name, zone));
            chargeZone.IForm = FactoryData.getOrAny(data, {'IForm'}, chargeZone.IForm);
            chargeZone.FL    = FactoryData.getOrAny(data, {'FL'},    chargeZone.FL);
            chargeZone.QD    = FactoryData.getOrAny(data, {'QD'},    chargeZone.QD);
            chargeZone.QM    = FactoryData.getOrAny(data, {'QM'},    chargeZone.QM);
        end
    end
end
