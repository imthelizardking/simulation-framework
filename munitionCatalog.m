function catalog = munitionCatalog()
    % Single source of truth for which munitions the scenario builder
    % (ScenarioBuilderApp/generateScenarioRunner) offers - both read this
    % same catalog, so the GUI's dropdown and the generator's factory-call
    % mapping can never drift apart. Phase-1 scope: unguided direct/
    % indirect-fire presets only (no sensors/guidance/controller) - all
    % five currently use the DynamicsMPMM/AeroMPMM pair, which is why
    % generateScenarioRunner can hardcode sim.Frame='FDR' without a
    % per-munition branch. Add a preset here (and nowhere else) to make it
    % selectable - charge zone/mode choices are fixed to a sensible
    % default per entry since the GUI doesn't expose those in phase 1.
    catalog = struct();
    catalog.M793_25mm = struct('Label', '25mm M793', ...
        'FactoryCall', 'MunitionFactory.create25mmM793()');
    catalog.HE_35mm = struct('Label', '35mm HE', ...
        'FactoryCall', 'MunitionFactory.create35mmHE()');
    catalog.HE_76mm = struct('Label', '76mm HE', ...
        'FactoryCall', 'MunitionFactory.create76mmHE()');
    catalog.M107 = struct('Label', 'M107 Artillery (155mm, CH7W)', ...
        'FactoryCall', 'MunitionFactory.createM107Artillery(''CH7W'')');
    catalog.MOD274 = struct('Label', 'MOD274 Base-Bleed (155mm, 6M)', ...
        'FactoryCall', 'MunitionFactory.createMOD274(''6M'')');
end
