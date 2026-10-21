classdef TestRegression < matlab.unittest.TestCase
%TESTREGRESSION Compare runner outputs to saved baselines.
%   Baselines live in tests/baselines/<name>.mat. After an intentional
%   physics change, regenerate them with tests/updateBaselines.

    properties (TestParameter)
        scenario = {'m107_direct', '25mm', 'sam_engagement'}
    end

    properties (Constant)
        RelTol = 1e-6
        AbsTol = 1e-6
    end

    methods (Test)
        function matchesBaseline(tc, scenario)
            here = fileparts(mfilename('fullpath'));
            file = fullfile(here, 'baselines', [scenario '.mat']);
            tc.assertTrue(isfile(file), ...
                sprintf('No baseline for %s. Run updateBaselines first.', scenario));
            b = load(file); b = b.baseline;
            r = runScenario(scenario);

            tc.verifyEqual(r.reason, b.reason, 'termination reason changed');
            for fld = setdiff(fieldnames(b), {'reason'})'
                tc.assertTrue(isfield(r, fld{1}), sprintf('missing metric %s', fld{1}));
                tc.verifyEqual(r.(fld{1}), b.(fld{1}), ...
                    sprintf('%s drifted from baseline', fld{1}), ...
                    'RelTol', tc.RelTol, 'AbsTol', tc.AbsTol);
            end
        end
    end
end
