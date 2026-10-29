classdef TestAeroConfigurations < matlab.unittest.TestCase
    % Synthetic data only: checks configuration routing/interpolation,
    % not the physical aerodynamics of any particular vehicle.
    methods (Test)
        function legacyPropertiesStillWork(tc)
            a = Aero6DoF();
            a.Mach = [0 1]; a.Alpha = [-1 1];
            a.CD = [1 3; 3 5]; a.CL = zeros(2);
            c = a.evaluateCoefficients(0.5, 0);
            tc.verifyEqual(c.CD, 3, 'AbsTol', 1e-12);
            tc.verifyEqual(c.CMQ, 0);
        end

        function switchesAtScheduledTime(tc)
            a = TestAeroConfigurations.model();
            a.DeploymentTime = 5;
            before = a.evaluateCoefficients(0.5, 0, 4.99);
            at = a.evaluateCoefficients(0.5, 0, 5);
            tc.verifyEqual(before.CD, 3, 'AbsTol', 1e-12);
            tc.verifyEqual(at.CD, 12, 'AbsTol', 1e-12);
            % An out-of-order RK4/query call must recover the stowed set.
            again = a.evaluateCoefficients(0.5, 0, 0);
            tc.verifyEqual(again, before);
            tc.verifyEqual(a.DeploymentRatio, 0);
        end

        function blendsDifferentGridsAndDerivatives(tc)
            a = TestAeroConfigurations.model();
            a.DeploymentTime = 5; a.DeploymentDuration = 2;
            first = a.evaluateCoefficients(0.5, 0, 5);
            middle = a.evaluateCoefficients(0.5, 0, 6);
            last = a.evaluateCoefficients(0.5, 0, 7);
            tc.verifyEqual([first.CD middle.CD last.CD], [3 7.5 12], 'AbsTol', 1e-12);
            tc.verifyEqual(middle.CMQ, -3, 'AbsTol', 1e-12);
            tc.verifyEqual(middle.CA, 3, 'AbsTol', 1e-12);
        end

        function manualRatioAndClamping(tc)
            a = TestAeroConfigurations.model();
            a.DeploymentRatio = 0.5;
            c = a.evaluateCoefficients(99, 99);
            % Each table clamps to its own grid: (5 + 16)/2.
            tc.verifyEqual(c.CD, 10.5, 'AbsTol', 1e-12);
            tc.verifyEqual(a.deploymentRatioAt(100), 0.5);
            a.DeploymentRatio = 2;
            tc.verifyError(@() a.evaluateCoefficients(0, 0), ...
                'Aero6DoF:DeploymentRatio');
        end

        function legacyInitialWithNamedDeployedTable(tc)
            a = TestAeroConfigurations.model();
            a.Mach = a.Tables.Stowed.Mach;
            a.Alpha = a.Tables.Stowed.Alpha;
            a.CD = a.Tables.Stowed.CD; a.CL = a.Tables.Stowed.CL;
            a.ActiveTableKey = 'Clean';
            c = a.evaluateCoefficients(0.5, 0);
            tc.verifyEqual(c.CD, 3, 'AbsTol', 1e-12);
        end

        function forcePathUsesTime(tc)
            a = TestAeroConfigurations.model(); a.Frame = 'NED';
            a.DeploymentTime = 5;
            X = zeros(21, 1); X(4) = 20; X(14) = 1; X(18) = 1;
            env = Atmosphere();
            [before, ~] = a.getForcesAndMoments(0, X, env, 0);
            [after, ~] = a.getForcesAndMoments(5, X, env, 0);
            tc.verifyEqual(before, zeros(3, 1));
            tc.verifyLessThan(after(1), 0);
        end

        function validatesDataAndKeys(tc)
            a = TestAeroConfigurations.model();
            tc.verifyError(@() a.setConfiguration('Bad', struct()), 'Aero6DoF:NoData');
            d = a.Tables.Stowed; d.Mach = [1 0];
            tc.verifyError(@() a.setConfiguration('Bad', d), 'Aero6DoF:Grid');
            d = a.Tables.Stowed; d.CMQ = ones(3);
            tc.verifyError(@() a.setConfiguration('Bad', d), 'Aero6DoF:CoefficientSize');
            d = a.Tables.Stowed; d.RefArea = a.RefArea * 2;
            tc.verifyError(@() a.setConfiguration('Bad', d), 'Aero6DoF:ReferenceMismatch');
            a.ActiveTableKey = 'Typo';
            tc.verifyError(@() a.evaluateCoefficients(0, 0), 'Aero6DoF:MissingConfiguration');
        end

        function supportsSinglePointGridAndObjectSnapshot(tc)
            a = Aero6DoF();
            d = struct('Mach', 0, 'Alpha', 0, 'CD', 1, 'CL', 2);
            a.setConfiguration('Single', d); a.ActiveTableKey = 'Single';
            c = a.evaluateCoefficients(2, 2);
            tc.verifyEqual(c.CL, 2);
            source = Aero6DoF(); source.Mach = 0; source.Alpha = 0;
            source.CD = 4; source.CL = 5;
            a.setConfiguration('Snapshot', source);
            source.CD = 99; a.ActiveTableKey = 'Snapshot';
            c = a.evaluateCoefficients(0, 0);
            tc.verifyEqual(c.CD, 4);
        end

        function supportsOneDimensionalGrids(tc)
            a = Aero6DoF();
            d = struct('Mach', [0 1], 'Alpha', 0, 'CD', [2 4], 'CL', [0 0]);
            a.setConfiguration('MachOnly', d); a.ActiveTableKey = 'MachOnly';
            c = a.evaluateCoefficients(0.5, 9);
            tc.verifyEqual(c.CD, 3, 'AbsTol', 1e-12);
            d = struct('Mach', 0, 'Alpha', [-1 1], 'CD', [2; 4], 'CL', [0; 0]);
            a.setConfiguration('AlphaOnly', d); a.ActiveTableKey = 'AlphaOnly';
            c = a.evaluateCoefficients(9, 0);
            tc.verifyEqual(c.CD, 3, 'AbsTol', 1e-12);
        end
    end

    methods (Static, Access = private)
        function a = model()
            a = Aero6DoF();
            stowed = struct('Mach', [0 1], 'Alpha', [-1 1], ...
                'CD', [1 3; 3 5], 'CL', zeros(2), 'CMQ', -2 * ones(2));
            % CD = 10 + 4*Mach + 2*Alpha; a different grid from stowed.
            deployed = struct('Mach', [0 0.5 1], 'Alpha', [-1 0 1], ...
                'CD', [8 10 12; 10 12 14; 12 14 16], 'CL', ones(3), ...
                'CA', 6 * ones(3), 'CMQ', -4 * ones(3));
            a.setConfiguration('Stowed', stowed);
            a.setConfiguration('Deployed', deployed);
            a.ActiveTableKey = 'Stowed';
            a.DeployedTableKey = 'Deployed';
        end
    end
end
