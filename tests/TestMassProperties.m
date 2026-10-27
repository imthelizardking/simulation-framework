classdef TestMassProperties < matlab.unittest.TestCase
    methods (Test)
        function fixedConfigurationPreservesInertia(tc)
            config = struct('Mass',12,'Inertia',diag([1 2 3]));
            s = MassPropertiesEvaluator.evaluate(config,12);
            tc.verifyEqual(s.Inertia,config.Inertia);
            tc.verifyEqual(s.CG,zeros(3,1));
            config.Inertia = [1;2;3];
            vectorInput = MassPropertiesEvaluator.evaluate(config,12);
            tc.verifyEqual(vectorInput.Inertia,diag([1 2 3]));
        end

        function endpointSchemaMatchesComponentModel(tc)
            model = UAVMassModel();
            model.DryMass = 2; model.DryInertia = diag([1 2 3]);
            model.InitialPayloadMass = 2; model.PayloadCG = [2;0;0];
            model.PayloadInertiaPerKg = diag([0.1 0.2 0.3]);
            model.ReleaseTime = 1; model.ReleaseDuration = 2;
            full = model.evaluate(0); empty = model.evaluate(4);
            tc.verifyEqual(full.CG,[1;0;0]);
            tc.verifyEqual(full.Inertia,diag([1.2 6.4 7.6]),'AbsTol',1e-12);
            config = struct('Mass',4,'MassEmpty',2,'Inertia',full.Inertia, ...
                'InertiaFull',full.Inertia,'InertiaEmpty',empty.Inertia, ...
                'CGFull',full.CG,'CGEmpty',empty.CG);
            for time = [0 2 4]
                expected = model.evaluate(time);
                actual = MassPropertiesEvaluator.evaluate(config,expected.Mass);
                tc.verifyEqual(actual.CG,expected.CG,'AbsTol',1e-12);
                tc.verifyEqual(actual.Inertia,expected.Inertia,'AbsTol',1e-12);
            end
        end

        function releaseIsBoundedAndOrderIndependent(tc)
            model = UAVMassModel(); model.InitialPayloadMass = 2; model.ReleaseTime = 1;
            full = model.evaluate(0);
            empty = model.evaluate(1);
            tc.verifyEqual([full.Mass empty.Mass],[14 12]);
            tc.verifyEqual(model.evaluate(0),full);
            tc.verifyEqual(model.evaluate(100),empty);
            model.ReleaseDuration = 2;
            middle = model.evaluate(2);
            tc.verifyEqual([middle.Mass middle.MassRate],[13 -1]);
        end

        function threeDofUsesItsOwnMassSlot(tc)
            m = Munition(); m.Dynamics = Dynamics3Dof();
            m.MassProperties = struct('Mass',2,'Inertia',eye(3));
            X = m.buildLaunchState([0;0;10],10,0,0);
            tc.verifyEqual(numel(X),8);
            tc.verifyEqual(m.Dynamics.getMass(X),2);
            config = m.MassProperties;
            dX = m.getDerivatives(0,X,Atmosphere());
            tc.verifyTrue(all(isfinite(dX)));
            tc.verifyEqual(dX(8),0);
            tc.verifyEqual(m.MassProperties,config);
            m.Dynamics.InitialState = X;
            sim = Simulation(m,[],Atmosphere(),[0 0.02],0.01);
            sim.Termination.MinAltitude = 0;
            evalc('tel = sim.run();');
            tc.verifyEqual(tel.mass,2*ones(3,1));
            tc.verifyEqual(tel.cg,zeros(3));
        end

        function existingFactoryInputsUseSharedEvaluation(tc)
            factories = {@() MunitionFactory.createM107Artillery(), ...
                         @() MunitionFactory.create25mmM793()};
            for k = 1:numel(factories)
                m = factories{k}();
                X = m.buildLaunchState(zeros(3,1),10,0,0);
                snapshot = m.massPropertiesAt(0,X);
                tc.verifyEqual(snapshot.Mass,m.MassProperties.Mass);
                tc.verifyEqual(snapshot.Inertia,m.MassProperties.Inertia);
                tc.verifyEqual(X(15:17),diag(m.MassProperties.Inertia));
            end
        end

        function substepsDoNotMutateConfiguration(tc)
            m = Munition(); m.Dynamics = Dynamics6DoF();
            model = UAVMassModel(); model.InitialPayloadMass = 1; model.ReleaseTime = 1;
            m.MassModel = model;
            X = m.buildLaunchState([0;0;10],10,0,0);
            [X, accepted] = m.synchronizeMassState(0,X);
            config = m.MassProperties;
            m.getDerivatives(2,X,Atmosphere());
            m.getDerivatives(0.5,X,Atmosphere());
            tc.verifyEqual(m.MassProperties,config);
            tc.verifyEqual(m.CurrentMassProperties,accepted);
            tc.verifyEqual(m.Dynamics.CurrentState,X);
        end

        function motorCumulativeMassIsExactAtBoundaries(tc)
            motor = Propulsion_SolidMotor([0 0;1 10;2 0],2);
            tc.verifyEqual(motor.massConsumed(-1),0);
            tc.verifyEqual(motor.massConsumed(0.5),0.25,'AbsTol',1e-12);
            tc.verifyEqual(motor.massConsumed(1),1,'AbsTol',1e-12);
            tc.verifyEqual(motor.massConsumed(3),2);
            m = Munition(); m.Dynamics = Dynamics3Dof(); m.Propulsion = motor;
            m.MassProperties = struct('Mass',5,'Inertia',eye(3));
            X = m.buildLaunchState([0;0;10],10,0,0);
            [X,s] = m.synchronizeMassState(3,X);
            tc.verifyEqual([m.Dynamics.getMass(X) s.Mass],[3 3]);
        end

        function mpmmUsesCurrentMass(tc)
            d = DynamicsMPMM(); d.SimCoriolis = false;
            X = d.buildLaunchState(zeros(3,1),10,0,0,3,0);
            config = struct('Mass',9,'Inertia',eye(3));
            moments = struct('Aero',zeros(3,1),'Propulsion',zeros(3,1));
            forces = struct('Aero',zeros(3,1),'Propulsion',zeros(3,1));
            without = d.computeDerivatives(0,X,forces,moments,config,0);
            forces.Aero = [6;0;0];
            with = d.computeDerivatives(0,X,forces,moments,config,0);
            tc.verifyEqual(with(4)-without(4),2,'AbsTol',1e-12);
        end

        function bodyMomentTransfersTowardCurrentCg(tc)
            m = Munition(); m.Dynamics = Dynamics6DoF();
            m.MassProperties = struct('Mass',2,'Inertia',eye(3),'CG',[1;0;0]);
            a = Aero6DoF(); a.Frame = 'NED';
            a.Mach = [0 1]; a.Alpha = [-1 1];
            a.CD = zeros(2); a.CL = zeros(2); a.CN = ones(2);
            m.Aero = a;
            X = m.buildLaunchState([0;0;10],10,0,0);
            dX = m.getDerivatives(0,X,Atmosphere());
            tc.verifyLessThan(dX(11),0);
        end

        function fullTensorIsUsed(tc)
            d = Dynamics6DoF();
            X = d.buildLaunchState(zeros(3,1),0,0,0,2);
            I = [2 0.2 0;0.2 3 0;0 0 4];
            config = struct('Inertia',I);
            forces = struct('Aero',zeros(3,1),'Propulsion',zeros(3,1));
            moments = struct('Aero',[0;1;0],'Propulsion',zeros(3,1));
            derivative = d.computeDerivatives(0,X,forces,moments,config,0);
            tc.verifyEqual(I*derivative(10:12),[0;1;0],'AbsTol',1e-12);
            tc.verifyNotEqual(derivative(10),0);
        end

        function launchPopulatesConfiguredInertiaAndSpin(tc)
            m = Munition(); m.Dynamics = Dynamics6DoF();
            m.MassProperties = struct('Mass',2,'Inertia',diag([1 2 3]));
            X = m.buildLaunchState(zeros(3,1),10,0,0,4);
            tc.verifyEqual(X(15:17),[1;2;3]);
            tc.verifyEqual(X(10),4);
            d = Dynamics6DoF();
            direct = d.buildLaunchState(zeros(3,1),10,0,0,2,4);
            tc.verifyEqual(direct(10),4);
        end

        function rejectsImpossibleConfiguration(tc)
            config = struct('Mass',2,'MassEmpty',3,'Inertia',eye(3));
            tc.verifyError(@() MassPropertiesEvaluator.evaluate(config,2),'MassProperties:MassBounds');
            model = UAVMassModel(); model.DryInertia = diag([1 1 10]);
            tc.verifyError(@() model.evaluate(0),'UAVMassModel:Inertia');
        end
    end
end
