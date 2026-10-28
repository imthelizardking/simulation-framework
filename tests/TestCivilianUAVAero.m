classdef TestCivilianUAVAero < matlab.unittest.TestCase
    methods (Test)
        function scalesLoadsWithAreaAndChord(tc)
            a = AeroCivilianUAV();
            a.Coefficients = struct('CD0',0.1,'CL0',0.2,'Cm0',0.3);
            [F,M] = a.bodyLoads(0,[10;0;0],zeros(3,1),1,100);
            tc.verifyEqual(F,[-5;0;-10],'AbsTol',1e-12);
            tc.verifyEqual(M,[0;7.5;0],'AbsTol',1e-12);
            a.RefArea = 2;
            [twiceF,twiceM] = a.bodyLoads(0,[10;0;0],zeros(3,1),1,100);
            tc.verifyEqual(twiceF,2*F,'AbsTol',1e-12);
            tc.verifyEqual(twiceM,2*M,'AbsTol',1e-12);
        end

        function liftAndSideDoNoWorkAlongAirVelocity(tc)
            a = AeroCivilianUAV();
            a.Coefficients = struct('CD0',0.1,'CL0',0.4,'CY_beta',-0.5);
            velocity = [12;2;3];
            [F,~,info] = a.bodyLoads(0,velocity,zeros(3,1),1.2,340);
            expectedPower = -0.5*1.2*norm(velocity)^3*0.1*a.RefArea;
            tc.verifyEqual(dot(F,velocity),expectedPower,'AbsTol',1e-10);
            tc.verifyGreaterThan(info.Beta,0);
            tc.verifyGreaterThan(info.Alpha,0);
        end

        function sideslipProducesLateralLoads(tc)
            a = AeroCivilianUAV();
            a.Coefficients = struct('CY_beta',-0.5,'Cl_beta',-0.2,'Cn_beta',0.1);
            [positiveF,positiveM] = a.bodyLoads(0,[10;1;0],zeros(3,1),1,100);
            [negativeF,negativeM] = a.bodyLoads(0,[10;-1;0],zeros(3,1),1,100);
            tc.verifyLessThan(positiveF(2),0);
            tc.verifyEqual(negativeF(2),-positiveF(2),'AbsTol',1e-12);
            tc.verifyEqual(negativeM,-positiveM,'AbsTol',1e-12);
            tc.verifyLessThan(positiveM(1),0);
            tc.verifyGreaterThan(positiveM(3),0);
        end

        function dampingOpposesBodyRates(tc)
            a = AeroCivilianUAV();
            a.Coefficients = struct('Cl_p',-1,'Cm_q',-2,'Cn_r',-3);
            rates = [0.2;-0.3;0.4];
            [~,M] = a.bodyLoads(0,[10;0;0],rates,1,100);
            tc.verifyLessThan(dot(M,rates),0);
            tc.verifyTrue(all(M.*rates < 0));
        end

        function controlsAndForceRateTermsAreActive(tc)
            a = AeroCivilianUAV();
            a.Coefficients = struct('CL_q',1,'CY_p',1,'CY_r',1, ...
                'CL_elevator',1,'Cm_elevator',-1,'Cl_aileron',1,'Cn_rudder',1);
            controls = struct('Elevator',0.1,'Aileron',0.2,'Rudder',0.3);
            [F,M] = a.bodyLoads(0,[10;0;0],[0.2;0.3;0.4],1,100,controls);
            tc.verifyGreaterThan(F(2),0);
            tc.verifyLessThan(F(3),0);
            tc.verifyGreaterThan(M(1),0);
            tc.verifyLessThan(M(2),0);
            tc.verifyGreaterThan(M(3),0);
        end

        function unsteadyInputsAreExplicitAndHistoryIndependent(tc)
            a = AeroCivilianUAV();
            a.Coefficients = struct('CL_alphaDot',1,'Cm_alphaDot',-1);
            tc.verifyError(@() a.bodyLoads(0,[10;0;0],zeros(3,1),1,100), ...
                'AeroCivilianUAV:AlphaDot');
            input = struct('AlphaDot',0.4);
            [F,M] = a.bodyLoads(2,[10;0;0],zeros(3,1),1,100,input);
            [againF,againM] = a.bodyLoads(0,[10;0;0],zeros(3,1),1,100,input);
            tc.verifyEqual(againF,F);
            tc.verifyEqual(againM,M);
            tc.verifyLessThan(F(3),0);
            tc.verifyLessThan(M(2),0);
        end

        function scheduledDeploymentAndMachLookup(tc)
            a = createCivilianUAVExampleAero();
            a.DeploymentTime = 5; a.DeploymentDuration = 2;
            before = a.coefficientsAt(5,0.1);
            middle = a.coefficientsAt(6,0.1);
            after = a.coefficientsAt(7,0.1);
            tc.verifyEqual([before.CL_alpha middle.CL_alpha after.CL_alpha],[1 2.5 4]);
            tc.verifyEqual(a.coefficientsAt(0,0.1),before);
            a = AeroCivilianUAV();
            a.Coefficients = struct('CD0',struct('Type','Table','Mach',[0 0.2],'Value',[0.1 0.3]));
            c = a.coefficientsAt(0,0.1);
            tc.verifyEqual(c.CD0,0.2,'AbsTol',1e-12);
        end

        function adapterSubtractsWindAndNormalizesQuaternion(tc)
            a = AeroCivilianUAV(); env = Atmosphere();
            X = zeros(21,1); X(3) = -10; X(4) = 10; X(18) = 1; X(14) = 2;
            [F,M] = a.getForcesAndMoments(0,X,env,0);
            env.BaseWind_NED = [5;0;0]; X(4) = 15; X(18) = 2;
            [sameF,sameM] = a.getForcesAndMoments(0,X,env,0);
            tc.verifyEqual(sameF,F,'AbsTol',1e-12);
            tc.verifyEqual(sameM,M,'AbsTol',1e-12);
        end

        function compatibleWithVehicleAndSimulation(tc)
            a = AeroCivilianUAV();
            m = Munition('Civilian_UAV_Test'); m.Type = 'Civilian UAV';
            m.Aero = a; m.Dynamics = Dynamics6DoF();
            m.MassProperties = struct('Mass',2,'Inertia',eye(3));
            X = m.buildLaunchState([0;0;10],10,0,0);
            % A rotated adapter/dynamics pair must still dissipate energy
            % along inertial velocity rather than accelerate into the wind.
            X(4:6) = [0;-10;0];
            X(18:21) = Dynamics6DoF.eulerToQuat(0,0,pi/2);
            derivative = m.getDerivatives(0,X,Atmosphere());
            tc.verifyGreaterThan(derivative(5),0);
            tc.verifyEqual(derivative(4),0,'AbsTol',1e-12);
            m.Dynamics.InitialState = X;
            sim = Simulation(m,[],Atmosphere(),[0 0.02],0.01);
            sim.Frame = 'NED'; sim.Termination.MinAltitude = 0;
            evalc('tel = sim.run();');
            tc.verifyTrue(all(isfinite(tel.pos(:))));
            tc.verifyEqual(tel.mass,2*ones(3,1));
        end

        function rejectsAmbiguityAndHandlesZeroSpeed(tc)
            a = AeroCivilianUAV();
            tc.verifyError(@() a.setConfiguration('Bad',struct('CNQ',1)), ...
                'AeroCivilianUAV:Coefficients');
            a.Coefficients = struct('CD0',-0.1);
            tc.verifyError(@() a.bodyLoads(0,[10;0;0],zeros(3,1),1,100), ...
                'AeroCivilianUAV:NegativeDrag');
            a.Coefficients = struct('CD0',0.1,'CL_alphaDot',1);
            [F,M] = a.bodyLoads(0,zeros(3,1),ones(3,1),1,100);
            tc.verifyEqual(F,zeros(3,1)); tc.verifyEqual(M,zeros(3,1));
        end
    end
end
