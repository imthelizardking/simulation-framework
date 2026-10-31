function validation_results = validate_wrapper(factory_func, test_matrix, dt, log_to_file)
    % Runs a batch validation suite against baseline trajectory data.
    
    if nargin < 3, dt = 0.01; end
    if nargin < 4, log_to_file = true; end
    filename = sprintf('validation_report_%s.txt', datestr(now, 'yyyymmdd_HHMMSS'));
    logger = BaseLogger(true, log_to_file, filename);

    num_tests = size(test_matrix, 1);
    validation_results = struct('QE', {}, 'V0', {}, 'SimOut', {}, 'Error', {});
    env = Atmosphere(); 
    launcher_pos = [0; 0; 0];
    azimuth = 0.0; % Firing due North

    logger.writeLine('============================================================');
    logger.writeLine('Starting Validation Suite: %d Test Cases', num_tests);
    logger.writeLine('Report saved to: %s', filename);
    logger.writeLine('============================================================');

    for i = 1:num_tests
        % 1. Extract Test Conditions
        qe_rad = test_matrix(i, 1);
        v0     = test_matrix(i, 2);
        
        expected_tof       = test_matrix(i, 3);
        expected_downrange = test_matrix(i, 4);
        expected_drift     = test_matrix(i, 5);

        % 2. Instantiate Fresh Model & Setup State
        missile = factory_func();
        
        if isa(missile.Dynamics, 'Dynamics6DoF')
            spin_init = 0.0;
            inertia = diag(missile.MassProperties.InertiaFull);
            missile.Dynamics.InitialState = missile.buildLaunchState(launcher_pos, v0, qe_rad, azimuth, spin_init, inertia);
        else
            missile.Dynamics.InitialState = missile.buildLaunchState(launcher_pos, v0, qe_rad, azimuth);
        end

        % 3. Configure Silent Simulation
        sim = Simulation(missile, [], env, [0 300], dt);
        sim.Visualizer = []; % DISABLING VISUALIZATION for speed
        
        % --- NEW: Frame-Aware Configuration ---
        if isa(missile.Dynamics, 'DynamicsMPMM')
            sim.Frame = 'FDR';
        else
            sim.Frame = 'NED';
        end
        
        sim.Termination.MinAltitude = 0; % Stop at ground impact
        
        % 4. Execute
        telemetry = sim.run();

        % 5. Extract Outputs (Frame-Aware)
        actual_tof = telemetry.time(end);
        final_pos  = telemetry.pos(end, :);
        
        if isa(missile.Dynamics, 'DynamicsMPMM')
            actual_downrange = final_pos(1); % FDR X = Downrange
            actual_drift     = final_pos(3); % FDR Z = Lateral Drift
        else
            actual_downrange = final_pos(1); % NED X = Downrange
            actual_drift     = final_pos(2); % NED Y = Lateral Drift
        end

        % 6. Compute Errors
        err_tof       = actual_tof - expected_tof;
        err_downrange = actual_downrange - expected_downrange;
        err_drift     = actual_drift - expected_drift;

        % Store
        validation_results(i).QE = qe_rad;
        validation_results(i).V0 = v0;
        validation_results(i).SimOut = [actual_tof, actual_downrange, actual_drift];
        validation_results(i).Error  = [err_tof, err_downrange, err_drift];

        % Report
        fprintf('Test %d | QE: %.1f mils | V0: %.1f m/s\n', i, FireControlComputer.rad2mil(qe_rad), v0);
        fprintf('  TOF       : Sim = %7.2f s | Baseline = %7.2f s | Err = %+6.2f s\n', actual_tof, expected_tof, err_tof);
        fprintf('  Downrange : Sim = %7.1f m | Baseline = %7.1f m | Err = %+6.1f m\n', actual_downrange, expected_downrange, err_downrange);
        fprintf('  Side Drift: Sim = %7.1f m | Baseline = %7.1f m | Err = %+6.1f m\n', actual_drift, expected_drift, err_drift);
        fprintf('------------------------------------------------------------\n');
    end
end