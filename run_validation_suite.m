function validation_results = run_validation_suite(factory_func, test_matrix, dt)
% Runs a batch validation suite against baseline trajectory data.
%
% Inputs:
%   factory_func - Function handle to the munition (e.g., @() MunitionFactory.createHydra70mm())
%   test_matrix  - Nx5 matrix: [QE_rad, V0_m_s, expected_TOF, expected_Downrange, expected_Drift]
%   dt           - Integration time step (e.g., 0.01 for 3DoF, 0.001 for spinning 6DoF)

if nargin < 3, dt = 0.01; end

num_tests = size(test_matrix, 1);
validation_results = struct('QE', {}, 'V0', {}, 'SimOut', {}, 'Error', {});
env = Atmosphere();
launcher_pos = [0; 0; 0];
azimuth = 0.0; % Firing due North (X-axis)

fprintf('============================================================\n');
fprintf('Starting Validation Suite: %d Test Cases\n', num_tests);
fprintf('============================================================\n');

for i = 1:num_tests
    % 1. Extract Test Conditions
    qe_rad = test_matrix(i, 1);
    v0     = test_matrix(i, 2);

    expected_tof       = test_matrix(i, 3);
    expected_downrange = test_matrix(i, 4);
    expected_drift     = test_matrix(i, 5);

    % 2. Instantiate Fresh Model & Setup State
    missile = factory_func();

    % Check if 6DoF requires extra launch parameters (spin, inertia)
    if isa(missile.Dynamics, 'Dynamics6DoF')
        spin_init = 0.0; % Override here if testing spinning unguided baseline
        inertia = diag(missile.MassProperties.InertiaFull);
        missile.Dynamics.InitialState = missile.buildLaunchState(launcher_pos, v0, qe_rad, azimuth, spin_init, inertia);
    else
        missile.Dynamics.InitialState = missile.buildLaunchState(launcher_pos, v0, qe_rad, azimuth);
    end

    % 3. Configure Silent Simulation
    sim = Simulation(missile, [], env, [0 300], dt);
    sim.Visualizer = []; % DISABLING VISUALIZATION for maximum speed
    sim.Termination.MinAltitude = 0; % Stop at ground impact

    % 4. Execute
    telemetry = sim.run();

    % 5. Extract Outputs
    actual_tof       = telemetry.time(end);
    final_pos        = telemetry.pos(end, :);
    actual_downrange = final_pos(1); % Assuming North/X launch
    actual_drift     = final_pos(2); % East/Y crossrange

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