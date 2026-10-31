% Define Test Matrix
% Columns: [QE_rad, V0_m/s, Expected_TOF, Expected_Downrange, Expected_Drift]
test_matrix = [
    deg2rad(15), 45.0,  18.5,  6200.0, -15.0;
    deg2rad(30), 45.0,  32.1,  9800.0, -45.0;
    deg2rad(45), 45.0,  45.0, 12500.0, -120.0
];

% Run Validation for Hydra 70mm (Unguided DATCOM model)
factory_handle = @() MunitionFactory.createHydra70mm('MISSILEDATCOM_HYDRA70');
dt = 0.005; % 1000 Hz requirement for stable rolling 6DoF integration

results = run_validation_suite(factory_handle, test_matrix, dt);