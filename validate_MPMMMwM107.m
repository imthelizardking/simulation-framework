% Script: validate_m107_mpmm.m
% Batch validation for the 155mm M107 HE Projectile using the Modified Point Mass Model.

clear; clc;

% 1. Define Test Matrix for M107 (Charge 7 White)
% Standard M107 approx firing table values: 
% Columns: [QE_rad, V0_m/s, Expected_TOF_s, Expected_Downrange_m, Expected_Drift_m]
% Note: Drift is typically positive (right) for a right-hand twist barrel.
test_matrix_m107 = [
    FireControlComputer.mil2rad(300),  684.0,  34.5, 11500.0,   60.0;
    FireControlComputer.mil2rad(500),  684.0,  55.2, 15300.0,  140.0;
    FireControlComputer.mil2rad(800),  684.0,  85.0, 18100.0,  320.0
];

% 2. Define Factory Handle
% We pass the 'CH7W' charge zone to correctly trigger STANAG 4355 form factors if they exist
factory_handle = @() MunitionFactory.createM107Artillery('CH7W');

% 3. Set Integration Time Step
% MPMM does not integrate high-frequency 6-DoF rigid body roll, so it is 
% extremely numerically stable. We can use a much larger dt (50 ms) for rapid execution.
dt = 0.05; 

% 4. Execute Suite
results = validate_wrapper(factory_handle, test_matrix_m107, dt);