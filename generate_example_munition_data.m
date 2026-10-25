%% Generate example data/<Name>.mat files for every remaining munition
% One-off setup script, not a scenario runner: writes a MunitionData.m-
% format .mat file for every MunitionFactory preset that doesn't already
% have one, using EXACTLY the same illustrative numbers that preset's
% own hardcoded defaults already use - so this only relocates existing
% values into the new standard file-based format, it doesn't change any
% current behavior. Run once in MATLAB; re-run any MunitionFactory
% preset afterward to see it now sourcing from these files instead of
% its own hardcoded fallback.
%
% M107_155mm.mat isn't written here - see generate_M107_example_data.m,
% which deliberately uses DIFFERENT numbers (to make the override
% visibly distinguishable). MOD274_155mm_BB.mat isn't written here
% either - runner_mod274_datatest.m has its own dedicated generator with
% deliberately different (high-drag test) values for that same reason;
% running both this script and that runner would just have whichever
% ran last win.
clear; clc;
here = fileparts(mfilename('fullpath'));

%% Hydra_70_M151 - MunitionFactory.createHydra70 (Aero3DoF: CD0/CL_alpha only)
S = struct();
S.aero.phase.CD0      = 0.38;
S.aero.phase.CL_alpha = 3.2;
S.inertial.mass                = 10.5;
S.inertial.inertial_axial      = 0.008;
S.inertial.inertial_transverse = 0.45;
write_mat(here, 'Hydra_70_M151', S);

%% FinStabilized_Dart - MunitionFactory.createFinStabilizedDart
S = struct();
S.aero.phase.CD0      = 0.3;
S.aero.phase.CL_alpha = 5.0;
S.inertial.mass                = 0.1;
S.inertial.inertial_axial      = 1e-5;
S.inertial.inertial_transverse = 1e-5;
write_mat(here, 'FinStabilized_Dart', S);

%% 25mm_M793 - MunitionFactory.create25mmM793 (AeroMPMM)
S = struct();
S.aero.phase.CD0   = 0.18;
S.aero.phase.CDA2  = 0;
S.aero.phase.CDA4  = 0;
S.aero.phase.CLA   = 1.5;
S.aero.phase.CLA3  = 0;
S.aero.phase.CSPIN = -0.02;
S.aero.phase.CMA   = 1.2;
S.aero.phase.CMA3  = 0;
S.aero.phase.CMAG  = 0.6;
S.inertial.mass                = 0.185;
S.inertial.inertial_axial      = 1e-4;
S.inertial.inertial_transverse = 1e-4;
write_mat(here, '25mm_M793', S);

%% 35mm_HET - MunitionFactory.create35mmHE (AeroMPMM)
S = struct();
S.aero.phase.CD0   = 0.25;
S.aero.phase.CDA2  = 0;
S.aero.phase.CDA4  = 0;
S.aero.phase.CLA   = 2.0;
S.aero.phase.CLA3  = 0;
S.aero.phase.CSPIN = -0.025;
S.aero.phase.CMA   = 1.6;
S.aero.phase.CMA3  = 0;
S.aero.phase.CMAG  = 0.8;
S.inertial.mass                = 0.55;
S.inertial.inertial_axial      = 0.0004;
S.inertial.inertial_transverse = 0.006;
write_mat(here, '35mm_HET', S);

%% 76mm_HE - MunitionFactory.create76mmHE (AeroMPMM)
% CD0/CLA were originally Mach-Value interpolation tables ('Table'
% type); converted to EXACTLY equivalent piecewise-linear [Mach,a0,a1]
% rows (a1 = local slope, a0 chosen so the row's own formula reproduces
% the original table's value at that breakpoint - the same result as
% linear interpolation, just expressed as this format's local
% polynomials). CDA2/CLA3 were originally unclamped 'Polynomial' types
% (one formula for all Mach, no breakpoints) - CLA3 was already linear,
% so its 2-row form below is exact; CDA2 was quadratic, so its rows are
% a close piecewise-linear APPROXIMATION of the original curve, not a
% bit-for-bit match (this coefficient set is explicitly illustrative/
% unvalidated data already, per MunitionFactory.create76mmHE's own
% comment, so an approximation here doesn't lose anything real).
S = struct();
S.aero.phase.CD0 = [ ...
    0.000  0.190000   0.014286; ...
    0.700  0.060000   0.200000; ...
    0.900 -0.840000   1.200000; ...
    0.950 -1.220000   1.600000; ...
    1.000 -0.220000   0.600000; ...
    1.050  0.200000   0.200000; ...
    1.100  0.640000  -0.200000; ...
    1.200  0.600000  -0.166667; ...
    1.500  0.500000  -0.100000; ...
    2.000  0.420000  -0.060000; ...
    2.500  0.370000  -0.040000; ...
    3.000  0.250000   0.000000];
S.aero.phase.CLA = [ ...
    0.000  2.000000   0.142857; ...
    0.700  1.050000   1.500000; ...
    0.900 -1.200000   4.000000; ...
    1.000  3.800000  -1.000000; ...
    1.200  3.800000  -1.000000; ...
    1.500  3.200000  -0.600000; ...
    2.000  2.800000  -0.400000; ...
    2.500  2.800000  -0.400000; ...
    3.000  1.600000   0.000000];
S.aero.phase.CDA2 = [ ...
    0.000  2.200000   0.940000; ...
    0.500  2.260000   0.820000; ...
    1.000  2.380000   0.700000; ...
    1.500  2.560000   0.580000; ...
    2.000  2.800000   0.460000; ...
    2.500  3.100000   0.340000; ...
    3.000  4.120000   0.000000];
S.aero.phase.CLA3  = [0.0 -8.0 1.5; 3.0 -3.5 0.0]; % exact - original was already linear
S.aero.phase.CDA4  = 8.5;
S.aero.phase.CSPIN = -0.03;
S.aero.phase.CMA   = 2.0;
S.aero.phase.CMA3  = 0;
S.aero.phase.CMAG  = 1.0;
S.inertial.mass                = 6.0;
S.inertial.inertial_axial      = 0.006;
S.inertial.inertial_transverse = 0.085;
write_mat(here, '76mm_HE', S);

%% APKWS_Guided_Rocket - MunitionFactory.createAPKWS (Mass only - Aero inherited from Hydra70)
S = struct();
S.inertial.mass = 15.0;
write_mat(here, 'APKWS_Guided_Rocket', S);

%% GuidedRocket_DATCOM - MunitionFactory.createGuidedRocketDatcom (Mass/Inertia only - Aero from ParserDatcom)
S = struct();
S.inertial.mass                = 15.0;
S.inertial.inertial_axial      = 0.008;
S.inertial.inertial_transverse = 0.45;
write_mat(here, 'GuidedRocket_DATCOM', S);

function write_mat(here, name, S)
    matPath = fullfile(here, 'data', [name '.mat']);
    save(matPath, 'S');
    fprintf('Wrote %s\n', matPath);
end
