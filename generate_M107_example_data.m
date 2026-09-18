%% Generate an example data/M107_155mm.mat (MunitionData format)
% One-off setup script, not a scenario runner: run this once in MATLAB to
% produce data/M107_155mm.mat, then re-run runner_m107.m or
% MunitionFactory.createM107Artillery() to see these values override the
% hardcoded illustrative defaults in that preset. Values here are
% DELIBERATELY different from those defaults - and CD0 is a genuine
% Mach-dependent table here, where the hardcoded default is a flat
% constant - so the override is visibly distinguishable, not just a
% silent copy of the existing numbers. See MunitionData.m for the format
% itself (data/<Munition.Name>.mat, aero.phase.<Coeff> matrices,
% inertial.Mass/.Inertia).
clear; clc;

% aero.phase.<Coeff>: Nx(1+K) matrix, column 1 = Mach breakpoint,
% columns 2..end = a0, a1, ... (row's local polynomial-in-Mach, valid
% from that row's breakpoint onward). CD0 here traces a transonic drag
% bucket (rises approaching Mach 1, falls off supersonic); the other
% coefficients are plain Mach-invariant scalars - both forms are valid,
% see MunitionData.toCoeff.
S.aero.phase.CD0 = [ ...
    0.0  0.20   0.0;  ...
    0.7  0.21   0.05; ...
    0.9  0.26   0.30; ...
    1.0  0.40   0.0;  ...
    1.2  0.36  -0.02; ...
    2.0  0.30  -0.01];
S.aero.phase.CLA   = 2.7;
S.aero.phase.CSPIN = -0.035;
S.aero.phase.CMA   = 2.1;
S.aero.phase.CMAG  = 1.25;

S.inertial.Mass    = 43.2;
S.inertial.Inertia = diag([0.14, 1.62, 1.62]);

here = fileparts(mfilename('fullpath'));
matPath = fullfile(here, 'data', 'M107_155mm.mat');
save(matPath, 'S');
fprintf('Wrote %s\n', matPath);

% Preview the CD0 table's Mach-dependence via the same conversion
% MunitionData.aeroCoeffs applies internally (round-tripped through
% MunitionData.load so this previews exactly what the factory preset
% will actually read, not a hand-rebuilt copy of it).
mdata = MunitionData.load('M107_155mm');
cd0_coeff = MunitionData.aeroCoeffs(mdata).CD0;
fprintf('CD0 at Mach 0/0.95/1.5:  %.3f / %.3f / %.3f (flat 0.22 without this file)\n', ...
    BaseAero.evalCoeff(cd0_coeff, 0.0), ...
    BaseAero.evalCoeff(cd0_coeff, 0.95), ...
    BaseAero.evalCoeff(cd0_coeff, 1.5));
fprintf('Re-run runner_m107.m or MunitionFactory.createM107Artillery() to see it take effect.\n');
