%% Generate an example data/M107_155mm.mat (MunitionData format)
% One-off setup script, not a scenario runner: run this once in MATLAB to
% produce data/M107_155mm.mat, then re-run runner_m107.m or
% MunitionFactory.createM107Artillery() to see these values override the
% hardcoded illustrative defaults in that preset. Values here are
% DELIBERATELY different from those defaults - and CD0 is a genuine
% Mach-dependent table here, where the hardcoded default is a flat
% constant - so the override is visibly distinguishable, not just a
% silent copy of the existing numbers.
%
% Schema confirmed against a real prototype file (data/mun_test.mat) -
% see MunitionData.m's class comment for the full layout.
clear; clc;

% aero.phase.<Coeff>: Nx(1+K) matrix, column 1 = Mach breakpoint,
% columns 2..end = a0, a1, ... (row's local polynomial-in-Mach, valid
% from that row's breakpoint onward). CD0 here traces a transonic drag
% bucket (rises approaching Mach 1, falls off supersonic); the other
% coefficients are plain Mach-invariant scalars - both forms are valid,
% see MunitionData.toCoeff. Field names match this framework's own
% vocabulary here (CSPIN/CMAG); MunitionFactory also accepts the
% mun_test.mat prototype's alternate spelling (CMASPIN/CMAGF) via
% FactoryData.getOrAny, in case your own files use that instead.
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

S.inertial.mass = 43.2;
% inertia_matrix (if present) takes priority over the axial/transverse
% scalar pair (see MunitionData.inertia) - using the scalar pair here to
% demonstrate that path; a dual-spin munition would set inertia_matrix
% (3x3 or 4x4) directly instead.
S.inertial.inertial_axial      = 0.14;
S.inertial.inertial_transverse = 1.62;

% iform/fl: per-charge-zone polynomial-in-QE (quadrant elevation, mils),
% value = a0 + a1*QE + a2*QE^2 (see MunitionData.formFactor). Zone codes
% match ChargeFactory.getM4A2's - M4A2 (CH-prefix) zones 3-7 (White bag)
% plus the zone-7 supercharge (Red bag). Illustrative: a mild, monotonic
% increase with zone number and a small QE-dependent term, not measured
% data.
zones  = {'CH3W', 'CH4W', 'CH5W', 'CH6W', 'CH7W', 'CH7R'};
for i = 1:numel(zones)
    S.iform.(zones{i}) = [1.00 + 0.01*i, 5e-5*i, 0.0];
    S.fl.(zones{i})    = [0.98 + 0.01*i, 3e-5*i, 0.0];
end

here = fileparts(mfilename('fullpath'));
matPath = fullfile(here, 'data', 'M107_155mm.mat');
save(matPath, 'S');
fprintf('Wrote %s\n', matPath);

% Preview via the same accessors MunitionFactory/runner_m107.m actually
% use, round-tripped through MunitionData.load, so this previews exactly
% what they'll read, not a hand-rebuilt copy of it.
mdata = MunitionData.load('M107_155mm');
cd0_coeff = MunitionData.aeroCoeffs(mdata).CD0;
fprintf('CD0 at Mach 0/0.95/1.5:  %.3f / %.3f / %.3f (flat 0.22 without this file)\n', ...
    BaseAero.evalCoeff(cd0_coeff, 0.0), ...
    BaseAero.evalCoeff(cd0_coeff, 0.95), ...
    BaseAero.evalCoeff(cd0_coeff, 1.5));
fprintf('IForm for CH7W at QE=600/1000 mils: %.4f / %.4f (1.0 without this file)\n', ...
    MunitionData.formFactor(mdata, 'iform', 'CH7W', 600, 1.0), ...
    MunitionData.formFactor(mdata, 'iform', 'CH7W', 1000, 1.0));
fprintf('Re-run runner_m107.m or MunitionFactory.createM107Artillery() to see it take effect.\n');
