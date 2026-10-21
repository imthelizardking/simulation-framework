function runAllTests()
%RUNALLTESTS Run the regression suite; errors (nonzero exit) on failure.
    here = fileparts(mfilename('fullpath'));
    addpath(fileparts(here), here);
    res = runtests(here);
    disp(table(res));
    assert(all([res.Passed]), 'Regression tests failed.');
end
