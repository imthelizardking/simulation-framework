function updateBaselines(names)
%UPDATEBASELINES Regenerate tests/baselines/*.mat from the current code.
%   Only run this after confirming a change in results is intentional.
    if nargin < 1, names = {'m107_direct', '25mm', 'sam_engagement'}; end
    here = fileparts(mfilename('fullpath'));
    for i = 1:numel(names)
        baseline = runScenario(names{i}); %#ok<NASGU>
        save(fullfile(here, 'baselines', [names{i} '.mat']), 'baseline');
        fprintf('Saved baseline: %s\n', names{i});
    end
end
