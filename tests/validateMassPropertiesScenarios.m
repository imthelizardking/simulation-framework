function report = validateMassPropertiesScenarios(names)
%VALIDATEMASSPROPERTIESSCENARIOS Run unchanged scenarios and compare baselines.
% Baseline differences are reported, never accepted/regenerated automatically.
% An old scenario that cannot run is included as a failed row with its error.
    if nargin < 1, names = {'m107_direct','25mm','sam_engagement','Hydra70mm'}; end
    here = fileparts(mfilename('fullpath'));
    addpath(fileparts(here), here);
    rows = repmat(struct('Scenario','','Status','','Message','', ...
        'InitialMass',NaN,'FinalMass',NaN,'MinimumInertia',NaN, ...
        'HasBaseline',false,'MatchesBaseline',false),numel(names),1);
    for k = 1:numel(names)
        rows(k).Scenario = names{k};
        try
            [metrics,tel] = runScenario(names{k});
            assert(isfield(tel,'mass'),'Scenario has no shared mass telemetry.');
            assert(all(isfinite(tel.mass)) && all(tel.mass > 0),'Invalid mass history.');
            assert(all(isfinite(tel.cg(:))),'Invalid CG history.');
            assert(all(isfinite(tel.inertia(:))),'Invalid inertia history.');
            smallest = Inf;
            for i = 1:numel(tel.time)
                I = reshape(tel.inertia(i,:),3,3);
                UAVMassModel.checkTensor(I,false);
                smallest = min(smallest,min(eig(I)));
            end
            rows(k).InitialMass = tel.mass(1);
            rows(k).FinalMass = tel.mass(end);
            rows(k).MinimumInertia = smallest;
            rows(k).Status = 'mass checks passed';
            path = fullfile(here,'baselines',[names{k} '.mat']);
            rows(k).HasBaseline = isfile(path);
            if isfile(path)
                saved = load(path,'baseline');
                different = {};
                for field = fieldnames(saved.baseline)'
                    name = field{1};
                    expected = saved.baseline.(name);
                    same = isfield(metrics,name);
                    if same && isnumeric(expected)
                        actual = metrics.(name);
                        same = isequal(size(actual),size(expected)) && ...
                            all(abs(actual(:)-expected(:)) <= 1e-6 + 1e-6*abs(expected(:)));
                    elseif same
                        same = isequal(metrics.(name),expected);
                    end
                    if ~same, different{end+1} = name; end %#ok<AGROW>
                end
                rows(k).MatchesBaseline = isempty(different);
                if isempty(different)
                    rows(k).Message = 'Existing baseline matches.';
                else
                    rows(k).Message = ['Review baseline differences: ' strjoin(different,', ')];
                end
            else
                rows(k).Message = 'No saved baseline; inspect scenario outputs separately.';
            end
        catch exception
            rows(k).Status = 'failed';
            rows(k).Message = [exception.identifier ': ' exception.message];
        end
    end
    report = struct2table(rows);
    disp(report);
end
