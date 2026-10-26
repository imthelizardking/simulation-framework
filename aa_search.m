% bulk_search.m
% Searches all .m files for a keyword with options for exact word and case sensitivity.

% --- Configuration ---
folderPath = pwd; 
searchWord = 'Atmosphere';

% --- Control Variables ---
matchWholeWord = true;  % true = exact word only, false = includes (partial match)
matchCase      = true;  % true = case-sensitive, false = case-insensitive
% -------------------------

% Build the regex pattern based on the whole-word control variable
if matchWholeWord
    searchPattern = ['\<', searchWord, '\>'];
else
    searchPattern = searchWord;
end

% Determine the regex function based on the case-sensitivity control variable
if matchCase
    regexFunc = @regexp;   % Case-sensitive search
else
    regexFunc = @regexpi;  % Case-insensitive search (ignores caps)
end

files = dir(fullfile(folderPath, '**', '*.m'));
foundFileCount = 0;

fprintf('Searching for "%s" (Whole Word: %s | Case Sensitive: %s)...\n', ...
    searchWord, mat2str(matchWholeWord), mat2str(matchCase));

for i = 1:length(files)
    filePath = fullfile(files(i).folder, files(i).name);
    
    try
        % Read the file and split it into lines
        fileContent = fileread(filePath);
        lines = splitlines(fileContent);
        
        fileHasMatch = false;
        
        % Check each line
        for lineNum = 1:length(lines)
            % Execute the search using the chosen rules
            if ~isempty(regexFunc(lines{lineNum}, searchPattern, 'once'))
                
                % Print the filename only once per file
                if ~fileHasMatch
                    fprintf('\n--- %s ---\n', files(i).name);
                    fileHasMatch = true;
                    foundFileCount = foundFileCount + 1;
                end
                
                % Print the line number and the code snippet
                fprintf('  Line %d: %s\n', lineNum, strtrim(lines{lineNum}));
            end
        end
    catch ME
        warning('Error reading %s: %s', files(i).name, ME.message);
    end
end

fprintf('\nSearch complete! Found in %d files.\n', foundFileCount);