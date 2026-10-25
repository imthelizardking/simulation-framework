% bulk_replace_exact.m
% Replaces a keyword ONLY if it is an exact, standalone word (Match Case).
clear; clc;
% --- Configuration ---
folderPath = pwd; 
oldWord = 'Guidance_Passthrough';
newWord = 'Guidance_Passthrough';
% ---------------------

% Create the regex pattern for exact word boundary match
% This creates the string: \<BaseTarget\>
searchPattern = ['\<', oldWord, '\>']; 

% Find all .m files recursively
files = dir(fullfile(folderPath, '**', '*.m'));
modifiedCount = 0;

fprintf('Scanning for exact word "%s"...\n', oldWord);

for i = 1:length(files)
    filePath = fullfile(files(i).folder, files(i).name);
    
    % Skip this script itself so it doesn't modify its own code
    if strcmp(files(i).name, 'bulk_replace_exact.m')
        continue;
    end
    
    try
        % Read the file content
        fileContent = fileread(filePath);
        
        % Check if the EXACT word exists using regex
        if ~isempty(regexp(fileContent, searchPattern, 'once'))
            
            % Replace using regexprep
            newContent = regexprep(fileContent, searchPattern, newWord);
            
            % Write the changes back to the file
            fid = fopen(filePath, 'w');
            if fid == -1
                warning('Could not open %s for writing.', files(i).name);
                continue;
            end
            
            % Write the updated content and close
            fprintf(fid, '%s', newContent);
            fclose(fid);
            
            fprintf('Updated: %s\n', files(i).name);
            modifiedCount = modifiedCount + 1;
        end
    catch ME
        warning('Error processing %s: %s', files(i).name, ME.message);
    end
end

fprintf('\nComplete! Modified %d files.\n', modifiedCount);