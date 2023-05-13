function write_txt(package, file_name)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    if nargin < 2
        file_name = 'my-output1';
    end

    fileID = fopen(strcat(file_name, '.txt'), 'w');
    
    % first line
    fprintf(fileID, '%1d, ', size(package{3}, 1));
    fprintf(fileID, '%1d, ', size(package, 2) - 2);
    fprintf(fileID, strcat(file_name, '.txt'));
    
    % calibration results
    fprintf(fileID, '\n  %3.2f,   %3.2f,   %3.2f\r', package{1});
    fprintf(fileID, '\n  %3.2f,   %3.2f,   %3.2f\r', package{2});
    
    % c_expected
    for i = 3:size(package, 2)
        for j = 1:size(package{i}, 1)
            fprintf(fileID, '\n  %3.2f,   %3.2f,   %3.2f\r', package{i}(j, :));
        end
    end
    
    fclose(fileID);
end