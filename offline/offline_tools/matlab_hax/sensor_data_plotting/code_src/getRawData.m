% gets the raw data using the input string provided, as a cell array

% opens the file, reads the contents, and closes it
% @param filename - the name of the file in a string
% @return rawData - the raw data gotten from the file as a cell array
function [rawData] = getRawData(filename)

    
    fileID = fopen(filename);
    rawData = textscan(fileID, '%s');
    fclose(fileID);


end