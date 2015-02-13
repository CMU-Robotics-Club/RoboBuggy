% parses the raw data obtained from calling getRawData


% returns one line of the file, for a given row
%@param row - a valid row within the cell array source
%@param source - the cell array to get the string from
%@return singleString - 
function truefalse = isInArray(item, array)

        for i = 1:size(array),
            
            if eq(array(i), item)
                truefalse = i;
                return;
            end
            
        end
        
        truefalse = -1;

end