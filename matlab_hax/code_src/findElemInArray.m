

function [toReturn] = findElemInArray(array, toFind)

    toReturn = -1;
%     len = cellfun(@length, array);
    len = size(array);


    for i = 1:len(1),
      if strcmp(array{i}{1}, toFind)
         toReturn = i;
         return
      end
        
    end
    
    return;
    


end