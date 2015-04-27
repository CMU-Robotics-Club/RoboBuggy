

function [isIn] = timeExistsinArray(toCompare, arr)

    isIn = false;
    lastEntrySize = size(arr);
    tocmpSize = size(toCompare);
    if lastEntrySize(2) < tocmpSize(2)
        return;
    end
    
    
    dex1 = (lastEntrySize(2) - tocmpSize(2));
    dex2 = lastEntrySize(2);
    com = arr(dex1+1:dex2);
    
    if strcmp(com, toCompare)
       isIn = true; 
    end


end