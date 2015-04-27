classdef Map
    %Class repersenting the map of the course
    
    properties
        mapObjectList
    end
    
    methods
        function drawMap(this)
            parfor i =  1:size(mapObjectList,1)
                thisMapObject = mapObjectList{i};
                thisMapObject.draw();
            end
        end
        
        function addObjectToMap(newMapObject)
           %TODO 
        end
        
        function removeObjectFromMap(mapObjectToRemove)
            %TODO
        end
        
        
    end
    
end

