function Iout = labelFill(I,label,startX,startY)
Iout = I;
frontier = [startX,startY];
visited = [];
k=0;
while(size(frontier,1) > 0)
    k= k+1
    visited = union(visited,frontier(1,:));
    if(I(frontier(1,1),frontier(1,2)) == 1)
        Iout(frontier(1,1),frontier(1,2)) = label;
        %add all nehboors that have not been visited yet to frontier
        nehboors = [];
        for x = -1:1
            for y = -1:1
                thisNehboor = [frontier(1,1) + x,frontier(1,2)+y];
                if(~ismember(thisNehboor,visited))
                    nehboors = union(nehboors,[frontier(1,1) + x,frontier(1,2)+y]);
                end
            end
        end
        frontier = union(frontier(1:size(frontier,1)-1,:),thisNehboor);
    end
end

end