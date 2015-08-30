%assumes m starts at 2 1 is for boundires
function I = labelFloodFill(I,m,x,y)
Frontier_head = dlnode(x,y);
I(x,y) = -2;
listSize = 1;
while(listSize > 0)
    [this_x,this_y] = getData(Frontier_head);
    %-2 is will visit 
    %-1 is not assigned and not in visit list 
    if(I(this_x,this_y) == -2)
        I(this_x,this_y) = m;
        if(I(this_x+1,this_y) == -1)
         I(this_x+1,this_y) = -2;
         A = dlnode(this_x+1,this_y);
         insertAfter(A,Frontier_head);
         listSize = listSize + 1;
        end
        
        if(I(this_x,this_y+1) == -1)
         I(this_x,this_y+1) = -2;
         B = dlnode(this_x,this_y+1);
         insertAfter(B,Frontier_head);
         listSize = listSize + 1;
        end
        
        if(I(this_x,this_y-1) == -1)
         I(this_x,this_y-1) = -2;
         C = dlnode(this_x,this_y-1);
         insertAfter(C,Frontier_head);
         listSize = listSize + 1;
        end        

        if(I(this_x-1,this_y) == -1)
         I(this_x-1,this_y) = -2;
         B = dlnode(this_x-1,this_y);
         insertAfter(B,Frontier_head);
         listSize = listSize + 1;
        end
    end
    listSize = listSize - 1;
    old_Frontier_head = Frontier_head;
    Frontier_head = Frontier_head.Next;
    removeNode(old_Frontier_head);
end

end