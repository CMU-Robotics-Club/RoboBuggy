function segments =  segmentIMG(I)
I_S = im2single(I);
segments = vl_slic(I_S,50,0.05);
segments = segments + 1;
%makes segments Border

%% TODO speed up the following section 
% segmentsBorder = zeros(size(I,1),size(I,2));
% %marks borders of the current segments 
% for x = 1:size(I,1)
%    for y = 1:size(I,2)
%        if(x<2 || y<2 || x+1 > size(I,1) || y+1 > size(I,2) || ...
%                segments(x-1,y-1) ~= segments(x,y) || ...
%                segments(x,y-1) ~= segments(x,y) || ...
%                segments(x+1,y-1) ~= segments(x,y) || ...
%                segments(x-1,y) ~= segments(x,y) || ...
%                segments(x+1,y) ~= segments(x,y) || ...
%                segments(x-1,y+1) ~= segments(x,y) || ...
%                segments(x,y+1) ~= segments(x,y) || ...
%                segments(x+1,y+1) ~= segments(x,y) ...
%                )
%            segmentsBorder(x,y) = 1;
%        end
%        
%        
%    end
% end
% 'done step one'
% 
% gray = rgb2gray(I);
% h = rgb2hsv(I);
% E = edge(gray,'sobel',.01);
% E = edge(h(:,:,3),'canny',.03);
% E = bwareaopen(E, 50,8);
% 
% 
% %%mark all edges and segment boundries, merge boundries which are very close
% %%to each other, flood fill segment label
% 
% out = -1*ones(size(I,1),size(I,2));
% %%TODO merge edges when close to each other do not just add them
% for x = 1:size(out,1)
%    for y = 1:size(out,2)
%        if(E(x,y) == 1 || segmentsBorder(x,y) == 1)
%            out(x,y) = 1;
%        end
%    end
% end
% 
% 
% %%labeling step
% l = 2;
% for x = 1:size(out,1)
%    %display for progress
%    if mod(x,10) == 1
%        x/size(out,1)
%    end
%    for y = 1:size(out,2)
%        if(out(x,y) == -1)
%            %m is the max label so far
%            out = labelFloodFill(out,l,x,y);
%            l = l +1;
%        end
%    end
% end
% 
% for x =1:size(out,1)
%    for y = 1:size(out,1)
%        if(out(x,y) == 1)
%            %is border so assign it to a nehiboor
%            change = 0;
%            i = 1;
%            while change == 0
%                for dx = -1:1
%                    for dy = -1:1
%                        if(x+dx*i >=1 && x+dx*i <= size(out,1) && ...
%                                y+dy*i >= 1 && y+dx*i <= size(out,2) && ...
%                                out(x+dx*i,y+dy*i) ~= 1)
%                            out(x,y) = out(x+dx*i,y+dy*i);
%                            change = 1;
%                        end
%                    end
%                end
%            end
%        end
%    end
% end
% segments = out;
% 
% %subplot(1,2,1)
% % out = showlabelsRandColor(out,I);
% % %for overlaying images
% % for x = 1:size(E,1)
% %    for y = 1:size(E,2)
% %        if(E(x,y) == 1)
% %           out(x,y,:) = 255 ;
% %        end
% %    end
% % end

end