
function [newimg,realImg,width] = objects(img,realImg,horizionOffset,numPointsForObject) 
newimg = 0;
realImg = 0;
width = 0;

width = 0;
    colors = [0,0,255;
              0,255,0;
              0,255,255;
              255,0,0;
              255,0,255;
              255,255,0;
              255,255,255];
          
          
newimg = zeros(size(img,1),size(img,2));
ids = [];
minWidth = 0;
minHeight = 0;
for pixelnum = 0:size(img,1)*size(img,2)-1
    %runs every pixel in parllel, each pixel is calculated from total
    %number of pixels to allow for parfor useage 
       i = floor((pixelnum)/size(img,2))+1;
       j = mod(pixelnum,size(img,2))+1;
            %if the pixel is valid then 
            if(isValid(img,i,j))
                id = pixelnum;
                %newimg(i,j) = id;
                ids = cat(1,ids,[id,i,j]);
            end
end
a = ids




%newimg

% 
% objectsMinX = size(img,2).*ones(size(ids,1),1);
% objectsMaxX = zeros(size(ids,1),1);
% 
% 
%  objects = zeros(size(ids,1),9);
%  %for i = 1:size(objects,1)
% %    objects(i,1) = size(img,2)*size(objects,1);
% %    objects(i,4) = size(img,2)*size(objects,1);
% %end
% 
% 
% for(k = 2:size(ids,1))
% thisObject = zeros(ids(k),2);
% thisObject_count = 1;
% 
% for i = 1:size(img,1)
%     for j = 1:size(img,2)
%         if(img(i,j) == k)
%            if( j < objectsMinX(k))
%                objectsMinX(k) = j;
%            end
%            if(j > objectsMaxX(k))
%                objectsMaxX(k) = j;
%            end
%            %
%            %thisObject(thisObject_count,:) = [i,j];
%            %thisObject_count = thisObject_count+1;
%         end
%     end
% end
% 
% %[p,s,u] = polyfit(thisObject(:,1),thisObject(:,2),1);
% %[p] = polyfit(thisObject(:,1),thisObject(:,2),1);
% %objects(k,1) = s.normr/ids(k,1);
% %objects(k,2) = p(1);
% 
% end
% if(size(objectsMaxX,1) > 1)
% objectsWidth = objectsMaxX - objectsMinX;
% width = objectsWidth(2);
% end
% 
% %objects(:,1:2)
%  
% objects_len = cat(2,abs(objects(:,2)-objects(:,1)), ...
% abs(objects(:,5)-objects(:,4)));
% %ratio = objects_len(:,1)./objects_len(:,2)
% %todo rotate
% 
%         
% ids = ids < numPointsForObject;
%  %good_ids = size(ids,1) - sum(ids);
%  %removes ids which have too few points
%  for i = 1:size(img,1)
%      for j = 1:size(img,2)
%          if(img(i,j) > 1 )
%          if(ids(img(i,j),1) == 1)   %removes non ided pixels 
%           %  objects(img(i,j),1) > 10) %gets rid of very noisey lines 
%              img(i,j) = 0;
%              newimg(i,j,1) = 0;
%              newimg(i,j,2) = 0;
%              newimg(i,j,3) = 0;
%          else
%              %set real image to the same colors
%                 realImg(i+horizionOffset,j,1) = colors(mod(img(i,j)-1,size(colors,1)-1)+1,1);
%                 realImg(i+horizionOffset,j,2) = colors(mod(img(i,j)-1,size(colors,1)-1)+1,2);
%                 realImg(i+horizionOffset,j,3) = colors(mod(img(i,j)-1,size(colors,1)-1)+1,3);
%          end
%          end
%      end
%  end
%  

 


end

%TODO make a paramter of objects 
%evaluates to a boolean 
function result = isValid(img,i,j)
    result = img(i,j) == 1;
%            if(img(i,j) == 1)
                
%                 changed = 0;
%                %check for color next to it 
%                for di = -minWidth:minWidth
%                    for dj = -minHeight:minHeight   
%                     if(i+di > 0 && i+di < size(img,1)  && ...
%                        j+dj > 0 && j+dj < size(img,2) && ...
%                        img(i+di,j+dj) > 1)
%                         img(i,j) = img(i+di,j+dj);
%                         ids(img(i+di,j+dj),1) = ids(img(i+di,j+dj),1) +1;
%                         newimg(i,j,1) = colors(mod(img(i+di,j+dj)-1,size(colors,1)-1)+1,1);
%                         newimg(i,j,2) = colors(mod(img(i+di,j+dj)-1,size(colors,1)-1)+1,2);
%                         newimg(i,j,3) = colors(mod(img(i+di,j+dj)-1,size(colors,1)-1)+1,3);
%                         changed = 1;
%                         break;
%                     end
%                    end
%                if(changed == 1)
%                    break
%                end
%                end
%             if(changed == 0)
%                 img(i,j) = id;
%                 ids = cat(1,ids,1);
%                 id = id+1;
%             end
end


%helper function for comparing two pixels
function result = equivlent(img1,i1,j1,img2,i2,j2)
            valid1 = isValid(img1,i1,j1);
            valid2 = isValid(img2,i2,j2);
            
            result = valid1 == valid2 && abs(i2 - i1) < 2 && abs(j2-j1) < 2;
end
