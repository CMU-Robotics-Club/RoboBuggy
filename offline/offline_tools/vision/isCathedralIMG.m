%color of cathedral
 function [newImg] = isCathedralIMG(img)
 layer1 = img(:,:,1) >= .3;
 layer2 = img(:,:,1) <= .45;
 layer3 = img(:,:,2) >= 0.0; %&& img(:,:,2) <= .15;
 layer4 = img(:,:,2) <= .1;
 layer5 = img(:,:,3) >= .65;
 layer6 = img(:,:,3) <= .9;
 newImg = layer1.*layer2.*layer3.*layer4.*layer5.*layer6;
 
 newImg2 = newImg;
 for i = 1:size(newImg,1)
     for j = 1:size(newImg,2)
         good = 1;
         for dx = -8:8
             for dy = -2:2
                 if(i+dx > 0 && i+dx <= size(newImg,1) && j +dy > 0 && j+dy <= size(newImg,2))
                     if(newImg(i+dx,j+dy) == 0)
                        newImg2(i,j) = 0;
                        good = 0;
                     end
                     
                 end
             end
         end
         if good == 1
             newImg2(i,j) = 2;
         end
     end
 end
  for i = 1:size(newImg2,1)
      for j = 1:size(newImg2,2)
         if newImg2(i,j) == 2
            newImg =  fillOnes(newImg,i,j);
         else
 %            newImg(i,j) = 0;
         end
      end
  end
 
 
 for i = 1:size(newImg2,1)
     for j = 1:size(newImg2,2)
         if newImg(i,j) == 2 
             newImg(i,j) = 1;
         else 
             newImg(i,j) = 0;
         end
     end
 end

 
 
 
% newImg = newImg2;
% newImg
% 

%  
%  newImg = newImg == 2;
% newImg = newImg2;
 %newImg = newImg2 == 2;
%  for i = 1:size(newImg2,1)
%      for j = 1:size(newImg2,2)
%         if newImg2(i,j) == 2
%            %flood fill 1's to become 2's
%            for dx = -1:1
%                for dy = -1:1
%                                      if(i+dx > 0 && i+dx <= size(newImg,1) && j +dy > 0 && j+dy <= size(newImg,2))
%                 if(newImg(i+dx,j+dy) == 1)
%                     newImg(i+dx,j+dy) = 2;
%                 end
%                 end
%                end
%            end
%         end
%      end
%  end
 
 
 end
