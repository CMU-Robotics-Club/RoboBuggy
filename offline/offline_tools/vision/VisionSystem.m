function [ output_args ] = VisionSystem()
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Writen by Trevor Decker

%TODO track object from frame to frame
%TODO make into a function/classes
%TODO get position and plot it in google earth
%TODO reproject to original image
%TODO filter more
%TODO document
%TODO speedup   (mostly done should at least double when we go to java)
%TODO detect cathedral of learning

%TODO get a map of points built from detections
%video = VideoReader('run1.MP4');
%video = VideoReader('run1_c.mp4');
video = VideoReader('run1_cathedral.mp4');
nFrames = video.NumberOfFrames
fps = video.frameRate;
assert(nFrames > 0)

writerObj = VideoWriter('out.avi'); % Name it
writerObj.FrameRate = fps; % How many frames per second.
open(writerObj); 


frames = read(video,[1 inf]);
for frameNum = 1:1:nFrames
   origFrame = frames(:,:,:,frameNum);
   thisFrame = imresize(origFrame,1);%0.25);
   realImg = thisFrame;   
   figure(1);
   subplot(2,2,1)
   imshow(origFrame)
   title('originalImg')
   
   %removes pixels of the buggy 
   thisFrame(60:120,100:115,:) = 0;  %TODO make indpendent of scaling 
   
   horizonLine =100;
   aboveHorizon = thisFrame(1:horizonLine,:,:);
   bellowHorizon  = thisFrame(horizonLine:size(thisFrame,1),:,:);
   subplot(2,1,1)
   imshow(aboveHorizon);
   catheImg = isCathedralIMG(rgb2hsv(aboveHorizon));
   [img4,realImg,thisWidth] = objects(catheImg,realImg,0,300);
%   gray = rgb2gray(aboveHorizon);
%   edgeImg = edge(gray,'canny');
   subplot(2,2,3)
   imshow(realImg);
   subplot(2,2,4)
   hold on 
   plot(frameNum,thisWidth,'.r','MarkerSize',20);
   
%   subplot(4,2,2)
%   imshow(aboveHorizon);
%   title('aboveHorizon');
%   subplot(4,2,4)
%   imshow(bellowHorizon);
%   title('belowHorizon');
   
%   img = rgb2hsv(bellowHorizon);

%whiteImg = isWhiteIMG(img);
%   [img4,realImg] = objects(whiteImg,realImg,horizonLine);
%   subplot(4,2,6);
%   imshow(img4);
%   title('white features')
   
%   img3 = isYellowIMG(img);      

 %  [img4,realImg] = objects(img3,realImg,horizonLine);
 %  subplot(4,2,8)
 %  imshow(img4)
 %  title('yellow features')
 %  subplot(2,2,3)
 %  imshow(realImg); 
 %  frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
 %       writeVideo(writerObj, frame);
   
   pause(.01)
end
close(writerObj); % Saves the movie.
end

%color of cathedral
 function [newImg] = isCathedralIMG(img)
 layer1 = img(:,:,1) >= .3;
 layer2 = img(:,:,1) <= .45;
 layer3 = img(:,:,2) >= 0.0; %&& img(:,:,2) <= .15;
 layer4 = img(:,:,2) <= .1;
 layer5 = img(:,:,3) >= .65;
 layer6 = img(:,:,3) <= .9;
 newImg = layer1.*layer2.*layer3.*layer4.*layer5.*layer6;
 
 newImg2 = conv2(newImg,ones(16,5)) == 17*5;
 
 %for i = 1:size(newImg,1)
 %    for j = 1:size(newImg,2)
 %        good = 1;
 %        for dx = -8:8
 %            for dy = -2:2
 %                if(i+dx > 0 && i+dx <= size(newImg,1) && j +dy > 0 && j+dy <= size(newImg,2))
 %                    if(newImg(i+dx,j+dy) == 0)
 %                       newImg2(i,j) = 0;
 %                       good = 0;
 %                    end
 %                    
 %                end
 %            end
 %        end
 %        if good == 1
 %            newImg2(i,j) = 2;
 %        end
 %    end
 %end
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
 
 function inImg =  fillOnes(inImg,i,j)
 if(i > 0 && i < size(inImg,1) && j > 0 && j < size(inImg,2))
    if(inImg(i,j) == 1)
        inImg(i,j) = 2;
        for dx = -1:2:1
            for dy = -1:2:1
                if i ~= 0 && j ~= 0 
                    inImg =  fillOnes(inImg,i+dx,j+dy);
                end
            end
        end
    end
 end
 
 end



 function [newImg] = isWhiteIMG(img)
 layer1 = img(:,:,2) >= 0; %&& img(:,:,2) <= .15;
 layer2 = img(:,:,2) <= .15;
 layer3 = img(:,:,3) >= .65;
 layer4 = img(:,:,3) <= 1;
 % && img(:,:,3) <= 1;
 newImg = layer1.*layer2.*layer3.*layer4;
 end

 function [newImg] = isYellowIMG(img)
 layer1 = img(:,:,1) >= .1; %&& img(:,:,2) <= .15;
 layer2 = img(:,:,1) <= .2;
 layer3 = img(:,:,2) >= .2;
 layer4 = img(:,:,2) <= 1;
 layer5 = img(:,:,3) >= .5;
 layer6 = img(:,:,3) <= 1;
 % && img(:,:,3) <= 1;
 newImg = layer1.*layer2.*layer3.*layer4.*layer5.*layer6;
 end
%color is an hsv
function [value] = isWhite(color)
      if(...%img(i,j,1) >= 0 && img(i,j,1) <= 1 && ... % h
                color(2) >= 0 && color(2) <= .15 ... % s 
            &&  color(3) >= .65 && color(3) <= 1)    %v
        value = 1;
      else
          value = 0;
      end
end

%color is an hsv
function [value] = isYellow(color)
    if(color(1) >= 0.1 && color(1) <= .2 ...  %h
           && color(2) >= .2 && color(2) <= 1 ...  %s 
           && color(3) >= .5 && color(3) <= 1)     %v
    value = 1;
    else
        value = 0;
    end
end

function [newimg,realImg,width] = objects(img,realImg,horizionOffset,numPointsForObject) 
width = 0;
    colors = [0,0,255;
              0,255,0;
              0,255,255;
              255,0,0;
              255,0,255;
              255,255,0;
              255,255,255];
          
          
newimg = zeros(size(img,1),size(img,2),3);
id = 2;
ids = [0];
for i = 1:size(img,1)
        for j = 1:size(img,2)
            if(img(i,j) == 1)
                changed = 0;
               %check for color next to it 
               for di = -10:10
                   for dj = -10:10   
                    if(i+di > 0 && i+di < size(img,1)  && ...
                       j+dj > 0 && j+dj < size(img,2) && ...
                       img(i+di,j+dj) > 1)
                        img(i,j) = img(i+di,j+dj);
                        ids(img(i+di,j+dj),1) = ids(img(i+di,j+dj),1) +1;
                        newimg(i,j,1) = colors(mod(img(i+di,j+dj)-1,size(colors,1)-1)+1,1);
                        newimg(i,j,2) = colors(mod(img(i+di,j+dj)-1,size(colors,1)-1)+1,2);
                        newimg(i,j,3) = colors(mod(img(i+di,j+dj)-1,size(colors,1)-1)+1,3);
                        changed = 1;
                        break;
                    end
                   end
               if(changed == 1)
                   break
               end
               end
            if(changed == 0)
                img(i,j) = id;
                ids = cat(1,ids,1);
                id = id+1;
            end
        end
    end
end


objectsMinX = size(img,2).*ones(size(ids,1),1);
objectsMaxX = zeros(size(ids,1),1);


 objects = zeros(size(ids,1),9);
%for i = 1:size(objects,1)
%    objects(i,1) = size(img,2)*size(objects,1);
%    objects(i,4) = size(img,2)*size(objects,1);
%end


for(k = 2:size(ids,1))
thisObject = zeros(ids(k),2);
thisObject_count = 1;

for i = 1:size(img,1)
    for j = 1:size(img,2)
        if(img(i,j) == k)
           if( j < objectsMinX(k))
               objectsMinX(k) = j;
           end
           if(j > objectsMaxX(k))
               objectsMaxX(k) = j;
           end
           %
           %thisObject(thisObject_count,:) = [i,j];
           %thisObject_count = thisObject_count+1;
        end
    end
end

%[p,s,u] = polyfit(thisObject(:,1),thisObject(:,2),1);
%[p] = polyfit(thisObject(:,1),thisObject(:,2),1);
%objects(k,1) = s.normr/ids(k,1);
%objects(k,2) = p(1);

end
if(size(objectsMaxX,1) > 1)
objectsWidth = objectsMaxX - objectsMinX;
width = objectsWidth(2);
end

%objects(:,1:2)
 
objects_len = cat(2,abs(objects(:,2)-objects(:,1)), ...
abs(objects(:,5)-objects(:,4)));
%ratio = objects_len(:,1)./objects_len(:,2)
%todo rotate

        
ids = ids < numPointsForObject;
 %good_ids = size(ids,1) - sum(ids);
 %removes ids which have too few points
 for i = 1:size(img,1)
     for j = 1:size(img,2)
         if(img(i,j) > 1 )
         if(ids(img(i,j),1) == 1)   %removes non ided pixels 
          %  objects(img(i,j),1) > 10) %gets rid of very noisey lines 
             img(i,j) = 0;
             newimg(i,j,1) = 0;
             newimg(i,j,2) = 0;
             newimg(i,j,3) = 0;
         else
             %set real image to the same colors
                realImg(i+horizionOffset,j,1) = colors(mod(img(i,j)-1,size(colors,1)-1)+1,1);
                realImg(i+horizionOffset,j,2) = colors(mod(img(i,j)-1,size(colors,1)-1)+1,2);
                realImg(i+horizionOffset,j,3) = colors(mod(img(i,j)-1,size(colors,1)-1)+1,3);
         end
         end
     end
 end
 

 


end

