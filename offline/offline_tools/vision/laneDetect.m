function [ output_args ] = laneDetect( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Writen by Trevor Decker

%TODO track object from frame to frame
%TODO make into a function 
%TODO reproject to original image
%TODO filter more
%TODO document
%TODO speedup

%TODO get a map of points built from detections 
video = VideoReader('run1_test.mp4');
nFrames = video.NumberOfFrames
assert(nFrames > 0)
frames = read(video,[1 inf]);
for frameNum = 1:2:nFrames
   thisFrame = frames(:,:,:,frameNum);
   thisFrame = imresize(thisFrame,0.25);
   figure(1);
   subplot(2,2,1)
   imshow(thisFrame)
   title('originalImg')
   realImg = thisFrame;   
   img = rgb2hsv(thisFrame);

   whiteImg = isWhiteIMG(img);
   [img4,realImg] = objects(whiteImg,realImg);
   subplot(2,2,2);
   imshow(img4);
   
   img3 = isYellowIMG(img);      
   subplot(2,2,4)
   [img4,realImg] = objects(img3,realImg);
   imshow(img4)
   subplot(2,2,3)
   imshow(realImg); 
   
   
   pause(.01)
end

%for frameNum = 1:nFrames
%    thisFrame = read(video,frameNum); %first frame only
%    imshow(thisFrame)
%    pause(.05)
%end


% for frame_num = 1:1:nFrames
% %frame_num = 1000;
% 
% 
% %nFrames = xyloObj.NumberOfFrames;
% %vidHeight = xyloObj.Height;
% %vidWidth = xyloObj     .Width;
%     file = strcat('caltech-lane-detection/src/run1/test',num2str(frame_num),'.jpg');
%     img = imread(file);
%     img0 = imresize(img,0.25);
%       realImg = img0;
%  
%       subplot(2,2,1)
%       img = rgb2hsv(img0);
%       imshow(hsv2rgb(img))
%        img2 = isWhiteIMG(img);
%     [img4,realImg] = objects(img2,realImg);
%        subplot(2,2,2)
%        imshow(img4)    
% %  
% %      
% %       img3 = isYellowIMG(img);      
% %        subplot(2,2,4)
% %      [img4,realImg] = objects(img3,realImg);
% %        imshow(img4)
% %        
% %        subplot(2,2,3)
% %        imshow(realImg);   
%   pause(.05)
%  end
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

function [newimg,realImg] = objects(img,realImg,color) 

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
        if(img(i,j) == k+1)
           thisObject(thisObject_count,:) = [i,j];
           thisObject_count = thisObject_count+1;
        end
    end
end

%[p,s,u] = polyfit(thisObject(:,1),thisObject(:,2),1);
%[p] = polyfit(thisObject(:,1),thisObject(:,2),1);
%objects(k,1) = s.normr/ids(k,1);
%objects(k,2) = p(1);

end

%objects(:,1:2)
 
objects_len = cat(2,abs(objects(:,2)-objects(:,1)), ...
abs(objects(:,5)-objects(:,4)));
%ratio = objects_len(:,1)./objects_len(:,2)
%todo rotate

        
ids = ids < 100;
 %good_ids = size(ids,1) - sum(ids);
 %removes ids which have too few points
 for i = 1:size(img,1)
     for j = 1:size(img,2)
         if(img(i,j) > 1 )
         if(ids(img(i,j),1) == 1 || ...   %removes non ided pixels 
            i < 25)% || ... %remvoes the horizon 
          %  objects(img(i,j),1) > 10) %gets rid of very noisey lines 
             img(i,j) = 0;
             newimg(i,j,1) = 0;
             newimg(i,j,2) = 0;
             newimg(i,j,3) = 0;
         else
             %set real image to the same colors
                realImg(i,j,1) = colors(mod(img(i,j)-1,size(colors,1)-1)+1,1);
                realImg(i,j,2) = colors(mod(img(i,j)-1,size(colors,1)-1)+1,2);
                realImg(i,j,3) = colors(mod(img(i,j)-1,size(colors,1)-1)+1,3);
         end
         end
     end
 end
 

 
end
