function [ output_args ] = isRoad(origFrame,thisFrame,img,samplingRate)
%Given an image extract the portion which 
    lowH = 0;
   highH = 1;
   lowS = 0;
   highS = .2;
   lowV=0;
   highV=.5;
   roadPoints = floor(samplingRate*[300, 300;
                    400, 300;
                     300, 500;
                     300, 500;
                    200, 400;
                    200, 300;
                     200, 500;]);
    roadPoints
    color_values = zeros(size(roadPoints,1),3);
   for i = 1:size(roadPoints,1)
    color_values(i,:) = img(roadPoints(i,1),roadPoints(i,2),:)
           for j =-5:1:5
           for k = -5:1:5
        img(roadPoints(i,1)+j,roadPoints(i,2)+k,:) = [.5,1,1];
           end
       end
   end
%  if frameNum > 50 
    %  ColorDataDisplay(origFrame,img,lowH,highH,lowS,highS,lowV,highV);

%  else
   subplot(2,2,1)
   imshow(origFrame)
   title('originalImg')

   subplot(2,2,2)
   highS = max(color_values(:,2))+.01;
 %  high = max(color_values(:,1))+.01;
   thres = colorFilter(img,lowH,highH,lowS,highS,lowV,highV);
   imshow(thres);
%  end
     subplot(2,2,1);
   hold on
   %plot(roadPoints(:,1)/samplingRate,roadPoints(:,2)/samplingRate,'.b');   
  subplot(2,2,2)
  hold on 
  subplot(2,2,3)
  Frame = thisFrame;
  imshow(Frame);
  subplot(2,2,4)
  gray = rgb2gray(Frame);
  bw = edge(gray,'canny',.05);
  horizonOffset = 0;
  bw = bw(1:10,1:10,:)
 [newbw,realImg,width] = objects(bw,origFrame,horizonOffset,0);
  
  %newbw = zeros(size(bw,1),size(bw,2));
  %for i = 1:size(bw,1)
  %    for j = 1:size(bw,2)
  %      for k = -3:3
  %          for l = -3:3
  %             if i+k > 0 && i+k < size(bw,1) && j+l > 0 && j+l < size(bw,2) && bw(i+k,j+l) == 1 
  %                  newbw(i,j) = 1;
  %              end
  %          end
  %      end
  %   end
  %end
  imshow(newbw)

end

