function [ output_args ] = ColorDataDisplay(origFrame,img,lowH,highH,lowS,highS,lowV,highV)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
   subplot(2,2,1)
   imshow(origFrame)
   title('originalImg')

   subplot(2,2,2)
   thres = colorFilter(img,lowH,highH,lowS,highS,lowV,highV);
   imshow(thres);
   subplot(2,3,4)
   hold on    
   hist(img(:,:,1));
   X = [highH,highH];
   Y = [0,200];
   plot(X,Y);
   X = [lowH,lowH];
   Y = [0,200];
   plot(X,Y);   
   subplot(2,3,5)
   hold on    
   hist(img(:,:,2))
   X = [highS,highS];
   Y = [0,200];
   plot(X,Y);
   X = [lowS,lowS];
   Y = [0,200];
   plot(X,Y);      
   subplot(2,3,6)
   hist(img(:,:,3))
   hold on    
   X = [highV,highV];
   Y = [0,200];
   plot(X,Y);
   X = [lowV,lowV];
   Y = [0,200];
   plot(X,Y);   


end

