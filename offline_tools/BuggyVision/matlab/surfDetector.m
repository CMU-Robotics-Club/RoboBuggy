function [ I_out ] = surfDetector(I_in,I_out)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

I_in_gray = rgb2gray(I_in);
points = detectSURFFeatures(I_in_gray);
for k = 1:size(points.Location,1)
    I_out = drawCircle(I_out,points.Location(k,1),points.Location(k,2),5);
end
end

function I = drawCircle(I,y_offset,x_offset,r)
for th = 0:.1:2*pi
   x = round(x_offset+r*cos(th));
   y = round(y_offset+r*sin(th));
   if(x >0 && x <= size(I,1) && y > 0 && y <= size(I,2))
    I(x,y,1)= 255;
    I(x,y,2) = 0;
    I(x,y,3) = 0;
   end
end

end