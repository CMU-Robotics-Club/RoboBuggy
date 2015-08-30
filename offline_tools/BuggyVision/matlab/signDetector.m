% Written by Trevor Decker
% Designed to pick out signs from an image 

%TODO train on our data 
function detectedImg = signDetector(I)
 detector = vision.CascadeObjectDetector('stopSignDetector.xml');
 bbox = step(detector, I);
 detectedImg = insertObjectAnnotation(I, 'rectangle', bbox, 'stop sign');
 imshow(detectedImg);
 pause(0.1);
end