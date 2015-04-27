function newImg =  colorFilter(img,lowH,highH,lowS,highS,lowV,highV)
    layer1 = img(:,:,1) >= lowH;
    layer2 = img(:,:,1) <= highH;
    layer3 = img(:,:,2) >= lowS;
    layer4 = img(:,:,2) <= highS;
    layer5 = img(:,:,3) >= lowV;
    layer6 = img(:,:,3) <= highV;
    newImg = layer1.*layer2.*layer3.*layer4.*layer5.*layer6;
    
end