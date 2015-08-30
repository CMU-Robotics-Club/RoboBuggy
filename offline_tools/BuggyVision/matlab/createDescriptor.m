function description = createDescriptor(I,filterBank)
%apply the filter to the image creating a (N*M)xD vector 
doubleI = double(I);
[L,a,b] = rgb2lab(doubleI(:,:,1), doubleI(:,:,2), doubleI(:,:,3));
%Getting the image response
pixelCount = size(doubleI,1)*size(doubleI,2);
filterResponses = zeros(pixelCount, length(filterBank)*3 + 3);

% For each filter and channel, apply the filter, and vectorize
for filterI=0:length(filterBank)-1
    filter = filterBank{filterI+1};
    filterResponses(:,filterI*3+1) = reshape(imfilter(L, filter), pixelCount, 1);
    filterResponses(:,filterI*3+2) = reshape(imfilter(a, filter), pixelCount, 1);
    filterResponses(:,filterI*3+3) = reshape(imfilter(b, filter), pixelCount, 1);
end

%includes color of the image 
I_hsv = rgb2hsv(I);
filterResponses(:,size(filterResponses,2)-3) = reshape(I_hsv(:,:,1),pixelCount,1);
filterResponses(:,size(filterResponses,2)-2) = reshape(I_hsv(:,:,2), pixelCount, 1);
filterResponses(:,size(filterResponses,2)-1) = reshape(I_hsv(:,:,3), pixelCount, 1);




description = filterResponses;
end