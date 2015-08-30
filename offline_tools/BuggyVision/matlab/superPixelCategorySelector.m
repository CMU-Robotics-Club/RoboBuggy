%I is the input image to process on 
%OutI is the image which we want to superimpose data on 


function outI =  superPixelCategorySelector(I,outI,T,filterBank,wordMaps,labels,dictionary,colors)
seg = segmentIMG(I);
numSegs = max(max(seg));

outI(:,:,:)  = 0;
'segmentation done'




description = createDescriptor(I,filterBank);

%TODO speed up  createDescriptor,
% maybe we could just calulate the descriptor for parts of the image which
% we are planing to smaple 
'description calulated'
for s = 1:numSegs
    [s/numSegs s numSegs]
   mask = seg == s;
   ValidPixels = find(mask);
   if(size(ValidPixels) ~= 0)
   %an alterntive to this would be to remove or merge sections of an image
   %that are too small 
   while(size(ValidPixels,1) < T)
     %concat validPixels on to itself to allow for repeates so we have at
     %aleast T pixels to choose from
     ValidPixels = cat(1,ValidPixels,ValidPixels);
   end
      'here'

   % Random permutation of the descrip
   randind = randperm(size(ValidPixels,1));
    
   %Getting the first T responses and adding them to the selected set;
   thisDescription = description(randind(1:T),:);

   %Getting the filter responses
    DMatrix = pdist2(thisDescription,dictionary);

   %Chosing the closest one
    [wordVector,index] = min(DMatrix,[],2);   
    minDist = 100000; %TODO rewrite
    minL = -1; 
    for j = 1:size(wordMaps,1)
       d = sum((wordVector-wordMaps{j}).^2);
       if minDist > d
           minDist = d;
           minL = j;
       end
    end
    l = labels(minL);
    for x = 1:size(mask,1)
        for y = 1:size(mask,2)
            if(mask(x,y) == 1)
               outI(x,y,1) = colors(l,1);
               outI(x,y,2) = colors(l,2);
               outI(x,y,3) = colors(l,3);
            end
        end
    end
   end
end
'end'
   %todo find histogram which is closest to the description


end