function outI =  superPixelCategorySelectorManul(I,outI,T,filterBank,wordMaps,labels,dictionary,colors)
%I is the input image to process on 
%OutI is the image which we want to superimpose data on 
I_hsv = rgb2hsv(I);

I_range   = rangefilt(I);
I_std     = stdfilt(rgb2gray(I));
I_entropy = entropyfilt(rgb2gray(I)); 
I_std_dense = I_std(:,:)>2;
I_std_dense =  bwareaopen(I_std_dense, 50);
I_std_dense = ~I_std_dense;

h0 = 1*[-2, 0, 0, 0, 0;
          0, -1, 0, 0, 0;
          0, 0, 0, 0, 0;
          0, 0, 0, 1, 0;
          0, 0, 0, 0, 2];
h1 = 1*[ 0, 0, -2, 0, 0;
          0, 0, -1, 0, 0;
          0, 0, 0, 0, 0;
          0, 0, 1, 0, 0;
          0, 0, 2, 0, 0];
h2 = 1*[ 0, 0, 0, 0, -2;
          0, 0, 0, -1, 0;
          0, 0, 0, 0, 0;
          0, 1, 0, 0, 0;
          3, 0, 0, 0, 0];      
%s = 20;
%scaleMultiply = 1;
%h = getGaussianFilter(s*scaleMultiply);
%I = rgb2gray(I);
I0 = abs(imfilter(I,h0));
I1 = abs(imfilter(I,h1));
I2 = abs(imfilter(I,h2));
%I3 = imfilter(I,h');
I4 = I0 + I1 + I2;
I4_gray = rgb2gray(I4);
a = I4_gray > 70;
b = I4_gray < 150;
c = a.*b;
%subplot(1,2,1)
%imshow(I)
%subplot(1,2,2)
%imshow(a.*b)
%pause()


%a = I_hsv(:,:,1) <= .55;
%b = I_hsv(:,:,1) >= .25;
%imshow(a.*b)
%imshow(I_entropy(:,:)> 3)

%pause()

seg = segmentIMG(I);
numSegs = max(max(seg));

outI(:,:,:)  = 0;

'segmentation done'
   
horizon = .25*size(I_hsv,1);    

    %HayBale DETECTOR 
    hayBale1 = I_hsv(:,:,1) >= .10;
    hayBale2 = I_hsv(:,:,1) <= .3;
    hayBale3 = I_hsv(:,:,2) >= .00;
    hayBale4 = I_hsv(:,:,2) <= .2;
    hayBale5 = I_hsv(:,:,3) >= .3;
    hayBale6 = I_hsv(:,:,3) <= .6;
    hayBale = hayBale1.*hayBale2.*hayBale3.*hayBale4.*hayBale5.*hayBale6;
    %TODO add texture 
%      subplot(1,2,1)
%      imshow(I);
%      subplot(1,2,2)
%      I_h = I;%zeros(size(I,1),size(I,2));
%      for x = 1:size(I,1)
%          for y = 1:size(I,2)
%              if(hayBale(x,y) == 1)
%      I_h(x,y,1) = 255;
%      I_h(x,y,2) = 0;
%      I_h(x,y,3) = 0;
%              end
%          end
%      end
%      imshow(I_h);
%      pause()

    %SKY blue TODO add detectors for when we have clouds and/or dark
    sky1 = I_hsv(:,:,1) > .48;
    sky2 = I_hsv(:,:,1) <= .75;
    sky3 = I_hsv(:,:,2) >= 0;
    sky4 = I_hsv(:,:,2) <= 1;
    sky5 = I_hsv(:,:,3) >= .9;     %only the sky is super bright 
    sky6 = I_hsv(:,:,3) <= 1;
    sky7 = I_std_dense;
    % for not allowing the sky to be in the bottom of the frame 
    % this is kind of a hack  
    sky8 = zeros(size(I_hsv,1),size(I_hsv,2));
    sky8(1:horizon,:) = 1;
    
    %combines all of the sky filters 
    sky = sky1.*sky2.*sky3.*sky4.*sky5.*sky6.*sky7.*sky8;

    
%     k = ones(255,255,3);
%     for x = 1:255
%         for y = 1:255
%             k(x,y,2) = x/256;
%             k(x,y,3) = y/256;
%         end
%     end
%     k(:,:,1) = .6;
%    imshow(sky)
%    pause()


    
    %Yellow Line 
    yLine1 = I_hsv(:,:,1) >= .0;%.14 %TODO think of moving up
    yLine2 = I_hsv(:,:,1) <= .22;
    yLine3 = I_hsv(:,:,2) >= 0;
    yLine4 = I_hsv(:,:,2) <= 1;
    yLine5 = I_hsv(:,:,3) >= 0;
    yLine6 = I_hsv(:,:,3) <= 1;
    
    % for not allowing the road to be in the top of the frame 
    % this is kind of a hack  
    yLine7 = zeros(size(I_hsv,1),size(I_hsv,2));
    yLine7(horizon:size(I_hsv,1),:) = 1;
    
    yLine = yLine1.*yLine2.*yLine3.*yLine4.*yLine5.*yLine6.*yLine7;    
    
    %White Line
    wline1 = I_hsv(:,:,1) >= 0;
    wline2 = I_hsv(:,:,1) <= 1;
    wline3 = I_hsv(:,:,2) >= 0;
    wline4 = I_hsv(:,:,2) <= .45;
    wline5 = I_hsv(:,:,3) >= .5;
    wline6 = I_hsv(:,:,3) <= 1;
    wline7 = ~I_std_dense;
   % for not allowing the road to be in the top of the frame 
    % this is kind of a hack  
    wLine8 = zeros(size(I_hsv,1),size(I_hsv,2));
    wLine8(horizon:size(I_hsv,1),:) = 1;
    
    
    wline = wline1.*wline2.*wline3.*wline4.*wline5.*wline6.*wline7.*wLine8;
    %means if a detection is a yellow line it is not a wline 
    wline = max(wline - yLine,0);
    

   
    



    
    %ROAD
    road1 = I_hsv(:,:,1) >= 0;
    road2 = I_hsv(:,:,1) <= 1;
    road3 = I_hsv(:,:,2) >= 0;
    road4 = I_hsv(:,:,2) <= .75;
    road5 = I_hsv(:,:,3) >= 0;
    road6 = I_hsv(:,:,3) <= .5;
    road7 = I_std_dense;
    road9 = c;
    % for not allowing the road to be in the top of the frame 
    % this is kind of a hack  
    road8 = zeros(size(I_hsv,1),size(I_hsv,2));
    road8(horizon:size(I_hsv,1),:) = 1;
    
    
    %assumes that the road will not have a green tint 
     road_nG_1 = I_hsv(:,:,1) <= .48;
     road_nG_2 = I_hsv(:,:,1) >= .22;
     road_nG_3 = I_hsv(:,:,2) >= .2;
     road_nG_4 = I_hsv(:,:,3) >= .2;
     road_nG = ~(road_nG_1.*road_nG_2.*road_nG_3.*road_nG_4);

%      for s = 1:max(max(seg))
%          s
%          thisI = zeros(size(I,1),size(I,2),3);
% 
%      for x = 1:size(I,1)
%          for y = 1:size(I,2)
%              if(seg(x,y) == s)
%                  thisI(x,y,1) = I(x,y,1);
%                  thisI(x,y,2) = I(x,y,2);
%                  thisI(x,y,3) = I(x,y,3);
%              end
%          end
%      end
%      thisI = thisI./255;
%      subplot(1,2,1)
%      imshow(thisI)
%      subplot(1,2,2)
%      r = road_nG.*(seg == s);
%      imshow(r)
% pause
%      end


%    imshow(road_nG)
    
    
    road =road_nG.*road1.*road2.*road3.*road4.*road5.*road6.*road7.*road8.*road9;

        %Tree 
    tree1 = I_hsv(:,:,1) >= 0;
    tree2 = I_hsv(:,:,1) <= 1;
    tree3 = I_hsv(:,:,2) >= 0;
    tree4 = I_hsv(:,:,2) <= .75;
    tree5 = I_hsv(:,:,3) >= 0;
    tree6 = I_hsv(:,:,3) <= .5;
    tree7 = ~I_std_dense;
    
    
    tree = tree1.*tree2.*tree3.*tree4.*tree5.*tree6.*tree7;
    
 

for s = 1:numSegs
    mask = seg == s;
   
    thisSky = sky.*mask;
    skySum = sum(sum(thisSky));
    
    thisRoad = road.*mask; 
   roadSum = sum(sum(thisRoad));
    
    thisWline = wline.*mask;
    wlineSum = sum(sum(thisWline));
    
    thisTree = tree.*mask;
    treeSum = sum(sum(thisTree));
    
    thisYLine = yLine.*mask;
    yLineSum = sum(sum(thisYLine));
    
    thisHayBale = hayBale.*mask;
    hayBaleSum = sum(sum(thisHayBale));
    
    best = max([skySum,roadSum,wlineSum,treeSum,yLineSum,hayBaleSum]);
    if(best == 0)
        %do nothing 
    
    %choses what type the super pixel is 
    elseif(skySum == best)
     %  'sky'
       
    pixels = find(mask);
    for k =1:size(pixels,1)
       x = floor(pixels(k)/size(outI,1)) + 1;
       y = mod(pixels(k),size(outI,1)) + 1  ; %don't think is correct 
       outI(y,x,1) = colors(11,1);
       outI(y,x,2) = colors(11,2);
       outI(y,x,3) = colors(11,3);
    end
    
    elseif(roadSum == best)
     %   'road'    
   pixels = find(mask);
    for k =1:size(pixels,1)
       x = floor(pixels(k)/size(outI,1)) + 1;
       y = mod(pixels(k),size(outI,1)) + 1  ; %don't think is correct 
       outI(y,x,1) = colors(1,1);
       outI(y,x,2) = colors(1,2);
       outI(y,x,3) = colors(1,3);
    end
    elseif(wlineSum == best)
    %    'wline'
   pixels = find(mask);
        for k =1:size(pixels,1)
       x = floor(pixels(k)/size(outI,1)) + 1;
       y = mod(pixels(k),size(outI,1)) + 1  ; %don't think is correct 
       outI(y,x,1) = colors(2,1);
       outI(y,x,2) = colors(2,2);
       outI(y,x,3) = colors(2   ,3);
        end
    elseif(treeSum == best)
        %tree
        
   pixels = find(mask);
        for k =1:size(pixels,1)
       x = floor(pixels(k)/size(outI,1)) + 1;
       y = mod(pixels(k),size(outI,1)) + 1  ; %don't think is correct 
       outI(y,x,1) = colors(12,1);
       outI(y,x,2) = colors(12,2);
       outI(y,x,3) = colors(12   ,3);
        end
    elseif(best == hayBaleSum)
        %haybale
        for k =1:size(pixels,1)
       x = floor(pixels(k)/size(outI,1)) + 1;
       y = mod(pixels(k),size(outI,1)) + 1  ; %don't think is correct 
       outI(y,x,1) = colors(13,1);
       outI(y,x,2) = colors(13,2);
       outI(y,x,3) = colors(13   ,3);
        end   
    else
        %yline
           pixels = find(mask);
        for k =1:size(pixels,1)
       x = floor(pixels(k)/size(outI,1)) + 1;
       y = mod(pixels(k),size(outI,1)) + 1  ; %don't think is correct 
       outI(y,x,1) = colors(3,1);
       outI(y,x,2) = colors(3,2);
       outI(y,x,3) = colors(3,3);
        end
        
       
    end
        
    
    
end
%subplot(1,2,1)
%imshow(I)
%subplot(1,2,2)
%imshow(outI)


%pause

end

function h = getGaussianFilter(sigma)
    h = fspecial('gaussian',ceil(sigma*3*2+1),sigma);
end