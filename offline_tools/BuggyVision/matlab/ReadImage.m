function [ output_args ] = ReadImage( input_args )
%TODO remove the 1 seg which is an outline instead of being split and
%asigned properly

%have retrain button so that the system will only retrain when
%CategoryML.mat is non exsistent or the button is pressed 

%TODO speed up, a lot 

%TODO a lot of training 

run('lib2/vlfeat-0.9.19/toolbox/vl_setup')
%TODO train ML stuff 
options = {'Road','white Lane Line','Yellow Lane Line','SideWalk','other','monument','cathedral','otherbuilding','I Do not know','haey bail','sky','tree'};
colors = [128,128,128; %gray
          255,255,255; %white
          255,255,  0; %yellow
          255,  0,  0; %red
          100,  0,  0; %gray Red
            0,100,  0; %gray green
            0,  0,100; %gray green
          100,100,  0; %gray yelow
            0,  0,  0; %black
            0,  0,  0; %black
            0,  0,255; %blue
            0,255,  0; %green
          255,165,  0;%orange
            ];
         
filterBank = createFilterBank();
T = 100;%number of pixels to sample per segment 
k = 60; %k must be greater then the size of the data set we are training with 
if(~exist('CategoryML.mat'))
    %so we do not need to retrain every time 
    [wordMaps,labels,dictionary] = trainSuperPixelCategorySelector(filterBank,options,T,k);
    save('CategoryML.mat','wordMaps','labels','dictionary');
end
load('CategoryML.mat');
%testI = imread('training_Log/20141227306.3963/I.jpg');


%trainSignDetector();



%start = 6000;
%start = 4500;
start = 6500;
numFrames_to_get = 2;

movObj = VideoReader('run1-cam0.avi');
step = 1;

% Read in all video frames.
vidFrames = read(movObj,[start,start+numFrames_to_get]);

% Get the number of frames.
numFrames = get(movObj, 'NumberOfFrames');

% creates a file for all of the logs
mkdir('training_Log')

% Create a MATLAB movie struct from the video frames.
for k = 1:numFrames_to_get %: numFrames
    I = vidFrames(:,:,:,k);
  
        outI = I;  %outI is the image which is saved for every frame 
    if mod(k,step) == 0
     %   learn_new_superPixel(I,options);
  %    outI = superPixelCategorySelectorManul(I,outI,T,filterBank,wordMaps,labels,dictionary,colors); 
   outI = superPixelCategorySelector(I,outI,T,filterBank,wordMaps,labels,dictionary,colors);  %Is the slow part
  %      'outI'
   %    outI = surfDetector(I,outI);
        subplot(1,2,1)
        imshow(outI)
        subplot(1,2,2)
        imshow(I)
        pause(.01);
%        outI = signDetector(I,outI);
%        outI = SuperPixelCategorySelector(I,outI);
        %TODO Visual Odom 
    end
    mov(k).cdata = outI;
    mov(k).colormap = [];
end
 
% Create a figure
hf = figure; 
       
% Resize figure based on the video's width and height
set(hf, 'position', [150 150 movObj.Width movObj.Height])

% Playback movie once at the video's frame rate
movie(hf, mov, 1, movObj.FrameRate);
movie2avi(mov, 'out.avi')
end

