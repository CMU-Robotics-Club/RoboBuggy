%TODO add read  ing of log files
%TODO add ML
%TODO consider adding blur to allow for small holes in lines
%TODO fix sampeling for antializing 
%TODO cleanup
%TODO deploy as standalone app
%TODO speed up
%TODO add back in surf and add template matching
%TODO fix mask has values greater then 1 in it wtf
function [ out ] = processImage(I)
options = {'Road','white Lane Line','Yellow Lane Line','SideWalk','other','monument','cathedral','otherbuilding','I Do not know','haey bail','sky','tree'};
wordMaps = readLogs(options);

I = imresize(I, .5);
segments = segmentIMG(I);
%TODO apply bag of words to each segment to categorise it
parfor i = 1:size(segments,1)
    mask = segments == i;
   %TODO 
end


out = I;
subplot(1,3,1)
showlabelsRandColor(segments,I);
subplot(1,3,2)
imshow(I)
subplot(1,3,3)


%randomly select a segment check to see what category it belongs with
m = max(max(segments));
pass = 0;
while(pass == 0)
    r  = randi(m);
    count = 0;
    for x = 1:size(I)
        for y = 1:size(I)
            if(segments(x,y) == r)
                out(x,y,1) = 255;
                out(x,y,2) = 0;
                out(x,y,3) = 0;
                count = count + 1;
            end
        end
    end
    display(count);
    %for somereason some sections have no elements
    if(count ~= 0)
        pass = true;
    end
end

mask = (segments == r);
imshow(out)
for i = 1:size(options,2)
    fig = gcf;
    x = mod(i,6);
    y = floor((i-1)/6);
    sth = uicontrol(fig,'Style','text',...
        'String','Beilf in this option',...
        'Position',[50+x*150 30+50*y 150 20]);
    pbh = uicontrol(fig,'Style','pushbutton','String',options(i),...
        'Position',[50+x*150 50+50*y 150 20], ...
        'Callback',{@saveData,I,mask,options(i)});
end
uiwait()

end

function saveData(hObject,callback_data,I,mask,option)
train_folder ='training_Log/';

% save to training_Log
val = clock;
clock_str = strcat(num2str(val(1)),num2str(val(2)),num2str(val(3)),num2str(val(4)),num2str(val(5)),num2str(val(6)));
folder = strcat(train_folder,clock_str);
mkdir(folder);
imwrite(I,strcat(folder,'/I.jpg'));
imwrite(mask,strcat(folder,'/Mask.jpg'));
f = fopen(strcat(folder,'/result'),'wt');
fprintf(f,option{1})
fclose(f)
%resumes the program
uiresume(gcbf)
end









function out = showlabelsRandColor(labels,I)
out = I;
C = get(0,'DefaultAxesColorOrder');
for x = 1:size(labels,1)
    for y = 1:size(labels,2)
        %       out(x,y,:) = C(labels(x,y),:);
        out(x,y,:) = 255*C(mod(labels(x,y),7)+1,:);
        
    end
end
imshow(out)
end

