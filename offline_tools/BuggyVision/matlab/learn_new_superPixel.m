function learn_new_superPixel(I,options)
segments = segmentIMG(I);
%randomly select a segment check to see what category it belongs with
m = max(max(segments));
pass = 0;
out =I;
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
subplot(1,2,1)
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