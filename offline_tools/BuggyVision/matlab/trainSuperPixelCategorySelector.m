function [wordMaps,labels,dictionary] =  trainSuperPixelCategorySelector(filterBank,options,T,k)
    train_folder ='training_Log/';
    d = dir(train_folder);
    isub = [d(:).isdir]; %# returns logical vector
    nameFolds = {d(isub).name}';
    %folder 1 is . and folder 2 is ..
    foldNum = size(nameFolds,1)-3;
    labels = zeros(foldNum,1);
    responses = cell(foldNum,1);
    foldNum
    parfor i = 1:foldNum
        i
        I_src = [train_folder,nameFolds{i+2},'/I.jpg'];
        mask_src = [train_folder,nameFolds{i+2},'/Mask.jpg'];
        result_src = [train_folder,nameFolds{i+2},'/result'];
        I = imread(I_src);
        mask = imread(mask_src);
        result = textread(result_src,'%s');
        description = createDescriptor(I,filterBank);
        %description is a Pxd where P is the number of pixels in the image and
        %d is the number of descriptors
    
        %select at random the T pixels we want to use to describe this image 
        %The T pixels need to be in the maks for this section if not enougth
        %unique pixels exsits then resample from already selected pixels 
        mask = mask./mask;
    
        % get enumeration of pixels in mask 
        ValidPixels = find(mask);

        %an alterntive to this would be to remove or merge sections of an image
        %that are too small 
        while(size(ValidPixels,1) < T)
            %concat validPixels on to itself to allow for repeates so we have at
            %aleast T pixels to choose from
            ValidPixels = cat(1,ValidPixels,ValidPixels);
        end
    
        % Random permutation of the descrip
         randind = randperm(size(ValidPixels,1));
    
        %Getting the first T responses and adding them to the selected set;
        descriptions{i} = description(randind(1:T),:);
    
        label = -1;
        for j = 1:size(options,2)
            if(strcmp(options{j},result))
                 label = j; 
            end
        end
    
        %if assert fails most likely a spelling error   
        %assert(label ~= -1)
        if(label == -1)
         label = 9;
        end
        labels(i) = label;
    end
    responses = vertcat(descriptions{:});

    %create dictionary
    [idx,dictionary] = kmeans(responses,k,'Display','final');

    %dictionary should be number of size kxnumber of descriptors
    
    %now create bag of words
    wordMaps = cell(foldNum,1);
    parfor i = 1:foldNum
    
        %Getting the filter responses
        DMatrix = pdist2(descriptions{i},dictionary);

        %Chosing the closest one
        [wordVector,index] = min(DMatrix,[],2);

        %Building the output matrix
        wordmap = vec2mat(wordVector,T)'; 
        wordMaps{i} = wordmap;
    end

'done with category training'
end