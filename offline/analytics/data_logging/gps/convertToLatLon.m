function [ output_args ] = convertToLatLon()

files = dir('data2/')
numColors = 9;
cmap = hsv(numColors);  %# Creates a 6-by-3 set of colors from the HSV colormap
lastValue = [0,0];  %for checking if value is valid 
for i = 3:size(files,1)
   if(files(i).isdir)
      filename = strcat(1,'data2/',files(i).name,'/output.txt');
      filename = filename(2:length(filename));%to get rid of start char
      filename
      data_ = csvread(filename);
      
      groundTruthFileName = strcat(1,'data2/',files(i).name,'/groundTruth.txt');
      groundTruthFileName = groundTruthFileName(2:length(groundTruthFileName));%to get rid of start char
      groundTruthData = csvread(groundTruthFileName)
      
     
     %data_ = cat(2,data_,zeros(size(data_,1),1))
     %data_ = data_(:size(data_,1),:);
     data = zeros(size(data_,1),size(data_,2));
     data(:,1) = data_(:,2);
     data(:,2) = data_(:,1);
     if i ~= 3 
         lastValue 
         [data(1,1) data(1,2)] 
         dLat = abs(data(:,1) - lastValue(2));
         dLon = abs(data(:,2) - lastValue(1));
         cat(2, dLat*1000,dLon*1000,dLat> .000001,dLon > .000001) 
         %abs(data(:,2) - lastValue(1)) > .000001,
         valid = (dLat> .000004).*(dLon > .000004);
         endZeros = 0;
         for j = 1:size(valid,1)
            if valid(j,1) == 1
                break;
            end
            endZeros = j;
         end
     data = data(1+endZeros:size(data,1),:)
     end
     
     lastValue = data_(size(data_,1),:);
     d = mod(data,.001);
     lats = data(:,2)';
     lons = data(:,1)';
      meanPose = [mean(lons),mean(lats)];
     pittsburhgLat_2_meters = 111034.61;
     pittsburghLon_2_meters = 85393.83;
%     stdPose  = [std(lats)*pittsburhgLat_2_meters,std(lons)*pittsburghLon_2_meters]
     plot(lons,lats,'.r','MarkerSize',20,'Color',cmap(mod(i,numColors-1)+1,:))
     errorLons  = (data(:,2) - groundTruthData(1)).*pittsburghLon_2_meters
     errorLats = (data(:,1) - groundTruthData(2)).*pittsburhgLat_2_meters
     if(size(data,1) > 0)
         c = cov(data);
         error_ellipse(c,meanPose)
     end
     hold on 
     pause()
   end
end
         plot_google_map

% % data = csvread('output.txt')
%  lats = data(:,1)';
%  lons = data(:,2)';
%  meanPose = [mean(lats),mean(lons)]
%  pittsburhgLat_2_meters = 111034.61;
%  pittsburghLon_2_meters = 85393.83;
%  stdPose  = [std(lats)*pittsburhgLat_2_meters,std(lons)*pittsburghLon_2_meters]
%  plot(lons,lats,'.r','MarkerSize',20) 
% % 
% % plot_google_map

end

