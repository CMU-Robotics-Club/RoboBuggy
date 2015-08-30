% code to take sensor data and map it out on a graph


filePath = '/Users/vivaanbahl/Desktop/Robobuggy/RoboBuggy/offline/offline_tools/matlab_hax/sensor_data_plotting/data_src/2015-03-21-07-12-46/sensors.txt';

rawData = getRawData(filePath);

rows = cellfun(@length, rawData);


sensorLen = 4;
timesLen = 40;

leftovers = [];
times = [];

sensors = cell(sensorLen, 1);

timeIndexAt = 1;
singleTimeLen = 8;

for x = 1:sensorLen,
    sensors{x}{1} = [];
end

i = 1;

while i <= rows, %should be rows
    string1 = rawData{1}{i};
    
    while ~strcmp(string1(1), 's'),
       i = i + 1;
       string1 = rawData{1}{i};
    end
    
    string2 = rawData{1}{i + 1};
    string = strcat(string1, '@', string2);
    
    indOfDate = strfind(string, ',');
    indOfTime = strfind(string, '@');
    sensorName = string(1:indOfDate(1) - 1);
    sensorTime = string((indOfTime(1) + 1):(indOfTime(1) + 8));
    
    senIndex = 1;
        
    if strcmp(sensorName, 'sensors/imu')
        senIndex = 1;
    
    elseif strcmp(sensorName, 'sensors/encoder')
        senIndex = 2;
        
    elseif strcmp(sensorName, 'sensors/gps')
        senIndex = 3;
    
    elseif strcmp(sensorName, 'sensors/logging_button')
        senIndex = 4;
    
    else
        leftovers = [leftovers ' ' sensorName];
            
    end
    
 
       
    if timeExistsinArray(sensorTime, times)

    tempSen = sensors{senIndex}{1};
    tempSen(timeIndexAt - 1) = tempSen(timeIndexAt - 1) + 1;
    sensors{senIndex}{1} = tempSen;

    else

        for si = 1:sensorLen,

            sensors{si}{1} = [ sensors{si}{1} 0 ];


        end

        tempSen = sensors{senIndex}{1};
        tempSen(timeIndexAt) = 1;
        sensors{senIndex}{1} = tempSen;

        timeIndexAt = timeIndexAt + 1;
        times = [ times sensorTime ];

    end
      
    i = i + 2;

end


% figure out what times there are

sz = size(times);
timesDec = [];

for m = 1:sz(2)/singleTimeLen,
    
   
    timesDec = [ timesDec m ];
    
    
end


toPlot = [];
timsiz = size(timesDec);

for a = 1:sensorLen,
   
    for b = 1:timsiz(2),
        
        toPlot(b, a) = sensors{a}{1}(b);
        
    end
    
end

plot(toPlot);
legend('imu', 'encoder', 'gps', 'logging button');


sensors
times


