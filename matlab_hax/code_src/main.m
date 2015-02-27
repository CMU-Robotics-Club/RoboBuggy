% code to take sensor data and map it out on a graph

%initdraw

% SENSOR LIST
% 1 - 15.46
% 2 - 18.43
% 3 - 21.5,
% 4 - 20.48
% 5 - 16.38
% 6 - 14.34
% 7 - 19.46
% 8 - 17.41
% 9 - 13.31
% 10 - 12.29
% 11 - 22.53
% 12 - 10.24
% 13 - 11.26
% 14 - 27.65
% 15 - 34.82
% 16 - 25.6,
% 17 - 23.55
% i refuse to log any more

%global times_human times_decimal;

filePath = '/Users/vivaanbahl/Desktop/Robobuggy/Robobuggy/matlab_hax/data_src/2014-11-23-07-23-37/sensors.txt';

rawData = getRawData(filePath);

rows = cellfun(@length, rawData);


sensorLen = 17;
timesLen = 40;
singleTimeLen = 8;

leftovers = [];
times = [];

sensors = cell(sensorLen, 1);

timeIndexAt = 1;

for x = 1:sensorLen,
    sensors{x}{1} = [];
end

for i = 1:2:101, %should be rows
    string1 = rawData{1}{i};
    string2 = rawData{1}{i + 1};
    string = strcat(string1, '+', string2);
    
    indOfPlus = strfind(string, '+');
    startOfSensorNum = indOfPlus + 10; %isolate the sensor number
    sensorNum = string(startOfSensorNum:startOfSensorNum + 4);
    sensorTime = string(indOfPlus+1:indOfPlus+8);
    
    %tempTimeDex = findElemInArray(times, sensorTime);
    %tempSensorDex = findElemInArray(sensors, sensorNum);
   
    %TODO put something in here that sees if the time is already in the
    %thing and if it is then do something
    
    
    senIndex = 1;
        switch sensorNum
            case '15.36'
                senIndex = 1;
            case '18.43'
                senIndex = 2;
            case '21.5,'
                senIndex = 3;
            case '20.48'
                senIndex = 4;
            case '16.38'
                senIndex = 5;
            case '14.34'
                senIndex = 6;
            case '19.46'
                senIndex = 7;
            case '17.41'
                senIndex = 8;
            case '13.31'
                senIndex = 9;
            case '12.29'
                senIndex = 10;
            case '22.53'
                senIndex = 11;
            case '10.24'
                senIndex = 12;
            case '11.26'
                senIndex = 13;
            case '27.65'
                senIndex = 14;
            case '34.82'
                senIndex = 15;
            case '25.6,'
                senIndex = 16;
            case '23.55'
                senIndex = 17;
            otherwise
                leftovers = [leftovers ' ' sensorNum];
                
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
        times = [ times sensorTime ]
        
    end
    
end





% figure out what times there are

sz = size(times);
timesDec = [];

for m = 1:sz(2)/singleTimeLen,
    
   
    timesDec = [ timesDec m ];
    
    
end




