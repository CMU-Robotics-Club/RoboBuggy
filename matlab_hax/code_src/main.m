% code to take sensor data and map it out on a graph

    %initdraw
    
    global times_human times_decimal;

    filePath = '/Users/vivaanbahl/Desktop/Robobuggy/Robobuggy/matlab_hax/data_src/2014-11-23-07-23-37/sensors.txt';

    rawData = getRawData(filePath);

    rows = cellfun(@length, rawData);
    
    sensors = [];
    plotpoints = [];
    times = [];
    hasGottenStartTime = 0;

    for i = 1:2:5,
        string1 = rawData{1}{i};
        string2 = rawData{1}{i + 1};
        string = strcat(string1, '+', string2);
        
        indOfPlus = strfind(string, '+');
        startOfSensorNum = indOfPlus + 10; %isolate the sensor number
        sensorNum = string(startOfSensorNum:startOfSensorNum + 4);
        sensorTime = string(indOfPlus+1:indOfPlus+8);
        
        times_human = [ times_human ' ' sensorTime ]
        
        bool = isInArray(sensorNum, sensors);
        indexToStart = 0;
        
        if bool == -1
            %add it in
            toConcat = sensorNum;
            sensors = [sensors; toConcat];
            indexToStart = size(sensors);
            
        else
            %its already in
            indexToStart = bool;
            
        end
        
        
        
        
        
    end
    
    