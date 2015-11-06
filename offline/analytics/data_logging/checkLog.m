function [ output_args ] = untitled2( input_args )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
fileID = fopen('sensors.txt','r');
line = fgetl(fileID)
encoder= 0;
steering = 0;
gps = 0;
imu = 0;
other = 0;
while line ~= -1
    C = strsplit(line,',');
    if(strcmp(C(1),'sensors/steering'))
        steering = steering +1;
    elseif(strcmp(C(1),'sensors/encoder'))
        encoder = encoder +1;
    elseif(strcmp(C(1),'sensors/gps'))
        gps = gps+1;
    elseif(strcmp(C(1),'sensors/imu'))
        imu = imu+1;
    else
        other = other + 1;
    end
    line = fgetl(fileID);
    fprintf( 'Steering: %d \t gps: %d \t imu: %d \t other: %d \n',steering,gps,imu,other )
end
fclose(fileID)
[steering,gps,imu,other]
end

%note will have a bug at midnight 
function result = parseTime(input)
s = strsplit(input{1},' ');
date = s(1);
time = strsplit(s{2},':')

result = 3600*str2num(time{1})+60*str2num(time{2})+str2num(time{3});

end



