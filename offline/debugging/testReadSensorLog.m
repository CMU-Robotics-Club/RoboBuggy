function testReadSensorLog
fileId = fopen('sensors.csv','r')
tline = fgetl(fileId);
%removes header
tline = tline(228:length(tline));
%time = tline(1:13)
%tline(1) == ','
%tline = tline(14:length(tline))

i = 1;
imu_data = [];
encoder_data = [];
servo_angle_command_data = [];
servo_angle_actual_data = [];
gps_data = [];

startTime = -1;
%for i = 1:10
while(size(tline,2) > 0)
[Timestamp,IMU_acc_x,IMU_acc_y,IMU_acc_z ...
         ,IMU_gyro_x,IMU_gyro_y,IMU_gyro_z,IMU_compass_x, ...
         IMU_compass_y,IMU_compass_z,Encoder_ticks_total,Encoder_time, ...
         Servo_angle_command,Servo_angle_Actual,GPS_lon,GPS_lat,tline] = processReading(tline);
     [i, size(tline)] %for detecting progress
     i = i+1;
     if(strcmp(Timestamp,''))
         assert(0);
         %skip this loop
     else
     Timestamp = str2num(Timestamp)
     if(startTime == -1)
         startTime = Timestamp;
         Timestamp = 0;
     else
         Timestamp = (Timestamp - startTime)/1000;
     end
     imu_data = parseIMU(Timestamp,IMU_acc_x,IMU_acc_y,IMU_acc_z ...
         ,IMU_gyro_x,IMU_gyro_y,IMU_gyro_z,IMU_compass_x, ...
         IMU_compass_y,IMU_compass_z,imu_data);
     
     encoder_data = parseEncoder(Timestamp,Encoder_ticks_total, ... 
         Encoder_time,encoder_data);
     
    servo_angle_command_data = parseServo_angle_command(Timestamp,Servo_angle_command,servo_angle_command_data);
    servo_angle_actual_data = parseServo_angle_actual_data(Timestamp,Servo_angle_Actual,servo_angle_actual_data);
    gps_data = parseGPS_data(Timestamp,GPS_lon,GPS_lat,gps_data);
     end
end
clf
hold on
if(size(imu_data,2) > 0)
    subplot(2,3,1)
    scatter(imu_data(:,1),imu_data(:,7),'b')
end

if(size(encoder_data,2) > 0)
    subplot(2,3,2)
    hold on
    scatter(encoder_data(:,1),encoder_data(:,2),'r')  
    scatter(encoder_data(:,1),encoder_data(:,3),'b')  
end

if(size(servo_angle_command_data,2) > 0)
    subplot(2,3,3)
    scatter(servo_angle_command_data(:,1),servo_angle_command_data(:,2),'g')
end

if(size(servo_angle_actual_data,2) > 0)
    subplot(2,3,4)
    scatter(servo_angle_actual_data(:,1),servo_angle_actual_data(:,2),'-r')
end

if(size(gps_data,2)>0)
    subplot(2,3,5);
    scatter(gps_data(:,1),gps_data(:,2),'-r')
    scatter(gps_data(:,1),gps_data(:,3),'-b')
end

fclose(fileId);
end

function gps_data = parseGPS_data(Timestamp,GPS_lon,GPS_lat,gps_data)
    if(~strcmp(GPS_lon,''))
        GPS_lon = str2num(GPS_lon);
        GPS_lat = str2num(GPS_lat);
        newData = [Timestamp,GPS_lon,GPS_lat];
        gps_data = cat(1,gps_data,newData);
    end
end

function servo_angle_actual_data = parseServo_angle_actual_data(Timestamp,Servo_angle_Actual,servo_angle_actual_data)
    if(~strcmp(Servo_angle_Actual,''))
        Servo_angle_Actual = str2num(Servo_angle_Actual);
        newData = [Timestamp,Servo_angle_Actual];
        servo_angle_actual_data = cat(1,servo_angle_actual_data,newData);
    end
end

function servo_angle_command_data = parseServo_angle_command(Timestamp,Servo_angle_command,servo_angle_command_data)
    if(~strcmp(Servo_angle_command,''))
        Servo_angle_command = str2num(Servo_angle_command);
        newData = [Timestamp,Servo_angle_command];
        servo_angle_command_data = cat(1,newData,servo_angle_command_data);
    end
end

function encoder_data = parseEncoder(Timestamp,Encoder_ticks_total,Encoder_time,encoder_data)
      if(~strcmp(Encoder_ticks_total,''))
         %we should have valid encoder data
         Encoder_ticks_total = str2num(Encoder_ticks_total);
         Encoder_time = str2num(Encoder_time);
         
         new_data = [Timestamp,Encoder_ticks_total,Encoder_time];
         encoder_data = cat(1,encoder_data,new_data);
          
      end
end

function imu_data = parseIMU(Timestamp,IMU_acc_x,IMU_acc_y,IMU_acc_z ...
         ,IMU_gyro_x,IMU_gyro_y,IMU_gyro_z,IMU_compass_x, ...
         IMU_compass_y,IMU_compass_z,imu_data)

     
    if(~strcmp(IMU_acc_x,''))
        IMU_acc_x = str2num(IMU_acc_x);
        IMU_acc_y = str2num(IMU_acc_y);
        IMU_acc_z = str2num(IMU_acc_z);
        IMU_gyro_x = str2num(IMU_gyro_x);
        IMU_gyro_y = str2num(IMU_gyro_y);
        IMU_gyro_z = str2num(IMU_gyro_z);
        IMU_compass_x = str2num(IMU_compass_x);
        IMU_compass_y = str2num(IMU_compass_y);
        IMU_compass_z = str2num(IMU_compass_z);
        new_data = [Timestamp,IMU_acc_x,IMU_acc_y,IMU_acc_z,IMU_gyro_x,IMU_gyro_y,IMU_gyro_z,IMU_compass_x,IMU_compass_y,IMU_compass_z];
        imu_data = cat(1,imu_data,new_data);
        size(imu_data)
    end
end



function [Timestamp,IMU_acc_x,IMU_acc_y,IMU_acc_z ...
         ,IMU_gyro_x,IMU_gyro_y,IMU_gyro_z,IMU_compass_x, ...
         IMU_compass_y,IMU_compass_z,Encoder_ticks_total,Encoder_time, ...
         Servo_angle_command,Servo_angle_Actual,GPS_lon,gps_lat,tline] = processReading(tline)
[Timestamp,tline] = getComma(tline);
[IMU_acc_x,tline] = getComma(tline);
[IMU_acc_y,tline] = getComma(tline);
[IMU_acc_z,tline] = getComma(tline);
[IMU_gyro_x,tline] = getComma(tline);
[IMU_gyro_y,tline] = getComma(tline);
[IMU_gyro_z,tline] = getComma(tline);
[IMU_compass_x,tline] = getComma(tline);
[IMU_compass_y,tline] = getComma(tline);
[IMU_compass_z,tline] = getComma(tline);
[Encoder_ticks_total,tline] = getComma(tline);
[Encoder_time,tline] = getComma(tline);
[Servo_angle_command,tline] = getComma(tline);
[Servo_angle_Actual,tline] = getComma(tline);
[GPS_lon,tline] = getComma(tline);
[gps_lat,tline] = getComma(tline);
while(length(tline) > 4 && ~strcmp(tline(1:4),'1414'))
[other,tline] = getComma(tline);
end


end

function [value,remander] =  getComma(tline)
%get the next column
i = 1;
while i < length(tline)
   if (tline(i) == ',')
       break;
   end
   i = i+1;
end
remander = tline(i+1:length(tline));

value = tline(1:i-1);
if(length(value) > 4 && strcmp(value(1:4),'null'))
    remander = strcat(value(5:length(value)),',',remander);
    value = value(1:4);
end
if strcmp(value,'null')
    value = '';
end
end
