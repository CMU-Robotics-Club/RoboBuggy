#!/bin/bash

#for each folder this script runs the gps post processor and saves a file in each folder 

for file in data2/* 
do
   echo "$file"
   python convertNmea.py $file/gpslog.txt $file/output.txt 
done
