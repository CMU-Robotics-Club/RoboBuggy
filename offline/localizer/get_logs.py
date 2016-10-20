#!/usr/bin/env python

import json
import numpy as np
import scipy.io

combined = True
d = None
file = '../../../rolls_logs/2016-10-09-06-17-15/sensors_2016-10-09-06-17-15.txt'
# file = '../rolls_logs/sample.txt'
with open(file) as json_data:
    d = json.load(json_data)

if d is None:
    print('No log file found')
    exit()

messages = d['sensor_data']
map_names = ['imu_linear_no_grav', 'imu_ang_pos', 'imu_ang_vel',
             'gps', 'encoder', 'steering']
map_names_nd = np.asarray(map_names, dtype='object')

if combined:
    logs = []
    for msg in messages:
        try:
            name = msg['topicName']
            name = str(name[8:])
            ind = map_names.index(name)
        except:
            continue

        if name == 'imu_linear_no_grav':
            logs.append([
                ind, msg['timestamp'],
                msg['x'], msg['y'], msg['z'], 0, 0, 0, 0, 0, 0])
        elif name == 'imu_ang_pos':
            line = [ind, msg['timestamp']]
            R = msg['rot'][0]
            R.extend(msg['rot'][1])
            R.extend(msg['rot'][2])
            line.extend(R)
            logs.append(line)
        elif name == 'imu_ang_vel':
            logs.append([
                ind, msg['timestamp'],
                msg['x'], msg['y'], msg['z'], 0, 0, 0, 0, 0, 0])
        elif name == 'gps':
            logs.append([
                ind, msg['timestamp'],
                msg['latitude'], msg['longitude'],
                msg['antennaAltitude'], 0, 0, 0, 0, 0, 0])
        elif name == 'encoder':
            logs.append([
                ind, msg['timestamp'],
                msg['distance'], msg['velocity'], 0, 0, 0, 0, 0, 0, 0])
        elif name == 'steering':
            logs.append([
                ind, msg['timestamp'],
                msg['angle'], 0, 0, 0, 0, 0, 0, 0, 0])

    logs = np.array(logs)
    print(logs.shape)
    vars_map = {'map_names': map_names_nd, 'logs': logs}
    scipy.io.savemat('./roll_logs_combined.mat', mdict=vars_map)

else:
    imu = []
    imu_ang_pos = []
    imu_ang_vel = []
    gps = []
    encoder = []
    steering = []

    for msg in messages:
        name = msg['topicName']
        name = name[8:]
        ind = map_names.index(name)

        if name == 'imu_linear_no_grav':
            imu.append([
                ind, msg['timestamp'],
                msg['x'], msg['y'], msg['z']])
        elif name == 'imu_ang_pos':
            line = [ind, msg['timestamp']]
            line.extend(msg['rot'])
            imu_ang_pos.append(line)
        elif name == 'imu_ang_vel':
            imu_ang_vel.append([
                ind, msg['timestamp'],
                msg['x'], msg['y'], msg['z']])
        elif name == 'gps':
            gps.append([
                ind, msg['timestamp'],
                msg['latitude'], msg['longitude']],
                msg['antennaAltitude'])
        elif name == 'encoder':
            encoder.append([
                ind, msg['timestamp'],
                msg['distance'], msg['velocity']])
        elif name == 'steering':
            steering.append([
                ind, msg['timestamp'],
                msg['angle']])

    imu = np.array(imu)
    gps = np.array(gps)
    encoder = np.array(encoder)
    steering = np.array(steering)

    vars_map = {'map_names': map_names_nd, 'imu': imu,
                'imu_ang_pos': imu_ang_pos, 'imu_ang_vel': imu_ang_vel,
                'gps': gps, 'encoder': encoder, 'steering': steering}
    scipy.io.savemat('./roll_logs_separate.mat', mdict=vars_map)
