#!/usr/bin/env python

import json
import numpy as np
import scipy.io

d = None
file = '../rolls_logs/2016-10-09-06-17-15/sensors_2016-10-09-06-17-15.txt'
# file = '../rolls_logs/sample.txt'
with open(file) as json_data:
    d = json.load(json_data)

if d is None:
    print('No log file found')
    exit()

messages = d['sensor_data']
for msg in messages:
    if msg['VERSION_ID'] == '':
        print(msg['VERSION_ID'])


arr = np.array([1, 2, 3])
scipy.io.savemat('./roll_logs.mat', mdict={'logs': arr})
