#!/usr/bin/env python

import json
import numpy as np
import scipy.io

combined = True
d = None
file = 'waypoints_course.txt'
logs = []
with open(file) as json_data:
    for line in json_data:
        d = json.loads(line)
        lat = d['latitude']
        lng = d['longitude']
        logs.append([lat, lng])


logs = np.array(logs)
print(logs.shape)
vars_map = {'logs': logs}
scipy.io.savemat('./waypoints.mat', mdict=vars_map)
