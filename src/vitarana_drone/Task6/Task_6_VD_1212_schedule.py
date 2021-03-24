#!/usr/bin/env python

# Importing the required libraries
from std_msgs.msg import String
import rospy
import numpy as np
import pandas as pd
import os
import math
import csv

class Scheduling():

    def __init__(self):
        rospy.init_node('node_schedule')

        # Reading manifest
        self.file = pd.read_csv(
            os.path.expanduser('~/delivery_drone/src/vitarana_drone/Task6/original.csv'),
            delimiter=",",
            engine="python",
            header=None)
        self.manifest = [list(row) for row in self.file.values]
        # Hash Table to access strings of manifest easily to rewrite new manifest
        self.hash = dict(enumerate(self.manifest))
        # Creating list of coordinates
        for i, j in enumerate(self.manifest):
            self.manifest[i] = [y for x in j for y in x.split(';')]
        # GPS coordinate between the A1 grid box of delivery pad and X1 grid box of returns pad
        self.WH = np.array([18.999873523, 72.000142461, 16.757981])
        self.deliveries = []
        self.returns = []
        self.paired = []

        # Number of deliveries and returns
        self.length_d = 0
        self.length_r = 0

    def lat_to_x(self, input_latitude):
        '''
        Purpose:
        ---
        For converting latitude to X in XY coordinate system

        Input Arguments:
        ---
        input_latitude :  [ float ]
            Latitude that is to be converted to X coordinate

        Returns:
        ---
        No specific name :  [ float ] 
            converted X coordinate

        Example call:
        ---
        self.lat_to_x(19.0)
        '''
        return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
        '''
        Purpose:
        ---
        For converting longitude to Y in XY coordinate system

        Input Arguments:
        ---
        input_longitude :  [ float ]
            Longitude that is to be converted to Y coordinate

        Returns:
        ---
        No specific name :  [ float ] 
            converted Y coordinate

        Example call:
        ---
        self.long_to_y(72.0)
        '''
        return -105292.0089353767 * (input_longitude - 72)

    def schedule_plan(self):
        '''
        Purpose:
        ---
        For scheduling deliveries and returns for the drone

        Input Arguments:
        ---
        None

        Returns:
        ---
        Does not return value but changes a member variable

        Example call:
        ---
        self.schedule_plan(output)
        '''
        # Warehouse location
        x1 = self.lat_to_x(self.WH[0])
        y1 = self.long_to_y(self.WH[1])

        # Making lists of delivery and return with index from original
        # manifest, distance from warehouse and x, y coordinates
        for i, m in enumerate(self.manifest):
            if m[0] == "DELIVERY":
                x2 = self.lat_to_x(float(m[2]))
                y2 = self.long_to_y(float(m[3]))
                dist = math.hypot((x1 - x2), (y1 - y2))
                self.deliveries.append([i, dist, x2, y2])
            else:
                x2 = self.lat_to_x(float(m[1]))
                y2 = self.long_to_y(float(m[2]))
                dist = math.hypot((x1 - x2), (y1 - y2))
                self.returns.append([i, dist, x2, y2])

        # Sorting lists according to distances in descending order
        self.deliveries.sort(reverse=True, key=lambda x: x[1])
        self.returns.sort(reverse=True, key=lambda x: x[1])
        self.length_d = len(self.deliveries)
        self.length_r = len(self.returns)

        # Pairing deliveries with closest returns
        for d in self.deliveries:
            closest_dist = 1000
            index = 0
            closest_ret = 0
            for i, r in enumerate(self.returns):
                dist = math.hypot((r[2] - d[2]), (r[3] - d[3]))
                if dist < closest_dist:
                    closest_dist = dist
                    index = i
                    closest_ret = r[0]
            # Pair found
            if closest_dist < 1000:
                self.returns.remove(self.returns[index])
                self.paired.append([d[0], closest_ret])
            # No pair
            else:
                self.paired.append([d[0], -1])

        # Rewriting sequenced manifest
        with open(os.path.expanduser(
                '~/catkin_ws/src/vitarana_drone/scripts/sequenced_manifest.csv'), 'w') as f:
            w = csv.writer(f)
            for i in self.paired:
                w.writerow(self.hash[i[0]])
                if(i[1] > -1):
                    w.writerow(self.hash[i[1]])
            if self.length_d < self.length_r:
                for i in self.returns:
                    w.writerow(self.hash[i[0]])

# Function Name:    main (built in)
#        Inputs:    None
#       Outputs:    None
#       Purpose:    To call the schedule_plan() function
if __name__ == '__main__':
    s = Scheduling()
    s.schedule_plan()
