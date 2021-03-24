#!/usr/bin/env python

# Importing the required libraries
from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix, LaserScan
from std_msgs.msg import Float32, String
import rospy
import numpy as np
import pandas as pd
import os
from vitarana_drone.srv import Gripper
import math
import time


class Position():

    def __init__(self):
        # initializing ros node with name position_controller
        rospy.init_node('node_position_controller')

        self.spawnloc = np.array([0, 0, 0])
        self.initial_loc = np.array([0, 0, 0])
        # GPS coordinates of buildings
        self.file = pd.read_csv(
            os.path.expanduser('~/delivery_drone/src/vitarana_drone/Task5/sequenced_manifest.csv'),
            delimiter=",",
            engine="python",
            header=None)
        self.manifest = [list(row) for row in self.file.values]

        # Location of each grid square in a dictionary for easy access
        self.pickuploc = {"A1": np.array([18.9998102845,
                                          72.000142461,
                                          16.757981]),
                          "A2": np.array([18.9998102845,
                                          72.000142461 + 0.000014245,
                                          16.757981]),
                          "A3": np.array([18.9998102845,
                                          72.000142461 + 2 * 0.000014245,
                                          16.757981]),
                          "B1": np.array([18.9998102845 + 0.000013552,
                                          72.000142461,
                                          16.757981]),
                          "B2": np.array([18.9998102845 + 0.000013552,
                                          72.000142461 + 0.000014245,
                                          16.757981]),
                          "B3": np.array([18.9998102845 + 0.000013552,
                                          72.000142461 + 2 * 0.000014245,
                                          16.757981]),
                          "C1": np.array([18.9998102845 + 2 * 0.000013552,
                                          72.000142461,
                                          16.757981]),
                          "C2": np.array([18.9998102845 + 2 * 0.000013552,
                                          72.000142461 + 0.000014245,
                                          16.757981]),
                          "C3": np.array([18.9998102845 + 2 * 0.000013552,
                                          72.000142461 + 2 * 0.000014245,
                                          16.757981])}

        self.returnloc = {"X1": np.array([18.9999367615,
                                          72.000142461,
                                          16.757981]),
                          "X2": np.array([18.9999367615,
                                          72.000142461 + 0.000014245,
                                          16.757981]),
                          "X3": np.array([18.9999367615,
                                          72.000142461 + 2 * 0.000014245,
                                          16.757981]),
                          "Y1": np.array([18.9999367615 + 0.000013552,
                                          72.000142461,
                                          16.757981]),
                          "Y2": np.array([18.9999367615 + 0.000013552,
                                          72.000142461 + 0.000014245,
                                          16.757981]),
                          "Y3": np.array([18.9999367615 + 0.000013552,
                                          72.000142461 + 2 * 0.000014245,
                                          16.757981]),
                          "Z1": np.array([18.9999367615 + 2 * 0.000013552,
                                          72.000142461,
                                          16.757981]),
                          "Z2": np.array([18.9999367615 + 2 * 0.000013552,
                                          72.000142461 + 0.000014245,
                                          16.757981]),
                          "Z3": np.array([18.9999367615 + 2 * 0.000013552,
                                          72.000142461 + 2 * 0.000014245,
                                          16.757981])}

        # Numpy array for current GPS location
        self.currentloc = np.array([0.0, 0.0, 0.0])
        # Current Location of drone in XYZ coordinate system
        self.currentlocxy = np.array([0, 0, 0])
        # Drone Command message of type edrone_cmd and initialisation
        self.setpoint_rpy = edrone_cmd()
        self.setpoint_rpy.rcRoll = 0
        self.setpoint_rpy.rcPitch = 0
        self.setpoint_rpy.rcYaw = 0
        self.setpoint_rpy.rcThrottle = 0

        # Parameters required for the search pattern
        self.side = 0
        self.iterator = 0
        self.building_flag = 0
        # For intial spawn location only
        self.flag = 0
        # For declaring initial start and end points only once
        self.flag_once = 0
        # For 2 detections of marker
        self.detection_count = 0

        # Parameters required for PID
        self.Kp = np.array([325, 325, 225]) * 0.6
        self.Ki = np.array([4, 4, 4]) * 0.008
        self.Kd = np.array([1825, 1825, 365]) * 0.3
        self.error = np.array([0, 0, 0])
        self.prev_values = np.array([0.0, 0.0, 0.0])
        self.integral = np.array([0.0, 0.0, 0.0])
        self.max_value = 2000
        self.min_value = 1000
        self.sample_rate = 10  # in Hz
        self.sample_time = 0.1  # in seconds

        # Flag to confirm detection
        self.detectconf = False
        # Info for detection
        self.detectedcoord = [0.0, "0.0", "0.0"]
        # Flag to switch state from building seek to detection
        self.start_detection_flag = 0
        # Flag to consider only one detection of marker
        self.delivery_flag = 0
        # Waypoint variables
        self.waypoint = np.array([0, 0, 0])
        self.start = np.array([0, 0, 0])
        self.end = np.array([0, 0, 0])
        # Other variables needed to generate waypoints
        self.dt = 0
        self.t = 0
        # Flag to check whether drone is in avoid state or goal seeking state
        self.avoid_flag = 0
        # Height from bottom range finder
        self.height = 0

        # Ranges from range finder top
        self.ranges = np.array([26, 26, 26, 26, 26])

        # Gripper related variables
        self.gripperState = False
        self.grippercount = 0

        # Publishing /edrone/drone_command
        self.setpoint_pub = rospy.Publisher(
            '/edrone/drone_command', edrone_cmd, queue_size=1)
        # Subscribers
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/detect_confirm', String, self.confirm_cordinates)
        rospy.Subscriber(
            '/edrone/range_finder_bottom',
            LaserScan,
            self.height_callback)
        rospy.Subscriber(
            '/edrone/range_finder_top',
            LaserScan,
            self.laser_callback)
        self.gripper_check = rospy.Subscriber(
            "/edrone/gripper_check", String, self.checkGripper)
        self.gripper_activate = rospy.ServiceProxy(
            '/edrone/activate_gripper', Gripper)
        # ------------------------------------------------------------------------------------------------------------

    # Callback for Laser sensor ranges
    def laser_callback(self, msg):
        self.ranges = np.array(msg.ranges)

    # Callback for GPS location
    def gps_callback(self, msg):
        self.currentloc = np.array([msg.latitude, msg.longitude, msg.altitude])

    # Callback for height
    def height_callback(self, msg):
        if msg.ranges[0] > 4:
            self.height = msg.ranges[0]

    # For convering latitude to X coordinate
    def lat_to_x(self, input_latitude):
        return 110692.0702932625 * (input_latitude - 19)

    # For converting longitude to Y coordinate
    def long_to_y(self, input_longitude):
        return -105292.0089353767 * (input_longitude - 72)

    # For generating waypoints between (lat1, long1) and (lat2, long2)
    # dist is the distance between waypoints / step size
    def waypoint_generator(self, loc1, loc2):
        x0, y0 = self.lat_to_x(loc1[0]), self.long_to_y(loc1[1])
        x1, y1 = self.lat_to_x(loc2[0]), self.long_to_y(loc2[1])
        # Distance between coordinates
        d = math.hypot((x1 - x0), (y1 - y0))
        dist = 35
        # Distance of next waypoint from (lat1, long1)
        self.dt = self.dt + dist if (self.dt + dist) < d else d
        # Ratio of waypoint distance to total distance (is equal to 1 when at
        # final goal)
        self.t = self.dt / d
        self.waypoint = np.array(
            [((1 - self.t) * x0 + self.t * x1), ((1 - self.t) * y0 + self.t * y1), 0])
        # Setting waypoint height according to whichever point is higher (start
        # or end)
        if loc1[2] > loc2[2]:
            self.waypoint[2] = loc1[2] + 10
        else:
            self.waypoint[2] = loc2[2] + 10

    # Function for checking limits of PID output
    def checkLimits(self, drone_command):
        if drone_command > self.max_value:
            return self.max_value
        elif drone_command < self.min_value:
            return self.min_value
        else:
            return drone_command

    # Callback for detected marker information
    def confirm_cordinates(self, data):
        strdata = data.data.split(',')
        if strdata[0] == 'True':
            self.detectconf = True
            self.detectedcoord[1] = strdata[1]
            self.detectedcoord[2] = strdata[2]

    # Square spiral search pattern that starts with a distance of 5 m at a
    # height of 10 m
    def Search_pattern(self):
        if(self.iterator % 2 == 0):
            self.side += 5

        if self.iterator == 0:
            self.waypoint[2] = self.end[2] + 10

        elif self.iterator % 4 == 0:
            self.waypoint[0] = self.currentlocxy[0] - self.side

            print("move left {}m".format(self.side))
        elif self.iterator % 4 == 1:
            self.waypoint[1] = self.currentlocxy[1] + self.side
            print("move up {}m".format(self.side))

        elif self.iterator % 4 == 2:
            self.waypoint[0] = self.currentlocxy[0] + self.side
            print("move right {}m".format(self.side))

        elif self.iterator % 4 == 3:
            self.waypoint[1] = self.currentlocxy[1] - self.side
            print("move down {}m".format(self.side))

        self.iterator += 1

    # Callback for gripper state
    def checkGripper(self, data):
        if data.data == "True":
            self.gripperState = True

    # Function for PID control
    def pid(self):
        # Calculating XYZ coordinates
        self.currentlocxy = np.array([self.lat_to_x(
            self.currentloc[0]), self.long_to_y(self.currentloc[1]), self.currentloc[2]])
        # Calculating error, derivative and integral
        self.error = np.round((self.waypoint - self.currentlocxy), 7)
        derivative = np.round(
            ((self.error - self.prev_values) / self.sample_time), 7)
        self.integral = np.round(
            ((self.integral + self.error) * self.sample_time), 7)
        # PID output
        output = np.round(
            ((self.Kp * self.error) + (self.Ki * self.integral) + (self.Kd * derivative)), 7)
        # Final values for publishing after checking limits
        throttle = self.checkLimits(1500.0 + output[2])
        pitch = self.checkLimits(1500.0 - output[1])
        roll = self.checkLimits(1500.0 + output[0])
        # Assigning previous value with error for next iteration
        self.prev_values = self.error
        # Publishing the commands
        self.setpoint_rpy.rcRoll = roll
        self.setpoint_rpy.rcPitch = pitch
        self.setpoint_rpy.rcYaw = 1500
        self.setpoint_rpy.rcThrottle = throttle
        self.setpoint_pub.publish(self.setpoint_rpy)

    def start_end(self):
        if self.manifest[self.building_flag][0] == "DELIVERY":
            floats = [float(x)
                      for x in self.manifest[self.building_flag][2].split(';')]
            if self.delivery_flag == 1:
                self.start = self.pickuploc[self.manifest[self.building_flag][1].strip(
                )]
                self.end = np.array(floats)
            else:
                self.start = self.spawnloc
                self.end = self.pickuploc[self.manifest[self.building_flag][1].strip(
                )]
        else:
            floats = [float(x)
                      for x in self.manifest[self.building_flag][1].split(';')]
            if self.delivery_flag == 1:
                self.start = np.array(floats)
                self.end = self.returnloc[self.manifest[self.building_flag][2].strip(
                )]
            else:
                self.start = self.spawnloc
                self.end = np.array(floats)

        if self.building_flag == len(self.manifest) + 1:
            self.start = self.spawnloc
            self.end = self.initial_loc

        self.flag_once = 1

    def avoid_func(self, index, sign):
        if self.avoid_flag == 0:
            self.avoid_flag = 1
            self.waypoint = self.currentlocxy
        self.waypoint[index] = self.waypoint[index] + sign * 10

    # Function for pickup state
    def pickup(self):
        if not np.any(self.currentloc):
            return
        # Initial spawn location
        elif self.flag == 0:
            self.spawnloc = self.currentloc
            self.initial_loc = self.currentloc
            self.flag = 1

        # Avoid if still far from pickup location
        if abs(self.error[0]) > 10 and abs(self.error[1]) > 10:
            # If obstacle detcted by left laser
            if self.ranges[3] < 5 and self.ranges[3] > 0.3:
                self.avoid_func(1, -1)

            # If obstacle detcted by front laser
            elif self.ranges[0] < 5 and self.ranges[0] > 0.3:
                self.avoid_func(0, -1)

            # If obstacle detcted by right laser
            elif self.ranges[1] < 5 and self.ranges[1] > 0.3:
                self.avoid_func(1, -1)

            # If obstacle detcted by back laser
            elif self.ranges[2] < 5 and self.ranges[2] > 0.3:
                self.avoid_func(0, 1)

        # Main waypoint generating condition if no obstacle
        elif ((abs(self.error[0]) < 10 and abs(self.error[1]) < 10 and abs(
                self.error[2]) < 0.1 and self.t != 1) or self.avoid_flag == 1):
            # After clearing an obstacle
            if self.avoid_flag == 1:
                self.start[0] = self.currentloc[0]
                self.start[1] = self.currentloc[1]
                self.dt = 0
                self.t = 0
                self.avoid_flag = 0
            # Declaring initial start point only once
            elif self.flag_once == 0:
                self.start_end()
            # Generating waypoints to final goal
            self.waypoint_generator(self.start, self.end)

        # Call PID function for publishing control commands
        self.pid()

        # After reaching end point
        if self.t == 1:
            # Reach end point with minimum error
            if abs(self.error[0]) > 0.1 or abs(self.error[1]) > 0.1:
                return
            # Switching to drop state
            elif self.delivery_flag == 1:
                self.start_detection_flag = 1
                self.delivery_flag = 0
                self.flag_once = 0
                self.detectconf = False
                self.detectedcoord = [0.0, "0.0", "0.0"]
            # Picking up package
            elif self.delivery_flag == 0:
                gripper_response = self.gripper_activate(True)
                self.waypoint[2] = self.end[2]
                if gripper_response.result is True:
                    if self.grippercount < 5:
                        self.grippercount += 1
                    else:
                        self.grippercount = 0
                        self.delivery_flag = 1
                        self.flag_once = 0
                else:
                    self.setpoint_rpy.rcThrottle = 1000
                    self.setpoint_pub.publish(self.setpoint_rpy)
                    return
            # Resetting variables
            self.error = np.array([0, 0, 0])
            self.dt = 0
            self.prev_values = np.array([0, 0, 0])
            self.integral = np.array([0, 0, 0])
            self.t = 0

    # Function for detection state
    def drop(self):
        # As soon as drone switches to drop state, start search if not already
        # detected
        if ((abs(self.error[0]) < 0.1 and abs(self.error[1]) < 0.1 and abs(self.error[2]) < 0.1)
                ) and self.detectconf is False and self.manifest[self.building_flag][0] == "DELIVERY":
            self.Search_pattern()

        # If detected, seek marker and drop to 5m above building for better
        # detection
        elif self.detectconf is True and self.detectedcoord[1] != "inf" and self.detectedcoord[1] != "-inf" and self.detectedcoord[1] != '0.0' and self.delivery_flag == 0 and self.manifest[self.building_flag][0] == "DELIVERY":
            self.waypoint[0] = self.currentlocxy[0] + \
                float(self.detectedcoord[1]) * self.height
            # 0.35 is camera offset from drone centre
            self.waypoint[1] = self.currentlocxy[1] + \
                float(self.detectedcoord[2]) * self.height + 0.35
            self.waypoint[2] = self.currentloc[2] - self.height + 5
            self.delivery_flag = 1
            self.detection_count += 1

        # After reaching marker
        elif (abs(self.error[0]) < 0.2 and abs(self.error[1]) < 0.2) and self.delivery_flag == 1 and self.detection_count < 2:
            # Detect and seek one more time
            if self.detection_count < 1:
                self.delivery_flag = 0
                return
            # When waypoint is within error threshold
            elif self.detection_count == 1 and (abs(self.error[0]) < 0.01 or abs(self.error[1]) < 0.01):
                self.detection_count += 1

        # Deactivate gripper
        elif self.detection_count == 2 or self.manifest[self.building_flag][0] != "DELIVERY":
            if self.flag_once == 0:
                self.waypoint[2] = self.currentloc[2] - self.height
                self.flag_once = 1
            elif abs(self.error[2]) < 0.5 and self.flag_once == 1:
                gripper_response = self.gripper_activate(False)
                if gripper_response.result is False:
                    if self.grippercount < 3:
                        self.grippercount += 1
                    else:
                        self.grippercount = 0
                        self.start_detection_flag = 0
                        self.spawnloc = self.currentloc
                        # Reinitialising parameters for next building
                        self.delivery_flag = 0
                        self.detection_count = 0
                        self.iterator = 0
                        self.side = 0
                        self.building_flag += 1
                        if self.building_flag == len(self.manifest):
                            self.setpoint_rpy.rcThrottle = 1000
                            self.setpoint_pub.publish(self.setpoint_rpy)
                            exit(0)
                        self.error = np.array([0, 0, 0])
                        self.flag_once = 0
                        return

        # Calling PID function
        self.pid()


# ------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':

    e_drone_position = Position()
    # Defining rospy rate such that PID algorithm loops at the
    # desired sampling rate
    r = rospy.Rate(e_drone_position.sample_rate)
    while not rospy.is_shutdown():
        # Call pickup function if delivery flag is 0 else call delivery
        # function
        if e_drone_position.start_detection_flag == 0:
            e_drone_position.pickup()
        elif e_drone_position.start_detection_flag == 1:
            e_drone_position.drop()
        r.sleep()
