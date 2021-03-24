#!/usr/bin/env python

# Importing the required libraries
from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix, LaserScan
from std_msgs.msg import Float32, String
import rospy
import numpy as np
import math
import time


class Position():

    def __init__(self):
        # initializing ros node with name position_controller
        rospy.init_node('node_position_controller')

        # GPS coordinates of buildings with height 1m more than specified
        self.buildingloc = np.array([[18.9993675932,
                                      72.0000569892,
                                      10.7 + 1],
                                     [18.9990965925,
                                      71.9999050292,
                                      22.2 + 1],
                                     [18.9990965928,
                                      72.0000664814,
                                      10.75 + 1]])
        self.marker_id = [3, 2, 1]
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
        self.flag = 0

        # Parameters required for PID
        self.Kp = np.array([325, 325, 225]) * 0.6
        self.Ki = np.array([0, 0, 4]) * 0.008
        self.Kd = np.array([1625, 1625, 465]) * 0.3
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
        self.detection_flag = 0
        # Waypoint for trajectory
        self.waypoint = np.array([0, 0, 0])

        # Publishing /edrone/drone_command and marker related information
        self.setpoint_pub = rospy.Publisher(
            '/edrone/drone_command', edrone_cmd, queue_size=1)
        self.marker_related_pub = rospy.Publisher(
            '/marker_related', String, queue_size=1)

        # Subscribers
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/detect_confirm', String, self.confirm_cordinates)
# ------------------------------------------------------------------------------------------------------------

    # Callback for Laser sensor ranges
    def laser_callback(self, msg):
        self.ranges = np.array(msg.ranges)

    # Callback for GPS location
    def gps_callback(self, msg):
        self.currentloc = np.array([msg.latitude, msg.longitude, msg.altitude])

    # For convering latitude to X coordinate
    def lat_to_x(self, input_latitude):
        return 110692.0702932625 * (input_latitude - 19)

    # For converting longitude to Y coordinate
    def long_to_y(self, input_longitude):
        return -105292.0089353767 * (input_longitude - 72)

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
            self.waypoint[2] = self.buildingloc[self.building_flag][2] + 10

        elif self.iterator % 4 == 0:
            self.waypoint[0] = self.currentlocxy[0] - self.sider

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

    # Function for PID control
    def pid(self):
        # Calculating XYZ coordinates
        self.currentlocxy = np.array([self.lat_to_x(
            self.currentloc[0]), self.long_to_y(self.currentloc[1]), self.currentloc[2]])
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

    # Function for building seek state
    def building_seek(self):
        # Initially, GPS location is not updated,i.e., [0,0,0]
        # To avoid undesired motion due to this the following is used.
        # It returns to the main function if any values are 0
        if not np.any(self.currentloc):
            return

        # Assign waypoint with the building location and height 26
        if self.flag == 0:
            self.waypoint[0] = self.lat_to_x(
                self.buildingloc[self.building_flag][0])
            self.waypoint[1] = self.long_to_y(
                self.buildingloc[self.building_flag][1])
            self.waypoint[2] = 26
            self.flag = 1

        # Calling PID function
        self.pid()
        self.marker_related_pub.publish("False," + str(0))

        # After reaching, drop to 1m above the specified height
        if abs(self.error[0]) < 0.1 and abs(self.error[1]) < 0.1:
            if self.waypoint[2] == 26:
                self.waypoint[2] = self.buildingloc[self.building_flag][2]
            # Change to detection state and do not consider previous detections
            elif abs(self.error[2]) < 0.1:
                self.detectconf = False
                self.detectedcoord = [0, "0", "0"]
                self.start_detection_flag = 1

    # Function for detection state
    def detection(self):
        # As soon as drone switches to detection state, start search
        if ((abs(self.error[0]) < 0.1 and abs(self.error[1]) < 0.1 and abs(
                self.error[2]) < 0.1)) and self.detectconf is False:
            self.Search_pattern()
            # Re-use this flag for a second, better detection
            self.flag = 0

        # If detected, seek marker and drop to 5m above building for better
        # detection
        elif self.detectconf is True and self.detection_flag == 0 and self.detectedcoord[1] != "inf" and self.detectedcoord[1] != "-inf" and self.detectedcoord[1] != '0.0':
            self.waypoint[0] = self.currentlocxy[0] + \
                float(self.detectedcoord[1])
            self.waypoint[1] = self.currentlocxy[1] + \
                float(self.detectedcoord[2])
            self.waypoint[2] = self.buildingloc[self.building_flag][2] + 5
            self.detection_flag = 1

        # After reaching marker
        elif ((abs(self.error[0]) < 0.1 and abs(self.error[1]) < 0.1 and abs(self.error[2]) < 0.1)):
            # Detect and seek one more time
            if self.flag == 0:
                self.detection_flag = 0
                self.flag = 1
                return
            # If the final building is reached, then land
            elif self.building_flag == 2:
                if abs(self.error[0]) > 0.01 or abs(self.error[1]) > 0.01:
                    self.pid()
                    return
                else:
                    self.setpoint_rpy.rcThrottle = 1000
                    self.setpoint_pub.publish(self.setpoint_rpy)
                    return
            # Reinitialising parameters for next building
            self.detection_flag = 0
            self.flag = 0
            self.start_detection_flag = 0
            self.iterator = 0
            self.side = 0
            self.building_flag += 1

        # Calling PID function
        self.pid()

        # Publish info to marker_related topic so it can be republished at 1Hz
        # through detection script
        if self.detection_flag == 1:
            # 1st and 3rd building have been flipped in self.building_loc
            curr_id = self.marker_id[self.building_flag]
            self.marker_related_pub.publish(
                "True," + str(curr_id) + "," + str(self.error[0]) + "," + str(self.error[1]))
        else:
            curr_id = self.marker_id[self.building_flag]
            self.marker_related_pub.publish("False," + str(curr_id))


# ------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':

    pos = Position()
    r = rospy.Rate(pos.sample_rate)
    while not rospy.is_shutdown():
        # Call building seek or detection depending on flag
        if pos.start_detection_flag == 0:
            pos.building_seek()
        else:
            pos.detection()
        r.sleep()
