#!/usr/bin/env python

# Importing the required libraries

from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import rospy
import numpy as np


class Position():

    def __init__(self):
        # initializing ros node with name position_controller
        rospy.init_node('node_position_controller')

        # Numpy array for GPS coordinate setpoints
        self.setpoints = np.array(
            [[19.0, 72.0, 3.0], [19.0000451704, 72.0, 3.0], [19.0000451704, 72.0, 0.31]])

        # Numpy array for current GPS location
        self.currentloc = np.array([0.0, 0.0, 0.0])
        # Index of GPS setpoint that the drone is headed towards
        self.loc = 0
        # Count for Stability
        self.stabilize = 0
        # Drone Command message of type edrone_cmd and initialisation
        self.setpoint_rpy = edrone_cmd()
        self.setpoint_rpy.rcRoll = 0
        self.setpoint_rpy.rcPitch = 0
        self.setpoint_rpy.rcYaw = 0
        self.setpoint_rpy.rcThrottle = 0

        # Numpy array for PID gains : [Latitude, Longitude, Altitude]
        # Coefficient ratios for Pid[Latitude] [Kp, Ki, Kd] : [6000, 0.08, 12000]
        # Coefficient ratios for Pid[Longitude] [Kp, Ki, Kd] : [6000, 0.8, 12000]
        # Coefficient ratios for Pid[Altitude] [Kp, Ki, Kd] : [0.6, 0.008, 0.3]

        # Value of [Kp, Ki, Kd][Latitude] : [720, 55, 1550]
        # Value of [Kp, Ki, Kd][Longitude] : [475, 0, 1450]
        # Value of [Kp, Ki, Kd][Altitude] : [225, 4, 465]
        self.Kp = np.array([720 * 6000, 475 * 6000, 225 * 0.6])
        self.Ki = np.array([55 * 0.08, 0 * 0.8, 4 * 0.008])
        self.Kd = np.array([1550 * 12000, 1450 * 12000, 465 * 0.3])

        # For storing previous error for derivative term
        self.prev_values = np.array([0.0, 0.0, 0.0])
        # For storing sum of error for integral term
        self.integral = np.array([0.0, 0.0, 0.0])
        # Maximum and Minimum values for roll, pitch and throttle commands
        self.max_value = 2000
        self.min_value = 1000

        # PID sampling rate and time
        self.sample_rate = 10  # in Hz
        self.sample_time = 0.1  # in seconds

        # Publishing /edrone/drone_command, /altitude_error, /latitude_error,
        # /longitude_error
        self.setpoint_pub = rospy.Publisher(
            '/edrone/drone_command', edrone_cmd, queue_size=1)
        self.altitude_error_pub = rospy.Publisher(
            '/altitude_error', Float32, queue_size=1)
        self.latitude_error_pub = rospy.Publisher(
            '/latitude_error', Float32, queue_size=1)
        self.longitude_error_pub = rospy.Publisher(
            '/longitude_error', Float32, queue_size=1)

        # Subscribing to /edrone/gps for current GPS location
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        # ------------------------------------------------------------------------------------------------------------

    # Callback for GPS location
    def gps_callback(self, msg):
        self.currentloc = np.array([msg.latitude, msg.longitude, msg.altitude])

    # Function for checking limits of PID output
    def checkLimits(self, drone_command):
        if drone_command > self.max_value:
            return self.max_value
        elif drone_command < self.min_value:
            return self.min_value
        else:
            return drone_command

    # PID algorithm
    def pid(self):
        # Initially, GPS location is not updated,i.e., [0,0,0]
        # To avoid undesired motion due to this the following is used.
        # It returns to the main function if any values are 0
        if not np.any(self.currentloc):
            return

        # Calculating error term and rounding off to 7 decimal points
        # Rounding off because double precision is overkill for PID
        error = np.round((self.setpoints[self.loc] - self.currentloc), 7)

        # Calculating derivative term and rounding off
        # / symbol allows divison for sample_time scalar with every element in the array
        derivative = np.round(
            ((error - self.prev_values) / self.sample_time), 7)

        # Calculating integral term and rounding off
        # * symbol allows multiplication for sample_time scalar with every element in the array
        self.integral = np.round(
            ((self.integral + error) * self.sample_time), 7)

        # Calculating PID output and rounding off
        # * symbol for numpy arrays allows multiplication of the corresponding elements
        # output = [out_latitude, out_longitude, out_altitude]
        output = np.round(
            ((self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)), 7)

        # Calculating throttle, roll and pitch and checking limits
        # Since 1500 is the mid value,
        # Direction will change according to sign of output
        throttle = self.checkLimits(1500.0 + output[2])
        pitch = self.checkLimits(1500.0 + output[1])
        roll = self.checkLimits(1500.0 + output[0])

        # Assigning prev_values with error for the next iteration
        self.prev_values = error

        # Publishing errors for Plotjuggler
        self.latitude_error_pub.publish(error[0])
        self.longitude_error_pub.publish(error[1])
        self.altitude_error_pub.publish(error[2])

	# Publishing final PID output on /edrone/drone_command for the attitude
        # controller
        self.setpoint_rpy.rcRoll = roll
        self.setpoint_rpy.rcPitch = pitch
        self.setpoint_rpy.rcYaw = 1500
        self.setpoint_rpy.rcThrottle = throttle
        self.setpoint_pub.publish(self.setpoint_rpy)

        # Checking condition for threshold box of setpoints
        if abs(
            error[1]) < 0.0000047487 and abs(
            error[0]) < 0.000004517 and abs(
                error[2]) < 0.2:
            # Checking if drone has reached threshold box of final setpoint
            # Publishing stop signal (throttle = 1000) for drone to land
            if self.loc == 2:
                self.setpoint_rpy.rcThrottle = 1000
                self.setpoint_rpy.rcRoll = 1500
                self.setpoint_rpy.rcPitch = 1500
                self.setpoint_rpy.rcYaw = 1500
                self.setpoint_pub.publish(self.setpoint_rpy)
                return
            # Counting till 10 using stabilize variable for drone to stabilize
            # at setpoint
            if self.stabilize < 15:
                self.stabilize = self.stabilize + 1
                return
            # Increasing value of location index so that next GPS setpoint is
            # set
            self.loc = self.loc + 1
            # Reinitialising stabilize to 0 so it can be used for next setpoint
            self.stabilize = 0

# ------------------------------------------------------------------------------------------------------------


if __name__ == '__main__':

    e_drone_position = Position()
    # Defining rospy rate such that PID algorithm loops at the
    # desired sampling rate
    r = rospy.Rate(e_drone_position.sample_rate)
    while not rospy.is_shutdown():
        # Calling PID function
        e_drone_position.pid()
        r.sleep()
