#!/usr/bin/env python

# Importing the required libraries

from vitarana_drone.msg import *
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import tf
import numpy as np


class Attitude():

    def __init__(self):
        # initializing ros node with name attitude_controller
        rospy.init_node('node_attitude_controller')

        # This corresponds to your current orientation of eDrone in quaternion format
        # [x,y,z,w]
        self.drone_orientation_quaternion = np.array([0.0, 0.0, 0.0, 0.0])

        # This corresponds to your current orientation of eDrone converted in euler angles form.
        # [p,r,y]
        self.drone_orientation_euler = np.array([0.0, 0.0, 0.0])

        # Subscribed from /edrone/drone_command
        # [p_setpoint, r_setpoint, y_setpoint]
        self.setpoint_cmd = np.array([0.0, 0.0, 0.0])
        self.throttle = 0.0

        # The setpoint of orientation in euler angles
        # [p_setpoint, r_psetpoint, y_setpoint]
        self.setpoint_euler = np.array([0.0, 0.0, 0.0])

        # Declaring pwm_cmd of message type prop_speed and initializing values
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        # Numpy array for PID gains : [Pitch, Roll, Yaw] * coefficient ratios
        self.Kp = np.array([20, 20, 108]) * 0.06
        self.Ki = np.array([11, 11, 8]) * 0.008
        self.Kd = np.array([36, 36, 306]) * 0.03

        # For storing previous error for derivative term
        self.prev_values = np.array([0.0, 0.0, 0.0])
        # For storing sum of error for integral term
        self.integral = np.array([0.0, 0.0, 0.0])
        # Maximum and Minimum values for pwm commands
        self.max_value = 1023.0
        self.min_value = 0.0

        # PID sampling rate and time
        self.sample_rate = 10  # in Hz
        self.sample_time = 0.1  # in seconds

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll,
        # /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber(
            '/edrone/drone_command',
            edrone_cmd,
            self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)

# ------------------------------------------------------------------------------------------------------------

    # Callback for orientation from IMU in quaternion
    def imu_callback(self, msg):
        self.drone_orientation_quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w]

    # Callback for roll, pitch, yaw and throttle setpoints
    def drone_command_callback(self, msg):
        self.setpoint_cmd = np.array([msg.rcPitch, msg.rcRoll, msg.rcYaw])
        self.throttle = msg.rcThrottle
 # ----------------------------------------------------------------------------------------------------------------------

    # Function for checking limits of PID output
    def checkLimits(self, pwm):
        if pwm > self.max_value:
            return self.max_value
        elif pwm < self.min_value:
            return self.min_value
        else:
            return pwm

    # PID algorithm
    def pid(self):
        # Converting quaternion to euler angles
        self.drone_orientation_euler = np.array(
            tf.transformations.euler_from_quaternion(
                self.drone_orientation_quaternion))

        # Initially, setpoint is not updated,i.e., [0,0,0]
        # To avoid undesired motion due to this, the following is used.
        # It returns to the main function if any values are 0
        if not np.any(self.setpoint_cmd):
            return

        # Convertng the range from 1000 to 2000 in the range of -10 degree to
        # 10 degree for pitch, roll  setpoints
        self.setpoint_euler[0:2] = self.setpoint_cmd[0:2] * 0.02 - 30
        # Converting the range from 1000 to 2000 in the range of -180 to
        # 180 degree for yaw setpoints
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.36 - 540

        # Converting throttle to 0-1023 range
        throttle = self.throttle * 1.023 - 1023

        # Calculating error term and rounding off to 7 decimal points
        # Rounding off because double precision is overkill for PID
        error = np.round(
            (self.setpoint_euler -
             np.degrees(
                 self.drone_orientation_euler)),
            7)

        # Calculating derivative term and rounding off
        # / symbol allows division for sample_time scalar with every element in the array
        derivative = np.round(
            ((error - self.prev_values) / self.sample_time), 7)

        # Calculating integral term and rounding off
        # * symbol allows multiplication for sample_time scalar with every element in the array
        self.integral = np.round(
            ((self.integral + error) * self.sample_time), 7)

        # Calculating PID output and rounding off
        # * symbol for numpy arrays allows multiplication of the corresponding elements
        # output = [out_pitch, out_roll, out_yaw]
        output = np.round(
            ((self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)), 7)

        # Motor Mixing Equations (calculating PWM values for motors)
        # A combination of throttle, pitch, roll and yaw
        # Limits are checked using checkLimits function defined earlier
        self.pwm_cmd.prop1 = self.checkLimits(
            throttle + output[0] - output[1] - output[2])
        self.pwm_cmd.prop2 = self.checkLimits(
            throttle - output[0] - output[1] + output[2])
        self.pwm_cmd.prop3 = self.checkLimits(
            throttle - output[0] + output[1] - output[2])
        self.pwm_cmd.prop4 = self.checkLimits(
            throttle + output[0] + output[1] + output[2])

        # Assigning prev_values with error for the next iteration
        self.prev_values = error

        # Publishing final PID output on /edrone/pwm to move drone
        self.pwm_pub.publish(self.pwm_cmd)


if __name__ == '__main__':

    e_drone_attitude = Attitude()
    # Defining rospy rate such that PID algorithm loops at
    # the desired sampling rate
    r = rospy.Rate(e_drone_attitude.sample_rate)
    while not rospy.is_shutdown():
        # Calling PID function
        e_drone_attitude.pid()
        r.sleep()
