#!/usr/bin/env python3

# Code for manual AUV control

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy

class JoystickControl:
    def __init__(self):
        rospy.init_node("joy_control", anonymous=True)

        self.vel_th_r = 0    # right thruster
        self.vel_th_l = 0    # left thruster
        self.vel_th_f = 0    # front thruster
        self.vel_th_b = 0    # back thruster

        self.sub = rospy.Subscriber("/joy", Joy, self.callback)

        self.pub1 = rospy.Publisher("/right_thruster", Int16, queue_size=1)
        self.pub2 = rospy.Publisher("/left_thruster", Int16, queue_size=1)
        self.pub3 = rospy.Publisher("/front_thruster", Int16, queue_size=1)
        self.pub4 = rospy.Publisher("/back_thruster", Int16, queue_size=1)

    def callback(self, data):
        xAxis1 = data.axes[0]   # yaw motion
        yAxis1 = data.axes[1]   # forward/backward motion
        xAxis2 = data.axes[3]   # pitch motion
        yAxis2 = data.axes[4]   # upward/downward motion

        # AUV Motion Control

        #Forward/Backward motion

        if (abs(yAxis1) > abs(xAxis1)):
            self.vel_th_r = int(yAxis1 * 400)
            self.vel_th_l = int(yAxis1 * 400)

        # Roll motion

        if (abs(yAxis1) < abs(xAxis1)):
            self.vel_th_r = int(xAxis1 * 400)
            self.vel_th_l = int(xAxis1 * -400)

        #Upward/Downward motion

        if (abs(yAxis2) > abs(xAxis2)):
            self.vel_th_r = int(yAxis1 * 400)
            self.vel_th_l = int(yAxis1 * 400)

        # Pitch motion

        if (abs(yAxis2) < abs(xAxis2)):
            self.vel_th_r = int(xAxis1 * 400)
            self.vel_th_l = int(xAxis1 * -400)

        # Ensure thrusters reset when joystick is centered
        if (abs(yAxis1) < 0.01) and (abs(xAxis1) < 0.01):  # Tiny values are treated as zero
            self.vel_th_r = 0
            self.vel_th_l = 0

        if (abs(yAxis2) < 0.01) and (abs(xAxis2) < 0.01):  # Tiny values are treated as zero
            self.vel_th_f = 0
            self.vel_th_b = 0        

        self.pub1.publish(self.vel_th_r)
        self.pub2.publish(self.vel_th_l)
        self.pub3.publish(self.vel_th_f)
        self.pub4.publish(self.vel_th_b)

if __name__ == '__main__':
    joystick_control = JoystickControl()
    rospy.spin()
