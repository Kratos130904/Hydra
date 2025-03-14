#!/usr/bin/env python3

#Code for individual thruster control test

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy

class JoystickControl:
    def __init__(self):
        rospy.init_node("joy_control", anonymous=True)

        self.vel_th1 = 0    # right thruster
        self.vel_th2 = 0    # left thruster
        self.vel_th3 = 0    # front thruster
        self.vel_th4 = 0    # back thruster

        self.sub = rospy.Subscriber("/joy", Joy, self.callback)

        self.pub1 = rospy.Publisher("/thruster1", Int16, queue_size=1)
        self.pub2 = rospy.Publisher("/thruster2", Int16, queue_size=1)
        self.pub3 = rospy.Publisher("/thruster3", Int16, queue_size=1)
        self.pub4 = rospy.Publisher("/thruster4", Int16, queue_size=1)

    def callback(self, data):
        xAxis1 = data.axes[0]  # Thruster 1 control
        yAxis1 = data.axes[1]  # Thruster 2 control
        xAxis2 = data.axes[3]  # Thruster 3 control
        yAxis2 = data.axes[4]  # Thruster 4 control

        # Thruster velocities

        self.vel_th1 = int(xAxis1 * 400)
        self.vel_th2 = int(yAxis1 * 400)
        self.vel_th3 = int(xAxis2 * 400)
        self.vel_th4 = int(yAxis2 * 400)

        self.pub1.publish(self.vel_th1)
        self.pub2.publish(self.vel_th2)
        self.pub3.publish(self.vel_th3)
        self.pub4.publish(self.vel_th4)

if __name__ == '__main__':
    joystick_control = JoystickControl()
    rospy.spin()
