#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import tf.transformations as tf_trans
import math

class IMUVisualizer:
    def __init__(self):
        rospy.init_node('imu_visualizer', anonymous=True)
        rospy.Subscriber("/imu_data_present", Float32MultiArray, self.imu_callback)
        
        # Wait for the Gazebo service
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        rospy.loginfo("IMU Visualizer initialized. Waiting for IMU data...")
    
    def imu_callback(self, msg):
        if len(msg.data) < 3:
            return # Ensure message contains roll, pitch, and yaw
        
        roll, pitch, yaw = msg.data[0], msg.data[1], msg.data[2]
        
        # Convert degrees to radians (BNO055 outputs in degrees)
        roll, pitch, yaw = math.radians(roll), math.radians(pitch), math.radians(yaw)
        
        # Convert to quaternion
        quaternion = tf_trans.quaternion_from_euler(roll, pitch, yaw)
        
        # Create model state message
        model_state = ModelState()
        model_state.model_name = "bno055_simulation"
        model_state.pose.position.x = 0
        model_state.pose.position.y = 0
        model_state.pose.position.z = 5
        model_state.pose.orientation.x = quaternion[0]
        model_state.pose.orientation.y = quaternion[1]
        model_state.pose.orientation.z = quaternion[2]
        model_state.pose.orientation.w = quaternion[3]
        model_state.reference_frame = "world"
        
        # Update model state in Gazebo
        try:
            self.set_model_state(model_state)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    visualizer = IMUVisualizer()
    visualizer.run()
