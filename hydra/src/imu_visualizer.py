#!/usr/bin/env python3
import rospy
import tf
import tf.transformations as tf_trans
from std_msgs.msg import Float32MultiArray

class IMUVisualizer:
    def __init__(self):
        rospy.init_node('imu_visualizer', anonymous=True)
        
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        rospy.Subscriber("/calibrated_imu", Float32MultiArray, self.imu_callback)
        
    def imu_callback(self, msg):
        roll, pitch, yaw = msg.data[0], msg.data[1], msg.data[2]

        # Convert roll, pitch, yaw to quaternion
        quaternion = tf_trans.quaternion_from_euler(roll, pitch, yaw)

        # Broadcast the IMU transform
        self.tf_broadcaster.sendTransform(
            (0, 0, 1),  # Position remains fixed
            quaternion,
            rospy.Time.now(),
            "imu_link",
            "world"
        )

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        visualizer = IMUVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
