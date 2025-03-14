#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;
std_msgs::Float32MultiArray imu_msg;
ros::Publisher imu_pub("/imu_data_present", &imu_msg);

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
    // Initialize the BNO055 sensor
    if (!bno.begin()) {
        while (1); // Failed to detect BNO055
    }
    
    delay(1000);
    bno.setExtCrystalUse(true);
    
    // Initialize ROS node and publisher
    nh.initNode();
    nh.advertise(imu_pub);
}

void loop() {
    // Get quaternion data instead of Euler angles for better accuracy
    imu::Quaternion quat = bno.getQuat();
    
    // Convert quaternion to Euler angles (in radians)
    // Based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    double w = quat.w();
    double x = quat.x();
    double y = quat.y();
    double z = quat.z();
    
    // Roll (rotation around X axis)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    double roll = atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (rotation around Y axis)
    double sinp = 2.0 * (w * y - z * x);
    double pitch;
    if (abs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);
    
    // Yaw (rotation around Z axis)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = atan2(siny_cosp, cosy_cosp);
    
    // Convert to degrees for easier human interpretation
    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;
    
    // Avoid malloc() -> Use static array
    static float imu_data[3];  
    imu_data[0] = roll;  // Properly labeled as roll (rotation around X)
    imu_data[1] = pitch; // Properly labeled as pitch (rotation around Y)
    imu_data[2] = yaw;   // Properly labeled as yaw (rotation around Z)
    
    imu_msg.data = imu_data;
    imu_msg.data_length = 3;
    
    imu_pub.publish(&imu_msg);
    nh.spinOnce();
    
    delay(100);
}
