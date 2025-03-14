#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <std_msgs/Float32MultiArray.h>

namespace gazebo
{
  class IMUVisualizer : public ModelPlugin
  {
  private:
    physics::ModelPtr model;
    physics::LinkPtr imuLink;
    ros::NodeHandle* nh;
    ros::Subscriber imu_sub;

  public:
    IMUVisualizer() {}

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      if (!ros::isInitialized())
      {
        ROS_FATAL("A ROS node for Gazebo has not been initialized!");
        return;
      }

      model = _parent;
      imuLink = model->GetLink("imu_link");
      nh = new ros::NodeHandle();

      imu_sub = nh->subscribe("/calibrated_imu", 10, &IMUVisualizer::imuCallback, this);
      ROS_INFO("IMU Visualizer Plugin Loaded!");
    }

    void imuCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
      if (msg->data.size() < 3) return;

      double roll = msg->data[0] * M_PI / 180.0; // Convert to radians
      double pitch = msg->data[1] * M_PI / 180.0;
      double yaw = msg->data[2] * M_PI / 180.0;

      ignition::math::Quaterniond quat;
      quat.Euler(roll, pitch, yaw);

      imuLink->SetWorldPose(ignition::math::Pose3d(imuLink->WorldPose().Pos(), quat));
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(IMUVisualizer)
}
