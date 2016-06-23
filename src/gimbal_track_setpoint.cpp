#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <cmath>
#include <geometry_msgs/PoseArray.h>
#include <dji_sdk/Gimbal.h>




ros::Subscriber apriltags_pos_sub;
ros::Subscriber gimbal_ori_sub;

ros::Publisher setpoint_yaw_pub;
ros::Publisher setpoint_pitch_pub;


std_msgs::Float64 setpoint_yaw;
std_msgs::Float64 setpoint_pitch;

double gimbal_roll;
double gimbal_yaw;
double gimbal_pitch;

double tag_x;
double tag_y;
double tag_z;

std::string tag_detection_topic;

void apriltagsPositionCallback(const geometry_msgs::PoseArray::ConstPtr& apriltag_pos_msg)
{
  if(std::begin(apriltag_pos_msg->poses) == std::end(apriltag_pos_msg->poses))
  {
    return;
  }
  else
  {
    tag_x = apriltag_pos_msg->poses[0].position.x;
    tag_y = -apriltag_pos_msg->poses[0].position.y;
    tag_z = apriltag_pos_msg->poses[0].position.z;

    if(fabs(tag_z) < 0.0001)
    {
      return;
    }

  }
  setpoint_yaw.data = gimbal_yaw + atan(tag_x / tag_z) * 180 / M_PI;
  setpoint_pitch.data = gimbal_pitch + atan(tag_y / tag_z) * 180 / M_PI;
}

void gimbalOrientationCallback(const dji_sdk::Gimbal::ConstPtr& gimbal_ori_msg)
{
  gimbal_roll = gimbal_ori_msg->roll;
  gimbal_yaw = gimbal_ori_msg->yaw;
  gimbal_pitch = gimbal_ori_msg->pitch;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "setpoint_node");
  ROS_INFO("Starting setpoint publisher");
  ros::NodeHandle nh;

  while (ros::Time(0) == ros::Time::now())
  {
    ROS_INFO("Setpoint_node spinning waiting for time to become non-zero");
    sleep(1);
  }

  nh.param<std::string>("tag_detection_topic", tag_detection_topic, "/apriltags_ros/tag_detections_pose");

  setpoint_yaw_pub = nh.advertise<std_msgs::Float64>("/teamhku/gimbal_track/setpoint_yaw", 1);
  setpoint_pitch_pub = nh.advertise<std_msgs::Float64>("/teamhku/gimbal_track/setpoint_pitch", 1);

  apriltags_pos_sub = nh.subscribe(tag_detection_topic, 1000, apriltagsPositionCallback);
  gimbal_ori_sub = nh.subscribe("/dji_sdk/gimbal", 1000, gimbalOrientationCallback);

  ros::Rate loop_rate(200);   // change setpoint every 5 seconds

  while (ros::ok())
  {
    ros::spinOnce();
    setpoint_yaw_pub.publish(setpoint_yaw);     
    setpoint_pitch_pub.publish(setpoint_pitch);

    loop_rate.sleep();
  }
}
