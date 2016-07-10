#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <cmath>
#include <geometry_msgs/PoseArray.h>
#include <dji_sdk/Gimbal.h>
#include <dji_sdk/dji_sdk.h>
#include <apriltags/AprilTagDetections.h>



ros::Subscriber apriltags_pos_sub;
ros::Subscriber gimbal_ori_sub;
ros::Subscriber filtered_x_sub;
ros::Subscriber filtered_y_sub;
ros::Subscriber filtered_z_sub;


ros::Publisher setpoint_yaw_pub;
ros::Publisher setpoint_pitch_pub;

ros::ServiceClient gimbal_control_service;


std_msgs::Float64 setpoint_yaw;
std_msgs::Float64 setpoint_pitch;

double gimbal_roll;
double gimbal_yaw;
double gimbal_pitch;

double tag_x;
double tag_y;
double tag_z;

double yaw_goal;
double pitch_goal;

bool updated = false;
bool apriltag_in_sight = false;


const double z_threshold = 0.0001;

std::string tag_detection_topic;



void apriltagsPositionCallback(const apriltags::AprilTagDetections::ConstPtr& apriltag_pos_msg)
{
  if(std::begin(apriltag_pos_msg->detections) == std::end(apriltag_pos_msg->detections))
  {
    apriltag_in_sight = false;
    return;
  }
  else
  {
    tag_x = apriltag_pos_msg->detections[0].pose.position.x;
    tag_y = -apriltag_pos_msg->detections[0].pose.position.y;
    tag_z = apriltag_pos_msg->detections[0].pose.position.z;

    if(fabs(tag_z) < 0.0001)
    {
      return;
    }

  }
  // setpoint_yaw.data = gimbal_yaw + atan(tag_x / tag_z) * 180 / M_PI;
  // setpoint_pitch.data = gimbal_pitch + atan(tag_y / tag_z) * 180 / M_PI;
  yaw_goal = atan(tag_x / tag_z) * 1800 / M_PI;
  pitch_goal = atan(tag_y / tag_z) * 1800 / M_PI;
}


void gimbalOrientationCallback(const dji_sdk::Gimbal::ConstPtr& gimbal_ori_msg)
{
  gimbal_roll = gimbal_ori_msg->roll;
  gimbal_yaw = gimbal_ori_msg->yaw;
  gimbal_pitch = gimbal_ori_msg->pitch;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "gimbal_angle_control_node");
  ROS_INFO("Starting gimbal angle control");
  ros::NodeHandle nh;

  nh.param<std::string>("/gimbal_track_setpoint/tag_detection_topic", tag_detection_topic, "/apriltags/36h11/detections");

  ROS_INFO("Listening to apriltag detection topic: %s", tag_detection_topic.c_str());

  gimbal_control_service = nh.serviceClient<dji_sdk::GimbalAngleControl>("dji_sdk/gimbal_angle_control");

  apriltags_pos_sub = nh.subscribe(tag_detection_topic, 1000, apriltagsPositionCallback);
  gimbal_ori_sub = nh.subscribe("/dji_sdk/gimbal", 1000, gimbalOrientationCallback);

  dji_sdk::GimbalAngleControl gimbal_angle_control;
  

  ros::Rate loop_rate(200); 

  while (ros::ok())
  {
    ros::spinOnce();
    ROS_DEBUG_ONCE("Controlling gimbal");

    gimbal_angle_control.request.yaw = yaw_goal;
    gimbal_angle_control.request.pitch = pitch_goal;
    gimbal_angle_control.request.roll = 0;
    gimbal_angle_control.request.duration = 10;
    gimbal_angle_control.request.absolute_or_incremental = 0;
    gimbal_angle_control.request.yaw_cmd_ignore = 0;
    gimbal_angle_control.request.roll_cmd_ignore = 0;
    gimbal_angle_control.request.pitch_cmd_ignore = 0;


    gimbal_control_service.call(gimbal_angle_control);

    if(!gimbal_angle_control.response.result)
    {
      ROS_ERROR("Cannot control gimbal");
    }

    loop_rate.sleep();
  }
}
