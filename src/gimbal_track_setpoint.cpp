#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <cmath>
#include <geometry_msgs/PoseArray.h>
#include <dji_sdk/Gimbal.h>
#include <apriltags/AprilTagDetections.h>



ros::Subscriber apriltags_pos_sub;
ros::Subscriber gimbal_ori_sub;
ros::Subscriber filtered_x_sub;
ros::Subscriber filtered_y_sub;
ros::Subscriber filtered_z_sub;


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
  setpoint_yaw.data = gimbal_yaw + atan(tag_x / tag_z) * 180 / M_PI;
  setpoint_pitch.data = gimbal_pitch + atan(tag_y / tag_z) * 180 / M_PI;
}








// void apriltagsPositionCallback(const geometry_msgs::PoseArray::ConstPtr& apriltag_pos_msg)
// {
//   if(std::begin(apriltag_pos_msg->poses) == std::end(apriltag_pos_msg->poses))
//   {
//     apriltag_in_sight = false;
//     return;
//   }
//   else
//   {
//     tag_x = apriltag_pos_msg->poses[0].position.x;
//     tag_y = -apriltag_pos_msg->poses[0].position.y;
//     tag_z = apriltag_pos_msg->poses[0].position.z;

//     if(fabs(tag_z) < 0.0001)
//     {
//       return;
//     }

//   }
//   setpoint_yaw.data = gimbal_yaw + atan(tag_x / tag_z) * 180 / M_PI;
//   setpoint_pitch.data = gimbal_pitch + atan(tag_y / tag_z) * 180 / M_PI;
// }

void gimbalOrientationCallback(const dji_sdk::Gimbal::ConstPtr& gimbal_ori_msg)
{
  gimbal_roll = gimbal_ori_msg->roll;
  gimbal_yaw = gimbal_ori_msg->yaw;
  gimbal_pitch = gimbal_ori_msg->pitch;
}

// void filteredXCallback(std_msgs::Float64 filtered_x_msg)
// {
//   if(updated)
//   {
//     if(fabs(tag_z) < z_threshold)
//     {
//       return;
//     }
//     tag_x = filtered_x_msg.data;
//     setpoint_yaw.data = gimbal_yaw + atan(tag_x / tag_z) * 180 / M_PI;
//   }
  
// }

// void filteredZCallback(std_msgs::Float64 filtered_z_msg)
// {
//   tag_z = filtered_z_msg.data;
//   updated = true;
// }

// void filteredYCallback(std_msgs::Float64 filtered_y_msg)
// {
//   if(updated)
//   {
//     if(fabs(tag_z) < z_threshold)
//     {
//       return;
//     }
//     tag_y = -filtered_y_msg.data;
//     setpoint_pitch.data = gimbal_pitch + atan(tag_y / tag_z) * 180 / M_PI;
//   }
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "setpoint_node");
  ROS_INFO("Starting gimbal track setpoint publisher");
  ros::NodeHandle nh;

  while (ros::Time(0) == ros::Time::now())
  {
    ROS_INFO("Setpoint_node spinning waiting for time to become non-zero");
    sleep(1);
  }

  nh.param<std::string>("/gimbal_track_setpoint/tag_detection_topic", tag_detection_topic, "/apriltags_ros/tag_detections_pose");

  ROS_INFO("Listening to apriltag detection topic: %s", tag_detection_topic.c_str());

  setpoint_yaw_pub = nh.advertise<std_msgs::Float64>("/teamhku/gimbal_track/setpoint_yaw", 1);
  setpoint_pitch_pub = nh.advertise<std_msgs::Float64>("/teamhku/gimbal_track/setpoint_pitch", 1);

  apriltags_pos_sub = nh.subscribe(tag_detection_topic, 1000, apriltagsPositionCallback);
  gimbal_ori_sub = nh.subscribe("/dji_sdk/gimbal", 1000, gimbalOrientationCallback);
  // filtered_x_sub = nh.subscribe("/teamhku/filtered_data/tag_detection_x", 100, filteredXCallback);
  // filtered_y_sub = nh.subscribe("/teamhku/filtered_data/tag_detection_y", 100, filteredYCallback);
  // filtered_z_sub = nh.subscribe("/teamhku/filtered_data/tag_detection_z", 100, filteredZCallback);

  ros::Rate loop_rate(200); 

  while (ros::ok())
  {
    ros::spinOnce();
    ROS_DEBUG_ONCE("Publishing the setpoint data...");
    setpoint_yaw_pub.publish(setpoint_yaw);     
    setpoint_pitch_pub.publish(setpoint_pitch);

    loop_rate.sleep();
  }
}
