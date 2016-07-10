#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <vector>
#include <iterator>
#include <string>

#include <dji_sdk/GimbalAngleControl.h>
#include <dji_sdk/GimbalSpeedControl.h>
#include <dji_sdk/Gimbal.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseArray.h>

#include <Eigen/Geometry> 


#include <cmath>

ros::Subscriber gimbal_ori_sub;

ros::Publisher yaw_state_pub;
ros::Publisher pitch_state_pub;



double gimbal_roll;
double gimbal_yaw;
double gimbal_pitch;


void gimbalOrientationCallback(const dji_sdk::Gimbal::ConstPtr& gimbal_ori_msg)
{
  gimbal_roll = gimbal_ori_msg->roll;
  gimbal_yaw = gimbal_ori_msg->yaw;
  gimbal_pitch = gimbal_ori_msg->pitch;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "gimbal_track_state_pub_node");
	ROS_INFO("Starting gimbal state publisher");
	ros::NodeHandle nh;

	gimbal_ori_sub = nh.subscribe("/dji_sdk/gimbal", 100, gimbalOrientationCallback);
	yaw_state_pub = nh.advertise<std_msgs::Float64>("/teamhku/gimbal_track/yaw_state", 10);
	pitch_state_pub = nh.advertise<std_msgs::Float64>("/teamhku/gimbal_track/pitch_state", 10);

	std_msgs::Float64 yaw_state_msg;
	std_msgs::Float64 pitch_state_msg;

  	ros::Rate loop_rate(200);   // change setpoint every 5 seconds

	while (ros::ok())
	{
		ros::spinOnce();

		yaw_state_msg.data = gimbal_yaw;
		pitch_state_msg.data = gimbal_pitch;

		yaw_state_pub.publish(yaw_state_msg);     
		pitch_state_pub.publish(pitch_state_msg);

		loop_rate.sleep();
	}
}
