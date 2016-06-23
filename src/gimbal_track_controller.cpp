#include <ros/ros.h>
#include <vector>
#include <iterator>
#include <string>

#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/GimbalAngleControl.h>
#include <dji_sdk/GimbalSpeedControl.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/PoseArray.h>

#include <cmath>
#include <iostream>

#include <Eigen/Geometry> 

ros::Subscriber gimbal_control_pitch_sub;
ros::Subscriber gimbal_control_yaw_sub;
ros::Subscriber tag_z_sub;
ros::Subscriber tag_x_sub;
ros::Subscriber gimbal_track_enable_sub;


ros::ServiceClient gimbal_control_service;

double gimbal_control_effort_pitch;
double gimbal_control_effort_yaw;

double tag_z;
double tag_x;

const double z_threshold = 0.001;
const double x_threshold = 0.07;

bool gimbal_track_enabled = true;

void gimbalPitchControlEffortXCallback(std_msgs::Float64 gimbal_control_effort_pitch_msg)
{
	gimbal_control_effort_pitch = gimbal_control_effort_pitch_msg.data;
}

void gimbalYawControlEffortYCallback(std_msgs::Float64 gimbal_control_effort_yaw_msg)
{
	gimbal_control_effort_yaw = gimbal_control_effort_yaw_msg.data;
}

void tagZCallback(std_msgs::Float64 tag_z_msg)
{
	tag_z = tag_z_msg.data;
}

void tagXCallback(std_msgs::Float64 tag_x_msg)
{
	tag_x = tag_x_msg.data;
}

void gimbal_track_enable_callback(const std_msgs::Bool& gimbal_track_enable_msg)
{
  gimbal_track_enabled = gimbal_track_enable_msg.data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gimbal_track_controller");
	ros::NodeHandle nh;

	gimbal_control_pitch_sub = nh.subscribe("/teamhku/gimbal_track/gimbal_control_effort_pitch", 1000, gimbalPitchControlEffortXCallback);
	gimbal_control_yaw_sub = nh.subscribe("/teamhku/gimbal_track/gimbal_control_effort_yaw", 1000, gimbalYawControlEffortYCallback);
	tag_z_sub = nh.subscribe("/teamhku/gimbal_track/tag_z", 1000, tagZCallback);
	tag_x_sub = nh.subscribe("/teamhku/gimbal_track/tag_x", 1000, tagXCallback);
	gimbal_track_enable_sub = nh.subscribe("/teamhku/gimbal_track/gimbal_track_enable", 1, gimbal_track_enable_callback );
	
	gimbal_control_service = nh.serviceClient<dji_sdk::GimbalSpeedControl>("dji_sdk/gimbal_speed_control");

	dji_sdk::GimbalSpeedControl gimbal_speed_control;

	double pitch_rate;
	double yaw_rate;

	while(ros::ok())
	{
		ros::spinOnce();

		if (fabs(tag_z) < z_threshold)
		{
			continue;
		}

		// pitch_rate = atan(gimbal_control_effort_pitch/fabs(tag_z)) * 1800 / M_PI;
		// yaw_rate = atan(gimbal_control_effort_yaw/fabs(tag_z)) * 1800 / M_PI;

		pitch_rate = gimbal_control_effort_pitch;
		yaw_rate = gimbal_control_effort_yaw;


		// pitch_rate = (pitch_rate > 200 || pitch_rate < -200) ? (pitch_rate > 0 ? 200 : -200) : pitch_rate;
		// yaw_rate = (yaw_rate > 200 || yaw_rate < -200) ? (yaw_rate > 0 ? 200 : -200) : yaw_rate;

		gimbal_speed_control.request.roll_rate = 0;
		gimbal_speed_control.request.pitch_rate = pitch_rate;
		gimbal_speed_control.request.yaw_rate = yaw_rate;

		// std::cout<< yaw_rate << std::endl;
		if(gimbal_track_enabled)
		{
			gimbal_control_service.call(gimbal_speed_control);

			if(!gimbal_speed_control.response.result)
			{
				ROS_ERROR("Cannot control gimbal");
			}
		}
		else
		{
			continue;
		}

		
	}
}