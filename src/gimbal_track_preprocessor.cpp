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

#include <apriltags/AprilTagDetections.h>

#include <cmath>

double tag_x;
double tag_y;
double tag_z;

std::string tag_detection_topic;

ros::Subscriber apriltags_pos_sub;

ros::Publisher tag_x_pub;
ros::Publisher tag_y_pub;
ros::Publisher tag_z_pub;

// void apriltagsPositionCallback(const apriltags::AprilTagDetections::ConstPtr& apriltag_pos_msg)
// {
// 	if(std::begin(apriltag_pos_msg->detections) == std::end(apriltag_pos_msg->detections))
// 	{
// 		ROS_INFO("lost tag");
// 		tag_x = 0;
//  		tag_y = 0;
//  		tag_z = 0;
// 	}
// 	else
// 	{
// 		tag_x = apriltag_pos_msg->detections[0].pose.position.x;
// 		tag_y = apriltag_pos_msg->detections[0].pose.position.y;
// 		tag_z = apriltag_pos_msg->detections[0].pose.position.z;
// 	}


// }


void apriltagsPositionCallback(const geometry_msgs::PoseArray::ConstPtr& apriltag_pos_msg)
{
	if(std::begin(apriltag_pos_msg->poses) == std::end(apriltag_pos_msg->poses))
	{
		tag_x = 0;
		tag_y = 0;
		tag_z = 0;
	}
	else
	{
		// tag_x = ((int) (apriltag_pos_msg->poses[0].position.x * 100)) / 100.0;
		// tag_y = ((int) (apriltag_pos_msg->poses[0].position.y * 100)) / 100.0;
		// tag_z = ((int) (apriltag_pos_msg->poses[0].position.z * 100)) / 100.0;

		tag_x = apriltag_pos_msg->poses[0].position.x;
		tag_y = apriltag_pos_msg->poses[0].position.y;
		tag_z = apriltag_pos_msg->poses[0].position.z;
	}


}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "gimbal_track_preprossor");

	ros::NodeHandle nh;

	// nh.param<std::string>("tag_detection_topic", tag_detection_topic, "/apriltags/detections");
	// nh.param<std::string>("tag_detection_topic", tag_detection_topic, "/dji_sdk/class_36h11/tag_detections_pose");
	nh.param<std::string>("tag_detection_topic", tag_detection_topic, "/apriltags_ros/tag_detections_pose");

	apriltags_pos_sub = nh.subscribe(tag_detection_topic, 100, apriltagsPositionCallback);

	tag_x_pub = nh.advertise<std_msgs::Float64>("/teamhku/gimbal_track/tag_x", 1);
	tag_y_pub = nh.advertise<std_msgs::Float64>("/teamhku/gimbal_track/tag_y", 1);
	tag_z_pub = nh.advertise<std_msgs::Float64>("/teamhku/gimbal_track/tag_z", 1);



	std_msgs::Float64 tag_x_msg;
	std_msgs::Float64 tag_y_msg;
	std_msgs::Float64 tag_z_msg;

	ros::Rate spin_rate(100);

	while(ros::ok())
	{
		ros::spinOnce();

		if(fabs(tag_z) < 0.0001)
		{
			tag_x_msg.data = 0;
			tag_y_msg.data = 0;
			tag_z_msg.data = 0;

			tag_x_pub.publish(tag_x_msg);
			tag_y_pub.publish(tag_y_msg);
			tag_z_pub.publish(tag_x_msg);

		}
		else
		{
			tag_x_msg.data = atan(tag_x / tag_z) * 1800 / M_PI;
			tag_y_msg.data = atan(tag_y / tag_z) * 1800 / M_PI;
			tag_z_msg.data = tag_z;

			tag_x_pub.publish(tag_x_msg);
			tag_y_pub.publish(tag_y_msg);
			tag_z_pub.publish(tag_x_msg);
		}

		

		spin_rate.sleep();

	}

}	
