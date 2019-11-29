//
// Created by akshit on 11/28/19.
//

#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/tf.h>
#include <fstream>


std::ofstream logFile;
int data_factor = -1;
nav_msgs::Odometry current_odom;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	current_odom = *msg;
}

void lscanCallback(const sensor_msgs::LaserScan::ConstPtr& lscan)
{
	tf::Quaternion q (current_odom.pose.pose.orientation.x,
			  current_odom.pose.pose.orientation.y,
			  current_odom.pose.pose.orientation.z,
			  current_odom.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	std::string output = "";
	
	output = "L " + std::to_string(current_odom.pose.pose.position.x * 100.0) + " " + std::to_string(current_odom.pose.pose.position.y * 100.0) + " " + std::to_string(yaw) + " " + std::to_string((current_odom.pose.pose.position.x + 0.31*cos(yaw)) * 100.0) + " " + std::to_string((current_odom.pose.pose.position.y + 0.31*sin(yaw)) * 100.0) + " " + std::to_string(yaw);
	for(int i=0; i<180; i++)
	{
		float curr_val = lscan->ranges[i];
		if(std::to_string(std::min(curr_val, 80.0f)) == "nan")
		{
			output = output + " " + std::to_string(80.0f * 100.0);
		}
		else
		{
			output = output + " " + std::to_string(curr_val * 100.0);
		}
	}
		//converting long double stamp is not available hence we are just using the sequence as time!
	output = output + " " + std::to_string(lscan->header.seq) + "\n";
	logFile << output;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "log_writer");
	ros::NodeHandle nh;
	std::string node_name = ros::this_node::getName();
	std::string odom_topic, lscan_topic, logfile_name;
	
	if(!nh.getParam(node_name + "/odom_topic", odom_topic)     ||
	   !nh.getParam(node_name + "/lscan_topic", lscan_topic)   ||
	   !nh.getParam(node_name + "/logfile_name", logfile_name) ||
	   !nh.getParam(node_name + "/data_factor", data_factor)
	   )
	{
		std::cerr << "Please specify all launch file params \n";
	}

	logFile.open(ros::package::getPath("D_SLAM") + logfile_name);

	ros::Subscriber odom_sub = nh.subscribe(odom_topic, 10, odomCallback);
	ros::Subscriber lscan_sub = nh.subscribe(lscan_topic, 10, lscanCallback);

	while(ros::ok())
	{
		ros::spinOnce();
	}
	logFile.close();
	return 0;
}
