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
int msg_counter = -1;
int data_factor = -1;
nav_msgs::Odometry current_odom;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	current_odom = *msg;
	msg_counter++;
}

void lscanCallback(const sensor_msgs::LaserScan::ConstPtr& lscan)
{
	if(msg_counter == -1)
		return;
	tf::Quaternion q (current_odom.pose.pose.orientation.x,
			  current_odom.pose.pose.orientation.y,
			  current_odom.pose.pose.orientation.z,
			  current_odom.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	std::string output = "";
	
	if(msg_counter % data_factor == 0)
	{
		output = "L " + std::to_string(current_odom.pose.pose.position.x) + " " + std::to_string(current_odom.pose.pose.position.y) + " " + std::to_string(yaw) + " " + std::to_string(current_odom.pose.pose.position.x) + " " + std::to_string(current_odom.pose.pose.position.y) + " " + std::to_string(yaw);
		for(int i=0; i<180; i++)
		{
			float curr_val = lscan->ranges[i];
			if(std::to_string(std::min(curr_val, 80.0f)) == "nan")
			{
				output = output + " " + std::to_string(80.0f);
			}
			else
			{
				output = output + " " + std::to_string(curr_val);
			}
		}
		//converting long double stamp is not available hence we are just using the sequence as time!
		output = output + " " + std::to_string(lscan->header.seq) + "\n";
		msg_counter = -1;
	}
	else
	{
		output = "O " + std::to_string(current_odom.pose.pose.position.x) + " " + std::to_string(current_odom.pose.pose.position.y) + " " + std::to_string(yaw) + " " + std::to_string(current_odom.header.seq) + "\n";
	}
	logFile << output;
}

void cvtToLog(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::LaserScanConstPtr& lscan)
{
	ROS_INFO("Callback called \n");
	// ROS_INFO("Time: %d %d \n", odom->header.stamp, lscan->header.stamp);
	tf::Quaternion q (odom->pose.pose.orientation.x,
			  odom->pose.pose.orientation.y,
			  odom->pose.pose.orientation.z,
			  odom->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	std::string output = "";
	output = "L " + std::to_string(odom->pose.pose.position.x) + " " + std::to_string(odom->pose.pose.position.y) + " " + std::to_string(yaw) + " " + std::to_string(odom->pose.pose.position.x) + " " + std::to_string(odom->pose.pose.position.y) + " " + std::to_string(yaw);
	for(int i=0; i<180; i++)
	{
		float curr_val = lscan->ranges[i];
		if(std::to_string(std::min(curr_val, 80.0f)) == "nan")
		{
			output = output + " " + std::to_string(80.0f);
		}
		else
		{
			output = output + " " + std::to_string(curr_val);
		}
	}
	//converting long double stamp is not available hence we are just using the sequence as time!
	output += std::to_string(lscan->header.seq) + "\n";
	logFile << output;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "log_writer");
	ros::NodeHandle nh;
	std::string odom_topic = "odometry/filtered";
	std::string lscan_topic = "scan";
	std::string logfile_name = "/data/test_log.txt";
	data_factor = 4.0;
	std::string nodeName = (ros::this_node::getName());

	logFile.open(ros::package::getPath("D_SLAM") + logfile_name);

	// message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, odom_topic, 1);
	// message_filters::Subscriber<sensor_msgs::LaserScan> lscan_sub(nh, lscan_topic, 1);

	// typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::LaserScan> syncPolicy;
	// message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), odom_sub, lscan_sub);
	// sync.registerCallback(boost::bind(cvtToLog, _1, _2));

	ros::Subscriber odom_sub = nh.subscribe(odom_topic, 10, odomCallback);
	ros::Subscriber lscan_sub = nh.subscribe(lscan_topic, 10, lscanCallback);

	while(ros::ok())
	{
		ros::spinOnce();
	}
	logFile.close();
	return 0;
}
