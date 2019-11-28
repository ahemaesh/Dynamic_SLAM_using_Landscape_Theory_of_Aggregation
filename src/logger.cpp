//
// Created by akshit on 11/28/19.
//

#include <iostream>
#include <fstream>
#include <string.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <logger.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

logger::logger(ros::NodeHandle &nh) : nh(nh)
{
	std::string nodeName = (ros::this_node::getName());
	std::string odom_topic, lscan_topic, logfile_name;

	if(!this->nh.getParam(nodeName + "/odom_topic", odom_topic) || !this->nh.getParam(nodeName + "/lscan_topic", lscan_topic) || !this->nh.getParam(nodeName + "/logfile_name", logfile_name))
	{
		std::cerr << "Please enter all topic names in launch file!\n";
		exit(-1);
	}

	logFile.open(ros::package::getPath("D_SLAM") + logfile_name);
	
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(this->nh, odom_topic, 1);
	message_filters::Subscriber<sensor_msgs::LaserScan> lscan_sub(this->nh, lscan_topic, 1);
	TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::LaserScan> sync(odom_sub, lscan_sub, 10);
	sync.registerCallback(boost::bind(&logger::cvtToLog, _1, _2), this);

	// this->odom_sub = this->nh.subscribe(odom_topic, 10, &logger::odomCallback, this);
	// this->lscan_sub = this->nh.subscribe(lscan_topic, 10, &logger::lscanCallback, this);
}

void logger::cvtToLog(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::LaserScan::ConstPtr& lscan)
{
	std::cout << "Callback called \n";
}