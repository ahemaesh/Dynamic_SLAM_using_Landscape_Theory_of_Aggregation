//
// Created by akshit on 11/28/19.
//

#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <logger.cpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "log_writer");
	ros::NodeHandle nh;

	logger writer(nh);
	ros::spin();

	return 0;
}