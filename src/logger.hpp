//
// Created by akshit on 11/28/19.
//

#ifndef D_SLAM_LOGGER_HPP
#define D_SLAM_LOGGER_HPP

#include <string>
#include <iostream>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

class logger
{
public:
    explicit logger(ros::NodeHandle &nh);

private:
    // void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    // void lscanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void cvtToLog(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::LaserScan::ConstPtr& lscan);

    ros::NodeHandle nh;
    std::ifstream logFile;
    // nav_msgs::Odometry current_odom;
    // ros::Subscriber odom_sub, lscan_sub;
};

#endif //D_SLAM_LOGGER_HPP