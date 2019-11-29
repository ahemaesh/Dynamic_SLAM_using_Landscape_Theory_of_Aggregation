#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

#include "processData.hpp"
#include "Classifier2.hpp"
#include "dynamicSlam.hpp"
#include "visualize.hpp"
#include <ctime>
#include <cstdlib>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

void randomInitialization(std::vector<bool> &state2);
void convertToLaserScan(sensor_msgs::LaserScan &current_scan, std::vector<Point> correspondedScans, std::vector<bool> state, std::vector<double> rawScan);

int main(int argc, char** argv){
    ros::init(argc, argv, "D_SLAM_solver");
    ros::NodeHandle n;
    ros::Rate rate(30);

    int windowSize = 20;

    ros::Publisher laser_scan = n.advertise<sensor_msgs::LaserScan>("scan", 10);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    ros::Publisher odom_raw_pub = n.advertise<nav_msgs::Odometry>("odom_raw", 10);
    tf::TransformBroadcaster odom_broadcaster;


    std::string filename("/data/test_log.log");
    ProcessData processData(filename);
    std::cout << "Data Processed!!! " << processData.getScanCount() << std::endl;

    for (int i = 0; i <= processData.getScanCount() - windowSize; i++)
    {

        auto correspondedScans = processData.getCorrespondedScans(i, windowSize);
        // std::cout << "Correspondences out of 180 : " << correspondedScans[0].size() << std::endl;


        std::vector<bool> state(correspondedScans[0].size(), false);
        std::vector<std::vector<double>> Pij;
        std::vector<double> Weight;

        DynamicSLAM::Propensity propensity;
        propensity.calculateDistances(correspondedScans);
        propensity.calculatePropensity(Pij, Weight, state);

//        randomInitialization(state);
//        plotPoints(correspondedScans[0], state);

        Classifier2 classifier;
        classifier.classify(Pij, Weight, state);

        //plotPoints(correspondedScans[0], state);

        // std::fill(state.begin(), state.end(), false);

        sensor_msgs::LaserScan current_scan;
        nav_msgs::Odometry current_odom;

        std::vector<double> rawScan = processData.getRawScan(i);
        Odom odom_data = processData.getSensorOdom(i);

        convertToLaserScan(current_scan, correspondedScans[0], state, rawScan);

        current_scan.header.stamp = ros::Time::now();
        laser_scan.publish(current_scan);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = odom_data.x / 100.0;
        odom_trans.transform.translation.y = odom_data.y / 100.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(odom_data.theta);
        odom_trans.header.stamp = ros::Time::now();

        odom_broadcaster.sendTransform(odom_trans);
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = "odom_raw";
        odom_broadcaster.sendTransform(odom_trans);

        current_odom.header.frame_id = "odom";
        current_odom.child_frame_id = "base_link";
        current_odom.pose.pose.position.x = odom_data.x / 100.0;
        current_odom.pose.pose.position.y = odom_data.y / 100.0;
        current_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_data.theta);
        current_odom.header.stamp = ros::Time::now();
        odom_pub.publish(current_odom);

        current_odom.header.frame_id = "map";
        current_odom.child_frame_id = "base_link_raw";
        odom_raw_pub.publish(current_odom);

        rate.sleep();
    }
    return 0;
}

void randomInitialization(std::vector<bool> &state2)
{
    srand(time(nullptr));

    for (auto && i : state2)
    {
        i = rand() % 2;
    }
}

void convertToLaserScan(sensor_msgs::LaserScan &current_scan, std::vector<Point> correspondedScans, std::vector<bool> state, std::vector<double> rawScan)
{
    current_scan.header.frame_id = "laser";
    current_scan.angle_min = -M_PI/2.0;
    current_scan.angle_max = M_PI/2.0;
    current_scan.angle_increment = M_PI / 180.0;
    current_scan.range_min = 0.0;
    current_scan.range_max = 80.0;

    uint32_t ranges_size = std::ceil((current_scan.angle_max - current_scan.angle_min) / current_scan.angle_increment);
    current_scan.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

    // std::cout << "Points got: " << correspondedScans.size() << " " << std::count(state.begin(), state.end(), true) << std::endl;

    for(size_t i = 0; i < correspondedScans.size(); i++)
    {
        if (!state[i])
        {
            current_scan.ranges[correspondedScans[i].id] = rawScan[correspondedScans[i].id]/100.0;
        }
    }

    // std::cout << std::endl;
}
