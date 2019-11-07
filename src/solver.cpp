
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

#include <ceres/loss_function.h>
#include <ceres/iteration_callback.h>
#include <ceres/rotation.h>

#include "processData.hpp"
#include "Classifier.hpp"
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
    ros::init(argc, argv, "D_SLAM");
    ros::NodeHandle n;
    ros::Publisher laser_scan = n.advertise<sensor_msgs::LaserScan>("base_scan", 10);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    tf::TransformBroadcaster odom_broadcaster;

    google::InitGoogleLogging(argv[0]);

    std::string filename("../data/robotdata1.log");
    ProcessData processData(filename);

    for (int i = 0; i < 700; i++)
    {

        auto correspondedScans = processData.getCorrespondedScans(i, 10);
        // std::cout << "Correspondences out of 180 : " << correspondedScans[0].size() << std::endl;


        std::vector<bool> state(correspondedScans[0].size(), false);
        std::vector<std::vector<double>> Pij;
        std::vector<double> Weight;

        DynamicSLAM::Propensity propensity;
        propensity.calculateDistances(correspondedScans);
        propensity.calculatePropensity(Pij, Weight, state);

        randomInitialization(state);
//        plotPoints(correspondedScans[0], state);

        Classifier2 classifier;
        classifier.classify(Pij, Weight, state);

        // plotPoints(correspondedScans[0], state);

        sensor_msgs::LaserScan current_scan;
        nav_msgs::Odometry current_odom;

        std::vector<double> rawScan = processData.getRawScan(i);
        Odom odom_data = processData.getSensorOdom(i);

        convertToLaserScan(current_scan, correspondedScans[0], state, rawScan);
        current_scan.header.stamp = ros::Time::now();
        laser_scan.publish(current_scan);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = odom_data.x / 100.0;
        odom_trans.transform.translation.y = odom_data.y / 100.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(odom_data.theta);

        odom_broadcaster.sendTransform(odom_trans);

        current_odom.header.frame_id = "odom";
        current_odom.child_frame_id = "base_link";
        current_odom.pose.pose.position.x = odom_data.x / 100.0;
        current_odom.pose.pose.position.y = odom_data.y / 100.0;
        current_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_data.theta);
        current_odom.header.stamp = ros::Time::now();
        odom_pub.publish(current_odom);
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
    current_scan.angle_min = 0.0;
    current_scan.angle_max = M_PI;
    current_scan.angle_increment = M_PI / 180.0;
    current_scan.range_min = 0.0;
    current_scan.range_max = 80.0;

    std::vector<float> ranges(180);
    // std::cout << "Points got: " << correspondedScans.size() << " " << std::count(state.begin(), state.end(), true) << std::endl;
    for(int i=0; i<180; i++)
    {
        if(correspondedScans[i].id == i && state[i])
        {
            ranges[i] = rawScan[i]/100.0;
        }
        else
        {
            ranges[i] = -1.0;
        }
    }
    // std::cout << std::endl;
    current_scan.ranges = ranges;
}
