//
// Created by parv on 10/31/19.
//

#ifndef D_SLAM_PROCESSDATA_HPP
#define D_SLAM_PROCESSDATA_HPP

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

struct Point
{
    double x;
    double y;
    int id;
};

struct Odom
{
    double x;
    double y;
    double theta;
    double time;
};

class ProcessData
{
public:
    ProcessData(const std::string& filename);
    std::vector<std::vector<Point>> getCorrespondedScans(int timeStamp, int windowSize);
    std::vector<double> getRawScan(int itr);
    Odom getSensorOdom(int itr);

private:
    void readLog();

    std::ifstream logFile;
    std::vector<std::vector<double>> rawScans;
    std::vector<std::vector<Point>> scans;
    std::vector<Odom> odometryVehicle;
    std::vector<Odom> odometrySensor;
    double matchRadius;
};


#endif //D_SLAM_PROCESSDATA_HPP
