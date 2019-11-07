//
// Created by parv on 10/31/19.
//

#include "processData.hpp"
#include <assert.h>
#include <cmath>

ProcessData::ProcessData(const std::string& filename)
{
    this->matchRadius = 100.0;
    logFile.open(filename);
    readLog();
}

void ProcessData::readLog()
{
    char measurementType;

    while(logFile >> measurementType)
    {
        if (measurementType == 'O')
        {
            double x, y, theta, time;
            logFile >> x >> y >> theta >> time;
        }
        else if (measurementType == 'L')
        {
            Odom vehicleOdom;
            Odom sensorOdom;

            logFile >> vehicleOdom.x >> vehicleOdom.y >> vehicleOdom.theta;
            logFile >> sensorOdom.x >> sensorOdom.y >> sensorOdom.theta;

            std::vector<double> rawScan;
            std::vector<Point> scan;

            for (int i = 0; i < 180; ++i)
            {
                double range;
                Point point;
                logFile >> range;

                auto angle = sensorOdom.theta + (i+1)*M_PI/180.0 - M_PI/2;
                point.x = 0.25 + sensorOdom.x + range*cos(angle);   //Added the laser offset from assignment 1 @akgandhi
                point.y = sensorOdom.y + range*sin(angle);
                point.id = i;

                rawScan.emplace_back(range);
                scan.emplace_back(point);
            }

            logFile >> vehicleOdom.time;
            sensorOdom.time = vehicleOdom.time;

            this->rawScans.emplace_back(rawScan);
            this->scans.emplace_back(scan);
            this->odometryVehicle.emplace_back(vehicleOdom);
            this->odometrySensor.emplace_back(sensorOdom);
        }
    }
}

std::vector<std::vector<Point>> ProcessData::getCorrespondedScans(int timeStamp, int windowSize)
{
    std::vector<std::vector<Point>> correspondedScans(windowSize, std::vector<Point>());

    assert(timeStamp + windowSize <= int(this->scans.size()));

    for (int i = 0; i < int(this->scans[timeStamp].size()); ++i)
    {
        std::vector<Point> correspondedPoints;
        bool matchFoundForEachTimeStamp = true;
        auto pointA = this->scans[timeStamp][i];

        correspondedPoints.emplace_back(pointA);

        for (int a = timeStamp + 1; a < timeStamp + windowSize; a++)
        {
            Point matchingPoint{0.0, 0.0, -1};
            double minDist = -1.0;

            for (int j = 0; j < int(this->scans[a].size()); ++j)
            {
                auto pointB = this->scans[a][j];
                auto currDist = sqrt(pow(pointA.x - pointB.x, 2) + pow(pointA.y - pointB.y, 2));

                if ((minDist == -1.0 || currDist < minDist) && currDist < this->matchRadius)
                {
                    minDist = currDist;
                    matchingPoint = pointB;
                }

            }
            if (minDist != -1.0)
            {
                correspondedPoints.emplace_back(matchingPoint);
            }
            else
            {
                matchFoundForEachTimeStamp = false;
                break;
            }
        }

        if (matchFoundForEachTimeStamp)
        {
            for(int a = 0; a < windowSize; a++)
            {
                correspondedScans[a].emplace_back(correspondedPoints[a]);
            }
        }
    }

    return correspondedScans;
}

std::vector<double> ProcessData::getRawScan(int itr)
{
    return this->rawScans[itr];
}

Odom ProcessData::getSensorOdom(int itr)
{
    return this->odometrySensor[itr];
}
