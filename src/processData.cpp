//
// Created by parv on 10/31/19.
//

#include "processData.hpp"
#include <math.h>


ProcessData::ProcessData(const std::string& filename)
{
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
                point.x = sensorOdom.x + range*cos(angle);
                point.y = sensorOdom.y + range*sin(angle);

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