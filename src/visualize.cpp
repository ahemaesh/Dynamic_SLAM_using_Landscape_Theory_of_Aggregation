//
// Created by akshit on 10/31/19.
//

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <tuple>
#include <algorithm>
#include "processData.hpp"

void plotPoints(std::vector<Point> pts, std::vector<bool> status)
{
    decltype(pts)::iterator minWidth, maxWidth, minHeight, maxHeight;
    std::tie(minWidth, maxWidth) = std::minmax_element(pts.begin(), pts.end(), [] (Point const& s1, Point const& s2)
                                                                                    {
                                                                                        return s1.x < s2.x;
                                                                                    });
    std::tie(minHeight, maxHeight) = std::minmax_element(pts.begin(), pts.end(), [] (Point const& s1, Point const& s2)
                                                                                    {
                                                                                        return s1.y < s2.y;
                                                                                    });
    int image_width = (int)maxWidth->x - (int)minWidth->x + 20;
    int image_height = (int)maxHeight->y - (int)minHeight->y + 20;

    float center_x = image_width / 2.0, center_y = image_height / 2.0;

    std::cout << "Image Stats: \n";
    std::cout << (int)minWidth->x << " " << (int)maxWidth->x << " ----- " << (int)minHeight->y << " " << (int)maxHeight->y << " ----- " << center_x << " " << center_y << std::endl;
    std::cout << image_width << " " << image_height << std::endl;
    int point_radius = 1, thickness = 2;

    cv::Scalar red(0, 0, 255), green(0, 255, 0);
    cv::Mat plot(image_width, image_height, CV_8UC3, cv::Scalar::all(0));

    for(size_t i=0; i<pts.size(); i++)
    {
        if(status[i])
        {
            cv::circle(plot, cv::Point(pts[i].x + center_x, pts[i].y + center_y), point_radius, green, thickness);
        }
        else
        {
            cv::circle(plot, cv::Point(pts[i].x + center_x, pts[i].y + center_y), point_radius, red, thickness);
        }
    }
    cv::imshow("Laser scan plot", plot);
    cv::waitKey(0);
}

int main()
{
    std::vector<Point> pts(18);
    std::vector<bool> status(18);

    for(size_t i=0; i<pts.size(); i++)
    {
        pts[i].x = rand() % 200 - 100;
        pts[i].y = rand() % 200 - 100;
        status[i] = (bool)(rand() % (RAND_MAX));
        std::cout << pts[i].x << " " << pts[i].y << " " << (int)status[i] << std::endl;
    }

    plotPoints(pts, status);
    return 0;
}
