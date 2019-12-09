# SLAM in Dynamic Environment using Landscape theory of Aggregation

## Abstract

Simultaneous Localization and Mapping (SLAM) is an essential part of any mobile robot. While the current state-of-the-art approach solves the problem in a static environment reasonably well, the performance in a dynamic environment is hit or miss. The traditional SLAM method assumes that the number of measurements from static objects would be large enough to dominate measurements from dynamic objects. We advocate for the explicit filtering of measurements from dynamic objects for better localization and mapping performance. We use the landscape theory of aggregation method to form an optimization problem. We observe the measurements for a time-window and compute weights for the optimization. We perform gradient descent to minimize the energy to classify and filter out measurements from dynamic objects. Finally, using ROS's GMapping Package we show improved SLAM output.

- [Final Report](https://github.com/ahemaesh/Dynamic_SLAM_using_Landscape_Theory_of_Aggregation/raw/master/report/Team_SLAM_Dunk_Final_Report.pdf)

- [Repo link](https://github.com/ahemaesh/Dynamic_SLAM_using_Landscape_Theory_of_Aggregation)


## Prerequisite

- [ROS Melodic](http://wiki.ros.org/melodic)

- [OpenCV](https://opencv.org/)

- [GMapping](http://wiki.ros.org/gmapping)

## Running

- Compile the package inside ROS [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
  
  ```bash
  roslaunch D_SLAM D_SLAM.launch
  ```

- To covert the data from ROS to .txt file
  
  ```bash
  roslaunch D_SLAM logger.launch
  ```

- To plot just the classification in the RVIZ
  
  ```bash
  roslaunch D_SLAM D_Scan.launch
  ```

## Results

[![Watch the video](https://img.youtube.com/vi/aJXhTr-SyeE/maxresdefault.jpg)](https://youtu.be/aJXhTr-SyeE)

[![Watch the video](https://img.youtube.com/vi/Am5gR6rEqZA/maxresdefault.jpg)](https://youtu.be/Am5gR6rEqZA)

[![Watch the video](https://img.youtube.com/vi/_B_P1TQPGAs/maxresdefault.jpg)](https://youtu.be/_B_P1TQPGAs)


## References:

[1] ​HUA Cheng-hao, DOU Li-hua, FANG Hao, FU Hao, A novel algorithm for SLAM in
dynamic environments using landscape theory of aggregation, 2016

[2] ​Robert Axelrod and D. Scott Bennett, A Landscape Theory of Aggregation, 1993
