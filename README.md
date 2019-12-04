# SLAM in Dynamic Environment using Landscape theory of Aggregation

## Introduction

Majority of the traditional SLAM problems have been addressed for static
environments. Addressing SLAM problem in dynamic environments is quite limited. With
the advent of self-driving cars and factory automation robots, impact of robot localization
in dynamic environment has become more profound. With this project, we expect to
implement the existing Dynamic SLAM approach described in [1] and improve upon this
approach. (eg additional cost functions for getting better compatibility score)

Proposed approach utilizes theory originally proposed in political science journal
titled Landscape Theory of Aggregation[2] of all places. While traditional SLAM
algorithm assumes that all the features are static and hopes that high number of static
features will counter the misleading measurements provided by dynamic features,
Landscape Theory of Aggregation will allow us to classify the features into static and
dynamic set and will enable explicit use of just stationary features in our SLAM pipeline
resulting in the improved map and localization output. Also, the structure of the
proposed approach allows for integration with any existing SLAM system to get
improved results in dynamic environments.

Novelty of the paper is in classifying features in static and dynamic class. The
classification is performed by finding a least squares optimization solution to set of
correspondences that are to be classified as static or dynamic. Also, proposed
approach requires traditional SLAM once classification is performed to get robot
trajectory and map. We plan to implement either Lidar Odometry and Mapping(LOAM)
or Extended Kalman Filter (EKF) or Factor Graph methods described in the class to
work along with the classifier.

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

## References:

[1] ​HUA Cheng-hao, DOU Li-hua, FANG Hao, FU Hao, A novel algorithm for SLAM in
dynamic environments using landscape theory of aggregation, 2016

[2] ​Robert Axelrod and D. Scott Bennett, A Landscape Theory of Aggregation, 1993
