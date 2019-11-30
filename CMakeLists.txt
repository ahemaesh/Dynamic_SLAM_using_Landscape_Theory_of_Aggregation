cmake_minimum_required(VERSION 2.8)
project(D_SLAM)
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        std_msgs
        tf2_ros
        tf
        roslib
        cv_bridge
        image_transport
        )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES box_estimation
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs tf tf
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_INCLUDE_PATH} ${CERES_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Add the executable
add_executable(D_SLAM_solver src/solver.cpp src/processData.cpp src/Classifier2.cpp src/dynamicSlam.cpp)
add_dependencies(D_SLAM_solver ${catkin_EXPORTED_TARGETS})
target_link_libraries(D_SLAM_solver ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(logwriter src/logger_main.cpp)
add_dependencies(logwriter ${catkin_EXPORTED_TARGETS})
target_link_libraries(logwriter ${catkin_LIBRARIES})
