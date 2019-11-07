cmake_minimum_required(VERSION 2.8)
project(D_SLAM)
set(CMAKE_CXX_STANDARD 11)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        std_msgs
        tf2_ros
        tf
        )

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_INCLUDE_PATH} ${CERES_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Add the executable
add_executable(solver src/solver.cpp src/processData.cpp src/Classifier.cpp src/Classifier2.cpp src/dynamicSlam.cpp)
add_dependencies(solver ${catkin_EXPORTED_TARGETS})
target_link_libraries(solver ${CERES_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})