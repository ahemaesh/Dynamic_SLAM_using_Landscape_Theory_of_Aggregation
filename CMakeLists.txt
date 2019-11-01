cmake_minimum_required(VERSION 2.8)
project(D_SLAM)
set(CMAKE_CXX_STANDARD 11)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${CMAKE_INCLUDE_PATH} ${CERES_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Add the executable
add_executable(solver src/solver.cpp src/processData.cpp src/Classifier.cpp src/dynamicSlam.cpp)
target_link_libraries(solver ${CERES_LIBRARIES} ${OpenCV_LIBRARIES})

