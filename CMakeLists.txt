cmake_minimum_required(VERSION 3.0.2)
project(opticalflow_ros)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roslib
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
#INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS roscpp cv_bridge sensor_msgs std_msgs geometry_msgs
# DEPENDS system_lib OPENCV
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(opticalflow_ros src/opticalflow_ros.cpp)
target_link_libraries(opticalflow_ros ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(opticalflow_ros ${catkin_EXPORTED_TARGETS})