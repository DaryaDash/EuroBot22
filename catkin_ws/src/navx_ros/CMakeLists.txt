cmake_minimum_required(VERSION 3.0.2)
project(navx_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

###########
## Build ##
###########

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  navXTimeSync
)


add_executable(navx_node
	src/navx_node.cpp
	navXTimeSync/AHRS.cpp
	navXTimeSync/ContinuousAngleTracker.cpp
	navXTimeSync/InertialDataIntegrator.cpp
	navXTimeSync/OffsetTracker.cpp
	navXTimeSync/SerialIO.cpp
	navXTimeSync/SerialPort.cpp
)
target_link_libraries(navx_node ${catkin_LIBRARIES})
