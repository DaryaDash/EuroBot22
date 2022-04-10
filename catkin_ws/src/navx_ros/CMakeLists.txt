cmake_minimum_required(VERSION 2.8.3)
project(navx_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
   ${catkin_INCLUDE_DIRS}
   navXTimeSync
)


<<<<<<< HEAD
add_executable(navx_node src/navx_node.cpp)
=======
add_executable(navx_node src/navx_node.cpp navXTimeSync/AHRS.cpp navXTimeSync/ContinuousAngleTracker.cpp navXTimeSync/InertialDataIntegrator.cpp navXTimeSync/OffsetTracker.cpp navXTimeSync/SerialIO.cpp navXTimeSync/SerialPort.cpp
)
>>>>>>> d2af8a19c80f93bc5e888b1dcb72b1b4eb88368e
target_link_libraries(navx_node ${catkin_LIBRARIES})
