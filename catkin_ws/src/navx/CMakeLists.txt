cmake_minimum_required(VERSION 3.0.2)
project(navx)
add_definitions(-std=c++11 -Wall -Wextra -Wno-switch)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

file(GLOB SOURCES
	"*.h"
	"*.cpp"
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
   LIBRARIES navx
  CATKIN_DEPENDS roscpp
#  DEPENDS other non-ROS libs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
  src/navx.cpp
)
 

add_executable(navx_node src/navx.cpp)
target_link_libraries(navx_node ${catkin_LIBRARIES})
add_dependencies(navx navx_cpp)
include_directories("${PROJECT_SOURCE_DIR}/include")
add_executable(NavXTimeStamp ${SOURCES})
target_link_libraries(NavXTimeStamp  ${CMAKE_THREAD_LIBS_INIT})

#install(
 # TARGETS ${PROJECT_NAME}
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)

