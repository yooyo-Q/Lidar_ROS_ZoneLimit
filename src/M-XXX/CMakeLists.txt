cmake_minimum_required(VERSION 2.8.3)
project(mxxx)

# Build ROS-independent library.
find_package(console_bridge REQUIRED)
include_directories(include ${console_bridge_INCLUDE_DIRS})
add_library(MXXX src/MXXX.cpp src/lim.cpp)
target_link_libraries(MXXX ${console_bridge_LIBRARIES})


# Regular catkin package follows.
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
catkin_package(CATKIN_DEPENDS roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(mxxx_ros_node src/mxxx_ros_node.cpp)
target_link_libraries(mxxx_ros_node MXXX ${catkin_LIBRARIES})


install(TARGETS MXXX mxxx_ros_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
