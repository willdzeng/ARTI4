cmake_minimum_required(VERSION 2.8)
project(pure_pursuit_controller)


find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    rospy
    rosbash
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf
    roscpp_serialization
    )

catkin_package(
    CATKIN_DEPENDS
    roscpp
    roscpp_serialization
    rospy
    rosbash
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

add_executable(pure_pursuit_controller_node src/pure_pursuit_controller_node.cpp src/pure_pursuit_controller.cpp)

target_link_libraries(pure_pursuit_controller_node
   ${catkin_LIBRARIES}
 )

#install(
#    TARGETS
#        pure_pursuit_controller_node
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)
