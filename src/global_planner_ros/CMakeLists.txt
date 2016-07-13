cmake_minimum_required(VERSION 2.8.3)
project(global_planner_ros)

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        roscpp
        pluginlib
        actionlib
        dynamic_reconfigure
        message_generation
        nav_core
        tf
        base_local_planner
        costmap_2d
        global_planner
)
find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/GlobalPlannerROS.cfg
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        dynamic_reconfigure
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

# global_planner_ros
add_library(global_planner_ros src/global_planner_ros.cpp)
target_link_libraries(global_planner_ros ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(global_planner_ros global_planner_gencfg)
add_dependencies(global_planner_ros geometry_msgs_gencpp)

add_executable(global_planner_node src/global_planner_node.cpp)
target_link_libraries(global_planner_node global_planner_ros)
# set_target_properties(global_planner_node PROPERTIES OUTPUT_NAME global_planner_ros)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    )

install(
    TARGETS
        global_planner_ros
        global_planner_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
