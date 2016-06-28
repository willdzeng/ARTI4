cmake_minimum_required(VERSION 2.8.3)
project(dad_local_planner)

#list(APPEND CMAKE_MODULE_PATH "/home/rpradeep/Documents/mrpt/build")
#set(MRPT_DIR "/home/rpradeep/Documents/mrpt/build")
#find_package(MRPT REQUIRED base)

find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            costmap_2d
            dynamic_reconfigure
            nav_core
            nav_msgs
            # ugv_msgs
            roscpp
            tf
            rviz_debug_tools
            dynamic_obstacle
        )
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(Eigen REQUIRED)
#find_package(PCL REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    )
add_definitions(${EIGEN_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    config/DADPlannerROS.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES dad_local_planner
    CATKIN_DEPENDS
        dynamic_reconfigure
        roscpp
)

# Planner stuff
set(PLANNER_DIR src/planner)

add_library(static_obstacle_cost_evaluator ${PLANNER_DIR}/StaticObstacleCostEvaluator.cpp)
target_link_libraries(static_obstacle_cost_evaluator ${catkin_LIBRARIES} node motion_primitive)


add_library(dynamic_obstacle_cost_evaluator ${PLANNER_DIR}/DynamicObstacleCostEvaluator.cpp)
target_link_libraries(dynamic_obstacle_cost_evaluator node ${catkin_LIBRARIES})

#add_executable(doce_node ${PLANNER_DIR}/DynamicObstacleCostEvaluatorROS.cpp ${PLANNER_DIR}/DynamicObstacleCostEvaluator.cpp)
#target_link_libraries(doce_node dynamic_obstacle_cost_evaluator motionprimitivemanager ${catkin_LIBRARIES})


add_library(spatial_hash ${PLANNER_DIR}/SpatialHash.cpp)

add_library(node ${PLANNER_DIR}/Node.cpp)

add_library(motion_primitive ${PLANNER_DIR}/MotionPrimitive.cpp)

add_library(dynamics_constraints ${PLANNER_DIR}/DynamicsConstraints.cpp)

add_library(motion_primitive_manager ${PLANNER_DIR}/MotionPrimitiveManager.cpp)
target_link_libraries(motion_primitive_manager dynamics_constraints motion_primitive trajectory_space)

#add_library(heuristics ${PLANNER_DIR}/Heuristics.cpp)
#target_link_libraries(heuristics dijkstra_search)


add_library(astar_search ${PLANNER_DIR}/AStarSearch.cpp)
target_link_libraries(
  astar_search 
  spatial_hash 
  waypoint_path 
  node 
  tube_cost_evaluator 
  dynamic_obstacle_cost_evaluator 
  static_obstacle_cost_evaluator 
  motion_primitive_manager 
  ${catkin_LIBRARIES})

add_library(tube_cost_evaluator ${PLANNER_DIR}/TubeCostEvaluator.cpp)
target_link_libraries(tube_cost_evaluator node waypoint_path)

add_library(reference_trajectory ${PLANNER_DIR}/ReferenceTrajectory.cpp)
#target_link_libraries(referencetrajectory node costevaluator motionprimitive yaml-cpp)

add_library(waypoint_path ${PLANNER_DIR}/WaypointPath.cpp)

add_executable(dad_planner_node src/dad_planner_node.cpp)
target_link_libraries(dad_planner_node ${catkin_LIBRARIES} dad_local_planner)

add_library(dad_local_planner src/dad_planner.cpp src/dad_planner_ros.cpp)
target_link_libraries(dad_local_planner ${catkin_LIBRARIES} astar_search)
add_dependencies(dad_local_planner dad_local_planner_gencfg)
add_dependencies(dad_local_planner nav_msgs_gencpp)

add_library(trajectory_space ${PLANNER_DIR}/TrajectorySpace.cpp)


# test stuff
add_executable(test_WaypointPath ${PLANNER_DIR}/test_WaypointPath.cpp)
target_link_libraries(test_WaypointPath waypoint_path ${Boost_FILESYSTEM_LIBRARY}
                      ${Boost_SYSTEM_LIBRARY}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
                      )

add_executable(test_SpatialHash ${PLANNER_DIR}/test_SpatialHash.cpp)
target_link_libraries(test_SpatialHash spatial_hash ${Boost_FILESYSTEM_LIBRARY}
                      ${Boost_SYSTEM_LIBRARY}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
                      )


install(TARGETS dad_local_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)