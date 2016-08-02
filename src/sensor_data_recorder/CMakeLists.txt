cmake_minimum_required(VERSION 2.8.3)
project(sensor_data_recorder)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            nav_msgs
            diagnostic_msgs
            # ugv_msgs
            arti_msgs
            roscpp
            tf
            trajectory_msgs
        )

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})
# find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

#add_message_files(DIRECTORY msg FILES Num.msg)
#generate_messages(DEPENDENCIES std_msgs)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    )
#add_definitions(${EIGEN_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

# dynamic reconfigure
# generate_dynamic_reconfigure_options(
#     cfg/TrajectoryFollower.cfg
# )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES sensor_data_recorder
    CATKIN_DEPENDS
        pluginlib
        roscpp
        # ugv_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11")

add_library(sensor_data_recorder src/sensor_data_recorder.cpp)
add_dependencies(sensor_data_recorder ugv_msgs_gencfg)
add_dependencies(sensor_data_recorder ugv_msgs_gencpp)

target_link_libraries(sensor_data_recorder ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_executable(sensor_data_recorder_node src/sensor_data_recorder_node.cpp src/sensor_data_recorder.cpp)
target_link_libraries(sensor_data_recorder_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} )
# add_dependencies(sensor_data_recorder_node ugv_msgs_gencpp)
add_dependencies(sensor_data_recorder_node arti_msgs_gencpp)


install(TARGETS sensor_data_recorder
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

#install(FILES blp_plugin.xml
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
