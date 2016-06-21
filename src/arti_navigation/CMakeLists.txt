cmake_minimum_required(VERSION 2.8.3)
project(arti_navigation)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            arti_msgs
            geometry_msgs
        )

# find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    )

add_definitions(${EIGEN_DEFINITIONS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(arti_goal_publisher src/arti_goal_publisher.cpp)
target_link_libraries(arti_goal_publisher ${libserial_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(arti_goal_publisher arti_msgs_gencpp geometry_msgs_gencpp)

add_executable(arti_goal_publisher_node src/arti_goal_publisher.cpp)
target_link_libraries(arti_goal_publisher_node arti_goal_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(TARGETS
    arti_goal_publisher_node
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

# ## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )
