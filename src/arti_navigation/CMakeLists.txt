cmake_minimum_required(VERSION 2.8.3)
project(arti_navigation)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            arti_msgs
            geometry_msgs
            std_srvs
            tf
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

add_library(multi_goal_publisher src/multi_goal_publisher.cpp)
target_link_libraries(multi_goal_publisher ${libserial_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(multi_goal_publisher arti_msgs_gencpp geometry_msgs_gencpp)

add_executable(multi_goal_publisher_node src/multi_goal_publisher.cpp)
target_link_libraries(multi_goal_publisher_node multi_goal_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(costmap_cleanner src/costmap_cleanner.cpp)
target_link_libraries(costmap_cleanner ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(stair_climb src/stair_climb.cpp)
target_link_libraries(stair_climb ${libserial_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(stair_climb arti_msgs_gencpp geometry_msgs_gencpp)

add_executable(stair_climb_node src/stair_climb.cpp)
target_link_libraries(stair_climb_node stair_climb ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
        multi_goal_publisher_node
        costmap_cleanner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# ## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )
