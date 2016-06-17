cmake_minimum_required(VERSION 2.8.3)
project(arti_control)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            arti_msgs
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

add_library(arti_control src/arti_control.cpp)
target_link_libraries(arti_control ${libserial_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(arti_control arti_msgs_gencpp geometry_msgs_gencpp)

add_executable(arti_control_node src/arti_control.cpp)
target_link_libraries(arti_control_node arti_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(TARGETS
        arti_control
        arti_control_node
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
