cmake_minimum_required(VERSION 2.8.3)
project(interactive_path_publisher)

find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            std_msgs
            nav_msgs
            roscpp
            tf
        )
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(Eigen REQUIRED)
#find_package(PCL REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    )
add_definitions(${EIGEN_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES interactive_path_publisher 
    CATKIN_DEPENDS
        roscpp
)

add_library(ipp src/IPP.cpp)
target_link_libraries(ipp ${catkin_LIBRARIES})

add_executable(IPPNode src/IPPNode.cpp)
target_link_libraries(IPPNode ipp ${catkin_LIBRARIES})