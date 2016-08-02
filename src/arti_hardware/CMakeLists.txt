cmake_minimum_required(VERSION 2.8.3)
project(arti_hardware)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            serial
            message_generation
            arti_msgs
            nav_msgs
            realtime_tools
            tf
        )
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    )

add_definitions(${EIGEN_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
    # INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS
)

add_library(arti_hardware src/arti_hardware.cpp)
target_link_libraries(arti_hardware ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(arti_hardware arti_msgs_gencpp)


add_executable(arti_base src/arti_hardware.cpp)
target_link_libraries(arti_base arti_hardware ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS 
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)