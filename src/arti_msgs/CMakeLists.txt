cmake_minimum_required(VERSION 2.8.3)
project(arti_msgs)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            message_generation
            # libserial
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

add_message_files(
  FILES
  DiffOdom.msg
  DiffCmd.msg
  Ultrasound.msg
  Temperature.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    # INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS
)

install(TARGETS 
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)