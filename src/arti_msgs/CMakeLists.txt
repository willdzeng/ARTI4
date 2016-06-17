cmake_minimum_required(VERSION 2.8.3)
project(arti_msgs)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            rosserial_arduino
            rosserial_client
            serial
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

# rosserial_generate_ros_lib(
#   PACKAGE rosserial_arduino
#   SCRIPT make_libraries.py
# )

# rosserial_configure_client(
#   DIRECTORY firmware
#   TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
# )

# add_library(arti_msgs src/arti_msgs.cpp)
# target_link_libraries(arti_msgs ${libserial_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# add_executable(arti_base src/arti_msgs.cpp)
# target_link_libraries(arti_base arti_msgs ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS 
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)