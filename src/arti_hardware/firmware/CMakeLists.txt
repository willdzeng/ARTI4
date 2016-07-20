# cmake_minimum_required(VERSION 2.8.3)

# include_directories(${ROS_LIB_DIR})

# # Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)
# set(SabertoothPath ${CMAKE_CURRENT_SOURCE_DIR}/libraries/Sabertooth)

# link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries)


# generate_arduino_firmware(blink_test
#   SRCS blink_test.cpp ${ROS_LIB_DIR}/time.cpp
#   BOARD mega2560
#   PORT /dev/ttyACM0
# )

# # set(SabertoothSimplified_RECURSE True)
# # generate_arduino_firmware(one_motor_test
# #   SRCS one_motor_test.cpp ${ROS_LIB_DIR}/time.cpp
# #   LIBS ${CMAKE_CURRENT_SOURCE_DIR}/libraries/SabertoothSimplified
# #   BOARD mega2560
# #   PORT /dev/ttyACM0
# # )

# set(Sabertooth_RECURSE True)
# generate_arduino_firmware(protocol
#   SRCS protocol.cpp ${ROS_LIB_DIR}/time.cpp
#   LIBS ${CMAKE_CURRENT_SOURCE_DIR}/libraries/Sabertooth
#   BOARD mega2560
#   PORT /dev/ttyACM0
# )