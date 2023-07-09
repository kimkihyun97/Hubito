cmake_minimum_required(VERSION 2.8.3)
project(human_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    message_filters
    message_generation
    nlink_parser
    pcl_conversions
    pcl_ros
##    rosserial
##    rosserial_arduino
    )

add_message_files(
    FILES
    UWB.msg
    ObjectPosition.msg
    ObjectPositions.msg
    filtered_data.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    )

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES nlink
    CATKIN_DEPENDS message_runtime ##rosserial rosserial_arduino
    #  DEPENDS system_lib
    )

include_directories(
    include
    include/${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )

link_directories(
    ${PCL_LIBRARY_DIRS}
    )

add_definitions(
    ${PCL_DEFINITIONS}
    )


add_executable(uwb_sensing
    src/uwb_sensing.cpp
    src/pid.cpp
    src/EKF.cpp
    )

target_link_libraries(uwb_sensing
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}

    )

add_dependencies(uwb_sensing
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)

add_executable(uwb_timestamp
  src/uwb_timestamp.cpp
)

target_link_libraries(uwb_timestamp
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )

add_dependencies(uwb_timestamp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

