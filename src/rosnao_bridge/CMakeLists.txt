cmake_minimum_required(VERSION 2.8.3)
project(rosnao_bridge)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    nav_msgs
    sensor_msgs
    std_msgs
    geometry_msgs
    roscpp
    rospy
    cv_bridge 
    image_transport
)
find_package(OpenCV 4.2.0 REQUIRED)
find_package(Boost REQUIRED)


catkin_package( CATKIN_DEPENDS 
    message_runtime 
    message_generation 
    sensor_msgs 
    std_msgs 
    geometry_msgs 
    nav_msgs 
    roscpp 
    rospy 
    cv_bridge 
    image_transport
)

set(ROSNAO_BRIDGE_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ../rosnao_common/include
)
set(ROSNAO_BRIDGE_LIBRARIES
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    rt # for interprocess
)

add_executable(image_relay src/image_relay.cpp)
target_include_directories(image_relay PUBLIC ${ROSNAO_BRIDGE_INCLUDE_DIRS})
target_link_libraries(image_relay ${ROSNAO_BRIDGE_LIBRARIES})

add_executable(test_motion src/test_motion.cpp)
target_include_directories(test_motion PUBLIC ${ROSNAO_BRIDGE_INCLUDE_DIRS})
target_link_libraries(test_motion ${ROSNAO_BRIDGE_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)