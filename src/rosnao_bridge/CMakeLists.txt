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

add_executable(rosnao_bridger_test 
    src/test.cpp)
target_include_directories(rosnao_bridger_test PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ../../sdk/rosnao_common/include
)
target_link_libraries(rosnao_bridger_test 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
