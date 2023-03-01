cmake_minimum_required(VERSION 2.8.12.2)
project(ROSNAO
    DESCRIPTION "Very Simple Wrapper of Naoqi2.8 for ROS Noetic"
    )

set(sdk ${CMAKE_CURRENT_LIST_DIR}/../../naoqi_sdk_cpp)
set(sdk_cmake ${sdk}/share/cmake)
set(sdk_lib ${sdk}/lib)

include(${sdk}/config.cmake) #set(CMAKE_CXX_FLAGS "-std=gnu++11 -D_GLIBCXX_USE_CXX11_ABI=0" CACHE INTERNAL "" FORCE)
include(${sdk_cmake}/qibuild/qibuild-config.cmake) # find_package(qibuild)
include(${sdk_cmake}/alcommon/alcommon-config.cmake)
include(${sdk_cmake}/alproxies/alproxies-config.cmake)
include(${sdk_cmake}/althread/althread-config.cmake)
include(${sdk_cmake}/alserial/alserial-config.cmake)
include(${sdk_cmake}/alvision/alvision-config.cmake)
include(${sdk_cmake}/alvalue/alvalue-config.cmake)
include(${sdk_cmake}/boost_signals/boost_signals-config.cmake)
include(${sdk_cmake}/boost_system/boost_system-config.cmake)
include(${sdk_cmake}/boost_filesystem/boost_filesystem-config.cmake)
include(${sdk_cmake}/icu/icu-config.cmake)
include(${sdk_cmake}/qibuild/modules/pthread-config.cmake)
include(${sdk_cmake}/qibuild/modules/dl-config.cmake)
include(${sdk_cmake}/qibuild/modules/rt-config.cmake)
include(${sdk_cmake}/qibuild/modules/zlib-config.cmake)

set(ROSNAO_LIBRARIES
    ${ALCOMMON_LIBRARIES}
    ${ALPROXIES_LIBRARIES}
    ${ALTHREAD_LIBRARIES}
    ${ALSERIAL_LIBRARIES}
    ${ALVISION_LIBRARIES}
    ${ALVALUE_LIBRARIES}
    ${BOOST_FILESYSTEM_LIBRARIES}
    ${BOOST_SYSTEM_LIBRARIES}
    ${BOOST_SIGNALS_LIBRARIES}
    ${ICU_LIBRARIES}
    ${PTHREAD_LIBRARIES}
    ${DL_LIBRARIES}
    ${RT_LIBRARIES}
    ${ZLIB_LIBRARIES} #cmake warning for libz.so being hidden by files in sdk_lib can be ignored
)

set(ROSNAO_INCLUDE_DIRS 
    ${sdk}/include
    ${CMAKE_CURRENT_LIST_DIR}/include
)

add_executable(rosnao src/main.cpp)
target_include_directories(rosnao PUBLIC ${ROSNAO_INCLUDE_DIRS})
target_link_libraries(rosnao PUBLIC ${ROSNAO_LIBRARIES})