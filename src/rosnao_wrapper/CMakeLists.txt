cmake_minimum_required(VERSION 2.8.12.2)
project(rosnao_wrapper
    DESCRIPTION "Very Simple Wrapper of Naoqi2.8 for ROS Noetic"
    )

set(sdk $ENV{HOME}/naoqi_sdk_cpp)
set(sdk_cmake ${sdk}/share/cmake)
set(sdk_lib ${sdk}/lib)

add_definitions(-w) # suppress all warnings

include(${sdk}/config.cmake) #set(CMAKE_CXX_FLAGS "-std=gnu++11 -D_GLIBCXX_USE_CXX11_ABI=0" CACHE INTERNAL "" FORCE)
include(${sdk_cmake}/qibuild/qibuild-config.cmake) # find_package(qibuild)
include(${sdk_cmake}/alcommon/alcommon-config.cmake)
include(${sdk_cmake}/alproxies/alproxies-config.cmake)
include(${sdk_cmake}/althread/althread-config.cmake)
include(${sdk_cmake}/alserial/alserial-config.cmake)
include(${sdk_cmake}/alvision/alvision-config.cmake)
include(${sdk_cmake}/alvalue/alvalue-config.cmake)
# explicitly link the boost 1.59.0 libraries from the sdk
set(BOOST_SYSTEM_LIBRARIES ${sdk_lib}/libboost_system.so.1.59.0)
set(BOOST_FILESYSTEM_LIBRARIES ${sdk_lib}/libboost_filesystem.so.1.59.0)
set(BOOST_SIGNALS_LIBRARIES ${sdk_lib}/libboost_signals.so.1.59.0)
set(BOOST_CHRONO_LIBRARIES ${sdk_lib}/libboost_chrono.so.1.59.0)
set(BOOST_THREAD_LIBRARIES ${sdk_lib}/libboost_thread.so.1.59.0)
set(BOOST_REGEX_LIBRARIES ${sdk_lib}/libboost_regex.so.1.59.0)
include(${sdk_cmake}/qi/qi-config.cmake)
include(${sdk_cmake}/icu/icu-config.cmake)
include(${sdk_cmake}/qibuild/modules/pthread-config.cmake)
include(${sdk_cmake}/qibuild/modules/dl-config.cmake)
include(${sdk_cmake}/qibuild/modules/rt-config.cmake)
# include(${sdk_cmake}/qibuild/modules/zlib-config.cmake) # may cause warnings about unsafe paths
set(ZLIB_LIBRARIES ${sdk_lib}/libz.so.1)

message(INFO ${BOOST_SYSTEM_LIBRARIES})
set(ROSNAO_WRAPPER_LIBRARIES
    ${ALCOMMON_LIBRARIES}
    ${ALPROXIES_LIBRARIES}
    ${ALTHREAD_LIBRARIES}
    ${ALSERIAL_LIBRARIES}
    ${ALVISION_LIBRARIES}
    ${ALVALUE_LIBRARIES}
    ${BOOST_SYSTEM_LIBRARIES}
    ${BOOST_FILESYSTEM_LIBRARIES}
    ${BOOST_SIGNALS_LIBRARIES}
    ${BOOST_CHRONO_LIBRARIES}
    ${BOOST_THREAD_LIBRARIES}
    ${BOOST_REGEX_LIBRARIES}
    ${QI_LIBRARIES}
    ${ICU_LIBRARIES}
    ${PTHREAD_LIBRARIES}
    ${DL_LIBRARIES}
    ${RT_LIBRARIES}
    ${ZLIB_LIBRARIES} #cmake warning for libz.so being hidden by files in sdk_lib can be ignored
)


set(ROSNAO_WRAPPER_INCLUDE_DIRS 
    ${sdk}/include
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/../rosnao_common/include
)

set(EXE_IMAGEPUB ${PROJECT_NAME}_imagepub)
add_executable(${EXE_IMAGEPUB} src/image_publisher.cpp)
target_include_directories(${EXE_IMAGEPUB} PUBLIC ${ROSNAO_WRAPPER_INCLUDE_DIRS})
target_link_libraries(${EXE_IMAGEPUB} PUBLIC ${ROSNAO_WRAPPER_LIBRARIES})