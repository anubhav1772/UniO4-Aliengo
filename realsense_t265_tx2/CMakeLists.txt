cmake_minimum_required(VERSION 2.8.3)
#project(unitree_legged_sdk)
# T265 tracking camera
project(realsense_t265_tx2)

#find H5Cpp library
#find_package(HDF5 REQUIRED CXX)
find_package(realsense2 REQUIRED)

include_directories(include)
include_directories(include/lcm_types)
#include_directories(${HDF5_INCLUDE_DIRS})
include_directories(${realsense2_INCLUDE_DIRS})
include_directories(camera_lcm_msgs)
include_directories(./)

# message(STATUS "HDF5 Include Dirs: ${HDF5_INCLUDE_DIRS}")

# Adding directory of the .h files
# include_directories(${CMAKE_CURRENT_SOURCE_DIR})

#link_directories(lib/cpp/amd64)

#add_compile_options(-std=c++11)
add_compile_options(-std=c++17)

#message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
#if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
#  set(ARCH amd64)
#endif()
#if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
#  set(ARCH arm64)
#endif()

#link_directories(lib/cpp/${ARCH})
#set(EXTRA_LIBS -pthread libunitree_legged_sdk.so lcm ${realsense2_LIBRARY})
#set(EXTRA_LIBS -pthread libunitree_legged_sdk.so lcm)

set(EXTRA_LIBS -pthread lcm ${realsense2_LIBRARY})

set(CMAKE_CXX_FLAGS "-O3 -fPIC")
 
#add_executable(example_walk_aliengo examples/example_walk_aliengo_sport.cpp)
#target_link_libraries(example_walk_aliengo ${EXTRA_LIBS})

#add_executable(example_start_aliengo examples/example_start_aliengo_sport.cpp)
#target_link_libraries(example_start_aliengo ${EXTRA_LIBS})

#add_executable(example_joystick_aliengo examples/example_joystick_aliengo_sport.cpp)
#target_link_libraries(example_joystick_aliengo ${EXTRA_LIBS})

#add_executable(example_recovery_aliengo examples/example_recovery_aliengo_sport.cpp)
#target_link_libraries(example_recovery_aliengo ${EXTRA_LIBS})

#add_executable(example_position_aliengo examples/example_position_aliengo.cpp)
#target_link_libraries(example_position_aliengo ${EXTRA_LIBS})

#add_executable(example_velocity_aliengo examples/example_velocity_aliengo.cpp)
#target_link_libraries(example_velocity_aliengo ${EXTRA_LIBS})

#add_executable(example_torque_aliengo examples/example_torque_aliengo.cpp)
#target_link_libraries(example_torque_aliengo ${EXTRA_LIBS})

#add_executable(lcm_position HDF5_recorder.h T265_reader.h lcm_position.cpp)
#target_link_libraries(lcm_position ${EXTRA_LIBS} ${HDF5_CXX_LIBRARIES} ${realsense2_LIBRARY} ${HDF5_LIBRARIES})

#add_executable(camera_lcm_msgs HDF5_recorder.h T265_reader.h camera_lcm_msgs.cpp)
#target_link_libraries(camera_lcm_msgs ${EXTRA_LIBS} ${realsense2_LIBRARY})

add_executable(camera_lcm_msgs T265_reader.h camera_lcm_msgs.cpp)
target_link_libraries(camera_lcm_msgs ${EXTRA_LIBS} ${realsense2_LIBRARY})

#add_executable(test T265_reader.h test.cpp)
#target_link_libraries(test ${EXTRA_LIBS})



 
