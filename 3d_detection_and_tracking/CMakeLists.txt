cmake_minimum_required(VERSION 3.0)
project(3d_detection_and_tracking LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
find_package(Boost COMPONENTS iostreams system)
include_directories(${Boost_INCLUDE_DIR})

add_subdirectory(modules)

add_executable(kitti_detection examples/kitti_detection.cpp)
target_link_libraries(kitti_detection
    object_tracking
    result_viewer
)

add_executable(kitti_tracking examples/kitti_tracking.cpp)
target_link_libraries(kitti_tracking
    object_tracking
    result_viewer
)

add_executable(real_time_kitti_tracking examples/real_time_kitti_tracking.cpp)
target_link_libraries(real_time_kitti_tracking
    object_tracking
    result_viewer
    ${Boost_LIBRARIES}
)