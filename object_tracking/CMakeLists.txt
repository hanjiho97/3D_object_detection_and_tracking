cmake_minimum_required(VERSION 3.0)
project(OBJECT-TRACKING CXX)

find_package(Eigen3 REQUIRED)

if (Eigen3_FOUND)
    message(STATUS "Eigen3 Found! - ${Eigen3_DIR}")
    set(Eigen3_LIBS Eigen3::Eigen)
endif()

include_directories(
    include
    ${Eigen3_INCLUDE_DIR}
)

add_library(object_tracking
    src/object_tracking/EKF.cpp
    src/object_tracking/Tracker.cpp
    src/object_tracking/Measurement.cpp
    src/object_tracking/Association.cpp
)

add_executable(example src/example.cpp)
target_link_libraries(example object_tracking)
