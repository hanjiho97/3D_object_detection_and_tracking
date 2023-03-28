cmake_minimum_required(VERSION 3.0)
project(object_tracking LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

if (Eigen3_FOUND)
    message(STATUS "Eigen3 Found! - ${Eigen3_DIR}")
    set(Eigen3_LIBS Eigen3::Eigen)
endif()

if (OpenCV_FOUND)
    message(STATUS "OpenCV Found! - ${OpenCV_DIR}")
endif()

set(OBJECT_TRACKING_SOURCE_FILES
    src/object_tracking/EKF.cpp
    src/object_tracking/Tracker.cpp
    src/object_tracking/Measurement.cpp
    src/object_tracking/Association.cpp
    src/object_tracking/Dataloader.cpp
)

add_library(object_tracking
    ${OBJECT_TRACKING_SOURCE_FILES}
)

target_include_directories(object_tracking PUBLIC
    include
    ${CMAKE_SOURCE_DIR}/include
    ${Eigen3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIR}
)

target_link_libraries(object_tracking
    ${Eigen3_LIBS}
    ${OpenCV_LIBS}
)
