cmake_minimum_required(VERSION 3.0)
project(result_viewer LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)

if (Eigen3_FOUND)
    message(STATUS "Eigen3 Found! - ${Eigen3_DIR}")
    set(Eigen3_LIBS Eigen3::Eigen)
endif()

if (OpenCV_FOUND)
    message(STATUS "OpenCV Found! - ${OpenCV_DIR}")
endif()

if (spdlog_FOUND)
    message(STATUS "spdlog Found! - ${spdlog_DIR}")
endif()

include_directories(
    include
    ${Eigen3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIR}
    ${spdlog_INCLUDE_DIR}
)

set(RESULT_VIEWER_SOURCE_FILES
    src/result_viewer/Dataloader.cpp
    src/result_viewer/Projection.cpp
    src/result_viewer/Viewer.cpp
)

add_library(result_viewer
    ${RESULT_VIEWER_SOURCE_FILES}
)

target_link_libraries(result_viewer
    ${Eigen3_LIBS}
    ${OpenCV_LIBS}
    ${spdlog_LIBS}
)

add_executable(main src/main.cpp)

target_link_libraries(main 
    result_viewer
)
