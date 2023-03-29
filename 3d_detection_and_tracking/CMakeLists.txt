cmake_minimum_required(VERSION 3.0)
project(3d_detection_and_tracking LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
find_package(Boost COMPONENTS iostreams system)
include_directories(${Boost_INCLUDE_DIR})

add_subdirectory(modules)

add_executable(example1 examples/example1.cpp)
target_link_libraries(example1
    object_tracking
    result_viewer
)

# add_executable(example2 examples/example2.cpp)
# target_link_libraries(example2
#     object_tracking
#     result_viewer
# )

add_executable(example3 examples/example3.cpp)
target_link_libraries(example3
    object_tracking
    result_viewer
    ${Boost_LIBRARIES}
)