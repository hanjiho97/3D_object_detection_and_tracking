cmake_minimum_required(VERSION 3.0)
project(3d_detection_and_tracking LANGUAGES CXX)

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