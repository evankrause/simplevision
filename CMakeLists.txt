cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(simple_vision_processor)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (simple_vision_processor main.cpp SimpleVisionProcessor.cpp)
target_link_libraries (simple_vision_processor ${PCL_LIBRARIES})
