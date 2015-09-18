cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(SimulateData)

find_package(PCL 1.2 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
aux_source_directory(. SRC_LIST)

add_executable(${PROJECT_NAME} ${SRC_LIST})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})



