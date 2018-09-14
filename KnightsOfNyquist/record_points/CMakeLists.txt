cmake_minimum_required(VERSION 2.8.12)

project(record_points)

if(NOT MSVC)
    add_definitions("-std=c++11 -Wall -Wextra -Werror")
else()
    add_definitions("-std=c++11 -WX -W2")
    include_directories(${CMAKE_SOURCE_DIR}/../../install/include)
    link_directories(${CMAKE_SOURCE_DIR}/../../install/lib)
endif()

add_executable(record_points
    record_points.cpp
)

target_link_libraries(record_points
    dronecore
    dronecore_action
    dronecore_mission
    dronecore_telemetry
)
