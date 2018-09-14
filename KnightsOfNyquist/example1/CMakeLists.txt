cmake_minimum_required(VERSION 2.8.12)

project(example1)

if(NOT MSVC)
    add_definitions("-std=c++11 -Wall -Wextra -Werror")
else()
    add_definitions("-std=c++11 -WX -W2")
    include_directories(${CMAKE_SOURCE_DIR}/../../install/include)
    link_directories(${CMAKE_SOURCE_DIR}/../../install/lib)
endif()

add_executable(example1
    example1.cpp
)

target_link_libraries(example1
    dronecore
    dronecore_action
    dronecore_mission
    dronecore_telemetry
)
