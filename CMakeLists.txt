#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(Point_Cloud_Stitching)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

option(BUILD_CLIENT "Build the central computing client" OFF)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -w -pthread")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -w -pthread")
endif()

set(MQTT_DEPENDENCIES paho-mqttpp3 paho-mqtt3a)

if (BUILD_CLIENT)
    find_package(PCL REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})

    add_executable(pcs-multicamera-client pcs-multicamera-client.cpp)
    target_link_libraries(pcs-multicamera-client ${RS2_DEPENDENCIES} ${PCL_LIBRARIES})

    add_executable(pcs-mqtt-subscriber pcs-mqtt-subscriber.cpp)
    target_link_libraries(pcs-mqtt-subscriber ${MQTT_DEPENDENCIES})

    install(
        TARGETS

        pcs-multicamera-client
        pcs-mqtt-subscriber

        RUNTIME DESTINATION
        ${CMAKE_INSTALL_PREFIX}/bin
    )
else()
    set(RS2_DEPENDENCIES realsense2 ${OPENGL_LIBRARIES})

    add_executable(pcs-camera-server pcs-camera-server.cpp)
    target_link_libraries(pcs-camera-server ${RS2_DEPENDENCIES})
        
    add_executable(pcs-mqtt-publisher pcs-mqtt-publisher.cpp)
    target_link_libraries(pcs-mqtt-publisher ${MQTT_DEPENDENCIES})

    install(
        TARGETS

        pcs-camera-server
        pcs-mqtt-publisher

        RUNTIME DESTINATION
        ${CMAKE_INSTALL_PREFIX}/bin
    )
endif()


