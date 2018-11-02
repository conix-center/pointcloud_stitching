if (BUILD_CLIENT)
    find_package(PCL REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})

    add_executable(pcs-multicamera-client pcs-multicamera-client.cpp)
    target_link_libraries(
        ${RS2_DEPENDENCIES}
        ${PCL_LIBRARIES}
    )

    install(
        TARGETS

        pcs-multicamera-client

        RUNTIME DESTINATION
        ${CMAKE_INSTALL_PREFIX}/bin
    )

else()
    set(RS2_DEPENDENCIES realsense2 ${OPENGL_LIBRARIES})

    add_executable(pcs-camera-server pcs-camera-server.cpp)
    target_link_libraries(pcs-camera-server
        ${RS2_DEPENDENCIES})
        ${MQTT_DEPENDENCIES})
        
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