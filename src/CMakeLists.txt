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
    find_package(OpenMP REQUIRED)

    if (OPENMP_FOUND)
        set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    endif()
    
    set(RS2_DEPENDENCIES realsense2 ${OPENGL_LIBRARIES})

    add_executable(pcs-camera-server pcs-camera-server.cpp)
    target_link_libraries(pcs-camera-server
        ${RS2_DEPENDENCIES} 
        "${OpenMP_CXX_FLAGS}"
    )
    set_target_properties(pcs-camera-server PROPERTIES COMPILE_FLAGS "-O3 -mavx -mfma" )
        
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