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
    find_package(Boost REQUIRED COMPONENTS serialization)
    include_directories(${Boost_INCLUDE_DIRS})

    find_package(realsense2 REQUIRED)

    #set(BOOST_ROOT )
    #set(RS2_DEPENDENCIES realsense2 ${OPENGL_LIBRARIES})

    add_executable(pcs-camera-server pcs-camera-server.cpp)
    target_link_libraries(
        pcs-camera-server
        realsense2
    )

    add_executable(pcs-camera-grab-frames pcs-camera-grab-frames.cpp)
    target_link_libraries(
        pcs-camera-grab-frames
        realsense2
    )

    install(
        TARGETS

        pcs-camera-server
        pcs-camera-grab-frames
        
        RUNTIME DESTINATION
        ${CMAKE_INSTALL_PREFIX}/bin
    )
endif()
