#find_package(Boost REQUIRED COMPONENTS serialization)
#include_directories(${Boost_INCLUDE_DIRS})

find_package(realsense2 REQUIRED)
find_package(OpenMP REQUIRED)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()



add_executable(pcs-camera-grab-frames pcs-camera-grab-frames.cpp)
target_link_libraries(
    pcs-camera-grab-frames
    realsense2
)

add_executable(pcs-camera-test-samples pcs-camera-test-samples.cpp)
target_link_libraries(
    pcs-camera-test-samples
    realsense2
)

add_executable(pcs-camera-optimized pcs-camera-optimized.cpp)
target_link_libraries(
    pcs-camera-optimized
    realsense2
    "${OpenMP_CXX_FLAGS}"
)
set_target_properties(pcs-camera-optimized PROPERTIES COMPILE_FLAGS "-mavx -mfma" )

#add_executable(pcs-client pcs-client.cpp)
#target_link_libraries(
#    pcs-client
#    realsense2
#    "${OpenMP_CXX_FLAGS}"
#)
#set_target_properties(pcs-client PROPERTIES COMPILE_FLAGS "-mavx -mfma -pthread" )

install(
    TARGETS
    pcs-camera-grab-frames
    pcs-camera-test-samples
    pcs-camera-optimized
#    pcs-client
    
    RUNTIME DESTINATION
    ${CMAKE_INSTALL_PREFIX}/bin
)

if (BUILD_CLIENT) 
    add_executable(pcs-multicamera-client pcs-multicamera-client.cpp)
    target_link_libraries(
        pcs-multicamera-client
        realsense2
    )
    set_target_properties(pcs-multicamera-client PROPERTIES COMPILE_FLAGS "-mavx -mfma" )

    add_executable(pcs-multicamera-optimized pcs-multicamera-optimized.cpp)
    target_link_libraries(
        pcs-multicamera-optimized
        realsense2
    )

    install(
        TARGETS
        pcs-multicamera-client
        pcs-multicamera-optimized
        
        RUNTIME DESTINATION
        ${CMAKE_INSTALL_PREFIX}/bin
    )
endif()
