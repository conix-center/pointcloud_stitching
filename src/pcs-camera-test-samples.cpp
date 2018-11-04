/*
 * Artur Balanuta
 * pcs-camera-grab-frames.cpp
 *
 */

#include <librealsense2/rs.hpp>

#include <signal.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>

#include <iostream>

char *filename = "samples.bag";

// Exit gracefully by closing all open sockets and freeing buffer
void sigintHandler(int dummy) {
    std::cout << "\n Exiting \n " << std::endl;
    exit(0);
}

void print_usage() {
    printf("\nUsage: pcs-camera-test-samples -f <samples.bag>\n\n");
}

// Parse arguments for extra runtime options
void parseArgs(int argc, char** argv) {
    int c;
    while ((c = getopt(argc, argv, "hf:")) != -1) {
        switch(c) {
            case 'h':
                print_usage();
                exit(0);
            case 'f':
                filename = optarg;
                break;
        }
    }

    std::cout << "\Reading Frames from File: " << filename << std::endl;
}



int main (int argc, char** argv) {

    // Parse Arguments
    parseArgs(argc, argv);

    // Set interrupt signal
    signal(SIGINT, sigintHandler);
    
    
    rs2::config cfg;
    cfg.enable_device_from_file(filename);

    rs2::pipeline pipe;
    pipe.start(cfg);

    rs2::device device = pipe.get_active_profile().get_device();
    std::cout << "Camera Info:" << device.get_info(RS2_CAMERA_INFO_NAME) << std::endl;

    int i = 0, last_frame = 0;
    
    while (true)
    {    
        rs2::frameset frames;
        if (!pipe.poll_for_frames(&frames))
        {
            continue;
        }
        else
        {   
            if (last_frame > frames.get_frame_number()) break;
            if (last_frame == frames.get_frame_number()) continue;
            
            //std::cout << "Got Frame # " << last_frame << std::endl;
            last_frame = frames.get_frame_number();
            i++;

            //use frames here


        }
    }
    
    pipe.stop();
    
    std::cout << "# Total Frames = " << i << std::endl;
    return 0;
}