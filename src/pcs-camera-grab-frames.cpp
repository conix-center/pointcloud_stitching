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
int n_frames = 30;

// Exit gracefully by closing all open sockets and freeing buffer
void sigintHandler(int dummy) {
    std::cout << "\n Exiting \n " << std::endl;
    exit(0);
}

void print_usage() {
    printf("\nUsage: pcs-camera-grab-frames -f <samples.bag> -n <#frames>\n\n");
}

// Parse arguments for extra runtime options
void parseArgs(int argc, char** argv) {
    int c;
    while ((c = getopt(argc, argv, "hf:n:")) != -1) {
        switch(c) {
            case 'h':
                print_usage();
                exit(0);
            case 'f':
                filename = optarg;
                break;
            case 'n':
                n_frames = atoi(optarg);
                break;
        }
    }

    std::cout << "\Capturing " << n_frames << " to Filename: " << filename << std::endl;
}



int main (int argc, char** argv) {

    // Parse Arguments
    parseArgs(argc, argv);

    // Set interrupt signal
    signal(SIGINT, sigintHandler);
    
    
    rs2::config cfg;
    cfg.enable_record_to_file(filename);

    rs2::pipeline pipe;
    rs2::pipeline_profile selection = pipe.start(cfg); //File will be opened in write mode at this point
    //rs2::device selected_device = selection.get_device();
    
    //auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    // Turn off laser emitter for better accuracy with multiple camera setup
    //if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    //    depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);

    for (int i = 0; i < n_frames; i++)
    {
        rs2::frameset frames = pipe.wait_for_frames();
        if( (i+1)%30 == 0){
            std::cout << "Grabbing Frame:" << i+1 << "/" << n_frames << std::endl;
        }
    }
    
    pipe.stop(); //File will be closed at this point

    return 0;
}