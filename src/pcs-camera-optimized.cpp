/*
 * Artur Balanuta
 * pcs-camera-optimized.cpp
 */

#include <cstring>
#include <iostream>
#include <chrono>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <signal.h>

#include <librealsense2/rs.hpp>

#define TIME_NOW    std::chrono::high_resolution_clock::now()
#define BUF_SIZE    4000000
#define CONV_RATE   1000

typedef std::chrono::high_resolution_clock clockTime;
typedef std::chrono::duration<double, std::milli> timeMilli;
typedef std::chrono::time_point<clockTime> timestamp;

char *filename = "samples.bag";
timestamp time_start, time_end;

void sendXYZRGBPointcloud(rs2::points pts, rs2::video_frame color, short * buffer);

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
    rs2::pipeline pipe;
    rs2::pointcloud pc;
    
    cfg.enable_device_from_file(filename);
    rs2::pipeline_profile selection = pipe.start(cfg);

    rs2::device device = pipe.get_active_profile().get_device();
    std::cout << "Camera Info: " << device.get_info(RS2_CAMERA_INFO_NAME) << " FW ver:" << device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
    
    auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto resolution = std::make_pair(depth_stream.width(), depth_stream.height());
    auto intr = depth_stream.get_intrinsics();


    //get_extrinsics(const rs2::stream_profile& from_stream, const rs2::stream_profile& to_stream)

    int i = 0, last_frame = 0;
    double duration_sum = 0;
    short *buffer = (short *)malloc(sizeof(short) * BUF_SIZE);
    
    rs2::frameset frames;
    
    
    while (true)
    {    
        
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
            time_start = TIME_NOW;
                                                                // stairs.bag vs sample.bag
            rs2::video_frame color = frames.get_color_frame();  // 0.003 ms vs 0.001ms
            rs2::depth_frame depth = frames.get_depth_frame();  // 0.001ms vs 0.001ms
            rs2::points pts = pc.calculate(depth);              // 27ms vs 27ms
                        
            pc.map_to(color);       // 0.01ms vs 0.02ms  // Maps color values to a point in 3D space
            sendXYZRGBPointcloud(pts, color, buffer);   // 86ms vs 9.7ms
            time_end = TIME_NOW;

            //std::cout << "Frame Time: " << timeMilli(time_end - time_start).count() << " ms" << std::endl;
            //std::cout << "FPS: " << 1000.0 / timeMilli(time_end - time_start).count() << "\n" << std::endl;
            duration_sum += timeMilli(time_end - time_start).count();
        }
    }
    
    pipe.stop();

    // Use last Frame to display frame Info
    rs2::video_frame color = frames.get_color_frame();
    rs2::depth_frame depth = frames.get_depth_frame();
    rs2::points pts = pc.calculate(depth);

    std::cout << "### Video Frames H x W : " << color.get_height() << " x " << color.get_width() << std::endl;
    std::cout << "### Depth Frames H x W : " << depth.get_height() << " x " << depth.get_width() << std::endl;
    std::cout << "### # Points : " << pts.size() << std::endl;
    
    std::cout << "\n### Total Frames = " << i << std::endl;
    std::cout << "### AVG Frame Time: " << duration_sum / i << " ms" << std::endl;
    std::cout << "### AVG FPS: " << 1000.0 / (duration_sum / i) << "\n" << std::endl;
    return 0;
}


// Returns the RGB value of a point in the pointcloud.
std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords) {
    const int w = texture.get_width(), h = texture.get_height();
    int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);
    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());

    return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
}

// Converts the XYZ values into shorts for less memory overhead,
// and puts the XYZRGB values of each point into the buffer.
int copyPointCloudXYZRGBToBuffer(rs2::points& pts, const rs2::video_frame& color, short * pc_buffer) {

    auto vertices = pts.get_vertices();
    auto tex_coords = pts.get_texture_coordinates();
    int size = 0;

    for (size_t i = 0; i < pts.size() && (5 * size + 2) < BUF_SIZE; i++) {
        
        // Set cutoff range for pixel points, to lower data size, and omit outlying points
        //if ((vertices[i].x != 0) && (vertices[i].x < 2) && (vertices[i].x > -2) && (vertices[i].z != 0) && (vertices[i].z < .5)) {
            
            //std::tuple<uint8_t, uint8_t, uint8_t> current_color = get_texcolor(color, tex_coords[i]);

            pc_buffer[size * 5 + 0] = static_cast<short>(vertices[i].x * CONV_RATE);
            pc_buffer[size * 5 + 1] = static_cast<short>(vertices[i].y * CONV_RATE);
            pc_buffer[size * 5 + 2] = static_cast<short>(vertices[i].z * CONV_RATE);
            //pc_buffer[size * 5 + 3] = (short)std::get<0>(current_color) + (short)(std::get<1>(current_color) << 8);
            //pc_buffer[size * 5 + 4] = std::get<2>(current_color);

            size++;
        //}
    }

    return size;
}

void sendXYZRGBPointcloud(rs2::points pts, rs2::video_frame color, short * buffer) {
    // Add size of buffer to beginning of message
    int size = copyPointCloudXYZRGBToBuffer(pts, color, &buffer[0] + sizeof(short));
    size = 5 * size * sizeof(short);
    memcpy(buffer, &size, sizeof(int));

    //send(client_sock, (char *)buffer, size + sizeof(int), 0);
}