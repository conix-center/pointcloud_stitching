/*
 * Evan Li
 * pcs-camera-server.cpp
 *
 * This program uses the librealsense library to grab frames from 
 * a D-400 Series Intel RealSense depth camera, and compute the 3D
 * point cloud. The raw XYZ and RGB values are then sent over a TCP
 * socket to a central computer to perform post-processing and 
 * visualization. 
 */

#include <librealsense2/rs.hpp>

#include <signal.h>
#include <unistd.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <cstring>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <chrono>
#include <thread>

typedef std::chrono::high_resolution_clock clockTime;
typedef std::chrono::time_point<clockTime> timePoint;
typedef std::chrono::duration<double, std::milli> timeMilli;

const int PORT = 8000;
const int BUF_SIZE = 4000000;
const int CONV_RATE = 1000;

int client_sock = 0;
int sockfd = 0;
short * buffer;
bool timer = false;
bool save = false;

// Exit gracefully by closing all open sockets and freeing buffer
void sigintHandler(int dummy) {
    close(client_sock);
    close(sockfd);
    free(buffer);
}

// Parse arguments for extra runtime options
void parseArgs(int argc, char** argv) {
    int c;
    while ((c = getopt(argc, argv, "hts")) != -1) {
        switch(c) {
            // Prints out the runtime of the main expensive functions and FPS
            case 't':
                timer = true;
                break;
            // Saves the first 20 frames in .ply     
            case 's':
                save = true;
                break;
            default:
            case 'h':
                std::cout << "\nPointcloud stitching camera server" << std::endl;
                std::cout << "Usage: pcs-camera-server <port> [options]\n" << std::endl;
                std::cout << "Options:" << std::endl;
                std::cout << " -h (help)    Display command line options" << std::endl;
                std::cout << " -t (timer)   Displays the runtime of certain functions" << std::endl;
                std::cout << " -s (save)    Saves 20 frames in a .ply format" << std::endl;
                exit(0);
        }
    }
}

// Creates TCP stream socket and connects to the central computer.
void initSocket(int port) {
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(port);

    if ((sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
        std::cerr << "\nSocket fd not received." << std::endl;
        exit(EXIT_FAILURE);
    }

    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "\nBind failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    if (listen(sockfd, 3) < 0) {
        std::cerr << "\nListen failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "Waiting for client..." << std::endl;

    if ((client_sock = accept(sockfd, NULL, NULL)) < 0) {
        std::cerr << "\nConnection failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "Established connection with client_sock: " << client_sock << std::endl;
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
// and puts the XYZ values of each point into the buffer. 
int copyPointCloudXYZToBuffer(rs2::points& pts, short * pc_buffer) {
    auto vertices = pts.get_vertices();
    int size = 0;

    for (size_t i = 0; i < pts.size() && (3 * size + 2) < BUF_SIZE; i++) {
        // Set cutoff range for pixel points, to lower data size, and omit outlying points
        if ((vertices[i].x != 0) && (vertices[i].x < 2) && (vertices[i].x > -2) && (vertices[i].z != 0) && (vertices[i].z < 3)) {
            pc_buffer[size * 3 + 0] = static_cast<short>(vertices[i].x * CONV_RATE);
            pc_buffer[size * 3 + 1] = static_cast<short>(vertices[i].y * CONV_RATE);
            pc_buffer[size * 3 + 2] = static_cast<short>(vertices[i].z * CONV_RATE);

            size++;
        }
    }

    return size;
}

// Converts the XYZ values into shorts for less memory overhead,
// and puts the XYZRGB values of each point into the buffer.
int copyPointCloudXYZRGBToBuffer(rs2::points& pts, const rs2::video_frame& color, short * pc_buffer) {
    timePoint copy_start, copy_end;
    if (timer)
            copy_start = std::chrono::high_resolution_clock::now();

    auto vertices = pts.get_vertices();
    auto tex_coords = pts.get_texture_coordinates();
    int size = 0;

    for (size_t i = 0; i < pts.size() && (5 * size + 2) < BUF_SIZE; i++) {
        // Set cutoff range for pixel points, to lower data size, and omit outlying points
        if ((vertices[i].x != 0) && (vertices[i].x < 2) && (vertices[i].x > -2) && (vertices[i].z != 0) && (vertices[i].z < 1)) {
            std::tuple<uint8_t, uint8_t, uint8_t> current_color = get_texcolor(color, tex_coords[i]);

            pc_buffer[size * 5 + 0] = static_cast<short>(vertices[i].x * CONV_RATE);
            pc_buffer[size * 5 + 1] = static_cast<short>(vertices[i].y * CONV_RATE);
            pc_buffer[size * 5 + 2] = static_cast<short>(vertices[i].z * CONV_RATE);
            pc_buffer[size * 5 + 3] = (short)std::get<0>(current_color) + (short)(std::get<1>(current_color) << 8);
            pc_buffer[size * 5 + 4] = std::get<2>(current_color);

            size++;
        }
    }


    if (timer) {
            copy_end = std::chrono::high_resolution_clock::now();
            std::cout << "Copy to buffer: " << timeMilli(copy_end - copy_start).count() << " ms" << std::endl;
    }

    return size;
}

void sendXYZPointcloud(rs2::points pts, short * buffer) {
    // Add size of buffer to beginning of message
    int size = copyPointCloudXYZToBuffer(pts, &buffer[0] + sizeof(short));
    size = 3 * size * sizeof(short);
    memcpy(buffer, &size, sizeof(int));

    send(client_sock, (char *)buffer, size + sizeof(int), 0);
}

void sendXYZRGBPointcloud(rs2::points pts, rs2::video_frame color, short * buffer) {
    // Add size of buffer to beginning of message
    int size = copyPointCloudXYZRGBToBuffer(pts, color, &buffer[0] + sizeof(short));
    size = 5 * size * sizeof(short);
    memcpy(buffer, &size, sizeof(int));

    send(client_sock, (char *)buffer, size + sizeof(int), 0);
}

int main (int argc, char** argv) {
    parseArgs(argc, argv);

    char pull_request[1] = {0};
    buffer = (short *)malloc(sizeof(short) * BUF_SIZE);
    timePoint frame_start, frame_end, grab_frame_start, grab_frame_end_calculate_start, calculate_end;

    rs2::pointcloud pc;
    rs2::pipeline pipe;
    rs2::pipeline_profile selection = pipe.start();
    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    // Turn off laser emitter for better accuracy with multiple camera setup
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);

    initSocket(PORT);
    signal(SIGINT, sigintHandler);

    // Loop until client disconnected
    while (1) {
        if (timer)
            frame_start = std::chrono::high_resolution_clock::now();
        
        // Wait for pull request
        if (recv(client_sock, pull_request, 1, 0) < 0) {
            std::cout << "Client disconnected" << std::endl;
            break;
        }
        if (pull_request[0] == 'Y') {               // Client requests color-less pointcloud (XYZ)
            // Grab depth frames from realsense and calculate pointcloud coordinates
            auto frames = pipe.wait_for_frames();
            auto depth = frames.get_depth_frame();
            auto pts = pc.calculate(depth);

            // Spawn a thread to send pointcloud over to client
            std::thread frame_thread(sendXYZPointcloud, pts, buffer);
            frame_thread.detach();
        }
        else if (pull_request[0] == 'Z') {          // Client requests color pointcloud (XYZRGB)
            if (timer) {
                grab_frame_start = std::chrono::high_resolution_clock::now();
            }

            // Grab depth and color frames, and map each point to a color value
            auto frames = pipe.wait_for_frames();
            auto depth = frames.get_depth_frame();
            auto color = frames.get_color_frame();

            if (timer) {
                grab_frame_end_calculate_start = std::chrono::high_resolution_clock::now();
            }

            auto pts = pc.calculate(depth);
            pc.map_to(color);                       // Maps color values to a point in 3D space

            if (timer) {
                calculate_end = std::chrono::high_resolution_clock::now();
            }

            // Spawn a thread to send pointcloud over to client
            std::thread frame_thread(sendXYZRGBPointcloud, pts, color, buffer);
            frame_thread.detach();
        }
        else {                                      // Did not receive a correct pull request
            std::cerr << "Faulty pull request" << std::endl;
            exit(EXIT_FAILURE);
        }

        if (timer) {
            frame_end = std::chrono::high_resolution_clock::now();
            std::cout << "Grab frame: " << timeMilli(grab_frame_end_calculate_start - grab_frame_start).count() << " ms" << std::endl;
            std::cout << "Get pointcloud: " << timeMilli(calculate_end - grab_frame_end_calculate_start).count() << " ms" << std::endl;
            std::cout << "Frame: " << timeMilli(frame_end - frame_start).count() << " ms" << std::endl;
            std::cout << "FPS: " << 1000.0 / timeMilli(frame_end - frame_start).count() << "\n" << std::endl;
        }
    }

    close(client_sock);
    close(sockfd);
    free(buffer);
    return 0;
}