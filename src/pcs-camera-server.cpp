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
#include <cstring>
#include <iostream>
#include <chrono>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <librealsense2/rs.hpp>
#include <omp.h>
#include <immintrin.h>
#include <xmmintrin.h>
#include <thread>

#define TIME_NOW    std::chrono::high_resolution_clock::now()
#define BUF_SIZE    5000000
#define CONV_RATE   1000.0
#define DOWNSAMPLE  1
#define PORT        8000

typedef std::chrono::high_resolution_clock clockTime;
typedef std::chrono::time_point<clockTime> timePoint;
typedef std::chrono::duration<double, std::milli> timeMilli;

int loop_count = 1;
int client_sock = 0;
int sockfd = 0;

short * buffer;

bool timer = false;
bool save = false;

float tf_mat[] =  {-0.69888007, -0.32213748,  0.63858757, -2.22900000,
                    -0.71520905,  0.32290986, -0.61984291,  2.91800000,
                    -0.00653159, -0.88991947, -0.45607091,  0.36400000,
                     0.00000000,  0.00000000,  0.00000000,  1.00000000};
                 
__m128 ss_a = _mm_set_ps(0, tf_mat[8], tf_mat[4], tf_mat[0]);
__m128 ss_b = _mm_set_ps(0, tf_mat[9], tf_mat[5], tf_mat[1]);
__m128 ss_c = _mm_set_ps(0, tf_mat[10], tf_mat[6], tf_mat[2]);
__m128 ss_d = _mm_set_ps(0, tf_mat[11], tf_mat[7], tf_mat[3]);

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

int copyPointCloudXYZRGBToBuffer(rs2::points& pts, const rs2::video_frame& color, short * pc_buffer)
{
    const auto vert = pts.get_vertices();
    const rs2::texture_coordinate* tcrd = pts.get_texture_coordinates();
    const uint8_t* color_data = reinterpret_cast<const uint8_t*>(color.get_data());

    const int pts_size = pts.size();
    const int w = color.get_width();
    const int h = color.get_height();
    const int cl_bp = color.get_bytes_per_pixel();
    const int cl_sb = color.get_stride_in_bytes();
    const int w_min = (w - 1);
    const int h_min = (h - 1);

    const float conv_rate = CONV_RATE;
    const float point5f = .5f;
    const float w_f = float(w);
    const float h_f = float(h);

    const __m128 _conv_rate = _mm_broadcast_ss(&conv_rate);
    const __m128 _f5 = _mm_broadcast_ss(&point5f);
    const __m128 _w = _mm_broadcast_ss(&w_f);
    const __m128 _h = _mm_broadcast_ss(&h_f);

    const __m128i _zero = _mm_setzero_si128();
    const __m128i _w_min = _mm_set1_epi32(w_min);
    const __m128i _h_min = _mm_set1_epi32(h_min);
    
    
    #pragma omp parallel for schedule(static, 10000) num_threads(7)
    for (int i = 0; i < pts_size; i += 4) {
        
        int i1 = i;
        int i2 = i+1;
        int i3 = i+2;
        int i4 = i+3;

        __attribute__((aligned(16))) float v_temp1[4];
        __attribute__((aligned(16))) float v_temp2[4];
        __attribute__((aligned(16))) float v_temp3[4];
        __attribute__((aligned(16))) float v_temp4[4];

        __attribute__((aligned(16))) int idx[4];
        __attribute__((aligned(16))) int idy[4];

        // Grab Color
        __m128 _x = _mm_set_ps(tcrd[i1].u, tcrd[i2].u, tcrd[i3].u, tcrd[i4].u); // load u
        __m128 _y = _mm_set_ps(tcrd[i1].v, tcrd[i2].v, tcrd[i3].v, tcrd[i4].v); // load v

        
        _x = _mm_fmadd_ps(_x, _w, _f5); // fma
        _y = _mm_fmadd_ps(_y, _h, _f5);

        // float to int
        __m128i _xi = _mm_cvttps_epi32(_x);
        __m128i _yi = _mm_cvttps_epi32(_y);

        _xi = _mm_max_epi32(_xi, _zero);    // max
        _yi = _mm_max_epi32(_yi, _zero);
        _xi = _mm_min_epi32(_xi, _w_min);   // min
        _yi = _mm_min_epi32(_yi, _h_min);
        
        _mm_storeu_si128((__m128i_u*)idx, _xi);
        _mm_storeu_si128((__m128i_u*)idy, _yi);

        int idx1 = idx[3]*cl_bp + idy[3]*cl_sb;
        int idx2 = idx[2]*cl_bp + idy[2]*cl_sb;
        int idx3 = idx[1]*cl_bp + idy[1]*cl_sb;
        int idx4 = idx[0]*cl_bp + idy[0]*cl_sb;

        // Point Transformatino using SIMD instructions for Matrix Multiplication on xyz points
        __m128 ss_x1 = _mm_broadcast_ss(&vert[i].x);
        __m128 ss_y1 = _mm_broadcast_ss(&vert[i].y);
        __m128 ss_z1 = _mm_broadcast_ss(&vert[i].z);

        __m128 ss_x2 = _mm_broadcast_ss(&vert[i2].x);
        __m128 ss_y2 = _mm_broadcast_ss(&vert[i2].y);
        __m128 ss_z2 = _mm_broadcast_ss(&vert[i2].z);

        __m128 ss_x3 = _mm_broadcast_ss(&vert[i3].x);
        __m128 ss_y3 = _mm_broadcast_ss(&vert[i3].y);
        __m128 ss_z3 = _mm_broadcast_ss(&vert[i3].z);

        __m128 ss_x4 = _mm_broadcast_ss(&vert[i4].x);
        __m128 ss_y4 = _mm_broadcast_ss(&vert[i4].y);
        __m128 ss_z4 = _mm_broadcast_ss(&vert[i4].z);

        __m128 _v1 = _mm_fmadd_ps(ss_x1, ss_a, ss_d);
        _v1 = _mm_fmadd_ps(ss_y1, ss_b, _v1);
        _v1 = _mm_fmadd_ps(ss_z1, ss_c, _v1);

        __m128 _v2 = _mm_fmadd_ps(ss_x2, ss_a, ss_d);
        _v2 = _mm_fmadd_ps(ss_y2, ss_b, _v2);
        _v2 = _mm_fmadd_ps(ss_z2, ss_c, _v2);

        __m128 _v3 = _mm_fmadd_ps(ss_x3, ss_a, ss_d);
        _v3 = _mm_fmadd_ps(ss_y3, ss_b, _v3);
        _v3 = _mm_fmadd_ps(ss_z3, ss_c, _v3);

        __m128 _v4 = _mm_fmadd_ps(ss_x4, ss_a, ss_d);
        _v4 = _mm_fmadd_ps(ss_y4, ss_b, _v4);
        _v4 = _mm_fmadd_ps(ss_z4, ss_c, _v4);

        // SIMD Conversion from floats to shorts
        _v1 = _mm_mul_ps(_v1, _conv_rate); // multiply with _conv_rate
        _v2 = _mm_mul_ps(_v2, _conv_rate); // multiply with _conv_rate
        _v3 = _mm_mul_ps(_v3, _conv_rate); // multiply with _conv_rate
        _v4 = _mm_mul_ps(_v4, _conv_rate); // multiply with _conv_rate
        
        _mm_store_ps(v_temp1, _v1);
        _mm_store_ps(v_temp2, _v2);
        _mm_store_ps(v_temp3, _v3);
        _mm_store_ps(v_temp4, _v4);

        //v1
        pc_buffer[i * 5 + 0] = short(v_temp1[0]);
        pc_buffer[i * 5 + 1] = short(v_temp1[1]);
        pc_buffer[i * 5 + 2] = short(v_temp1[2]);
        pc_buffer[i * 5 + 3] = color_data[idx1] + (color_data[idx1 + 1] << 8);
        pc_buffer[i * 5 + 4] = color_data[idx1 + 2];
        
        //v2
        pc_buffer[i * 5 + 5] = short(v_temp2[0]);
        pc_buffer[i * 5 + 6] = short(v_temp2[1]);
        pc_buffer[i * 5 + 7] = short(v_temp2[2]);
        pc_buffer[i * 5 + 8] = color_data[idx2] + (color_data[idx2 + 1] << 8);
        pc_buffer[i * 5 + 9] = color_data[idx2 + 2];
        
        //v3
        pc_buffer[i * 5 + 10] = short(v_temp3[0]);
        pc_buffer[i * 5 + 11] = short(v_temp3[1]);
        pc_buffer[i * 5 + 12] = short(v_temp3[2]);
        pc_buffer[i * 5 + 13] = color_data[idx3] + (color_data[idx3 + 1] << 8);
        pc_buffer[i * 5 + 14] = color_data[idx3 + 2];
        
        //v4
        pc_buffer[i * 5 + 15] = short(v_temp4[0]);
        pc_buffer[i * 5 + 16] = short(v_temp4[1]);
        pc_buffer[i * 5 + 17] = short(v_temp4[2]);
        pc_buffer[i * 5 + 18] = color_data[idx4] + (color_data[idx4 + 1] << 8);
        pc_buffer[i * 5 + 19] = color_data[idx4 + 2];

    }
    
    return pts_size;
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

    double frame_total, pc_total;
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
            frame_start = TIME_NOW;
        
        // Wait for pull request
        if (recv(client_sock, pull_request, 1, 0) < 0) {
            std::cout << "Client disconnected" << std::endl;
            break;
        }
        if (pull_request[0] == 'Z') {          // Client requests color pointcloud (XYZRGB)
            if (timer) {
                grab_frame_start = TIME_NOW;
            }

            // Grab depth and color frames, and map each point to a color value
            auto frames = pipe.wait_for_frames();
            auto depth = frames.get_depth_frame();
            auto color = frames.get_color_frame();

            if (timer) {
                grab_frame_end_calculate_start = TIME_NOW;
            }

            auto pts = pc.calculate(depth);
            pc.map_to(color);                       // Maps color values to a point in 3D space

            if (timer) {
                calculate_end = TIME_NOW;
            }

            // Spawn a thread to send pointcloud over to client
            std::thread frame_thread(sendXYZRGBPointcloud, pts, color, buffer);
            frame_thread.detach();
        }
        else {                                     // Did not receive a correct pull request
            std::cerr << "Faulty pull request" << std::endl;
            exit(EXIT_FAILURE);
        }

        if (timer) {
            frame_end = TIME_NOW;
            double temp_frame = timeMilli(grab_frame_end_calculate_start - grab_frame_start).count();
            double temp_pc = timeMilli(calculate_end - grab_frame_end_calculate_start).count();
            frame_total += temp_frame;
            pc_total += temp_pc;
            std::cout << "Grab frame average: " << frame_total / loop_count << " ms" << std::endl;
            std::cout << "Calculate pc average: " << pc_total / loop_count << " ms" << std::endl;
            std::cout << "Loop count: " << loop_count << "\n\n" << std::endl;
            loop_count++;
            // std::cout << "Grab frame: " << timeMilli(grab_frame_end_calculate_start - grab_frame_start).count() << " ms" << std::endl;
            // std::cout << "Get pointcloud: " << timeMilli(calculate_end - grab_frame_end_calculate_start).count() << " ms" << std::endl;
            std::cout << "Frame: " << timeMilli(frame_end - frame_start).count() << " ms" << std::endl;
            // std::cout << "FPS: " << 1000.0 / timeMilli(frame_end - frame_start).count() << "\n" << std::endl;
        }
    }

    close(client_sock);
    close(sockfd);
    free(buffer);
    return 0;
}