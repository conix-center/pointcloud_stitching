/*
 * Artur Balanuta, Evan Li
 * pcs-camera-optimized.cpp
 */

#include <cstring>
#include <iostream>
#include <chrono>

#include <string>
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

// #include "snappy.h"

#define TIME_NOW    std::chrono::high_resolution_clock::now()
#define BUF_SIZE    5000000
#define CONV_RATE   1000.0
#define DOWNSAMPLE  1
#define PORT        8000

struct point {
    
    // Coordenates
    short x;
    short y;
    short z;

    // Colors
    char r;
    char g;
    char b; 
};

typedef std::chrono::high_resolution_clock clockTime;
typedef std::chrono::duration<double, std::milli> timeMilli;
typedef std::chrono::time_point<clockTime> timestamp;

timestamp time_start, time_end;

char *filename;

bool display_updates = false;
bool send_buffer = false;
bool cutoff = false;
bool use_simd = false;
bool compress = false;
bool initialized = false;

int num_of_threads = 1;
int client_sock = 0;
int sockfd = 0;
int pts_size;

float conv_rate = CONV_RATE;
float half = 0.5;
float x_hi = 2;
float x_lo = -2;
float y_hi = 2;
float y_lo = -2;
float z_hi = 3;
float z_lo = 0;

float tf_mat[] =   {-0.99977970,  0.00926272,  0.01883480,  0.00000000,
                    -0.01638983,  0.21604544, -0.97624574,  3.41600000,
                    -0.01311186, -0.97633937, -0.21584603,  1.80200000,
                     0.00000000,  0.00000000,  0.00000000,  1.00000000};

__m256 mat_x_v = _mm256_setr_ps(tf_mat[0], tf_mat[4], tf_mat[8], 0, tf_mat[0], tf_mat[4], tf_mat[8], 0);
__m256 mat_y_v = _mm256_setr_ps(tf_mat[1], tf_mat[5], tf_mat[9], 0, tf_mat[1], tf_mat[5], tf_mat[9], 0);
__m256 mat_z_v = _mm256_setr_ps(tf_mat[2], tf_mat[6], tf_mat[10], 0, tf_mat[2], tf_mat[6], tf_mat[10], 0);
__m256 mat_d_v = _mm256_setr_ps(tf_mat[3], tf_mat[7], tf_mat[11], 0, tf_mat[3], tf_mat[7], tf_mat[11], 0);

__m256 w_v;
__m256 h_v;
__m256 w_min_v;
__m256 h_min_v;
__m256 cl_bp_v;
__m256 cl_sb_v;

const __m256 conv_rate_v = _mm256_broadcast_ss(&conv_rate);
const __m256 z_lo_v      = _mm256_broadcast_ss(&z_lo);
const __m256 z_hi_v      = _mm256_broadcast_ss(&z_hi);
const __m256 x_lo_v      = _mm256_broadcast_ss(&x_lo);
const __m256 x_hi_v      = _mm256_broadcast_ss(&x_hi);
const __m256 zero_v      = _mm256_setzero_ps();

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

int sendXYZRGBPointcloud(rs2::points pts, rs2::video_frame color, short * buffer);

// Exit gracefully by closing all open sockets and freeing buffer
void sigintHandler(int dummy) {
    std::cout << "\n Exiting \n " << std::endl;
    exit(0);
}

void print_usage() {
    printf("\nUsage: pcs-camera-test-samples -f <samples.bag> -v (visualize)\n\n");
}

// Parse arguments for extra runtime options
void parseArgs(int argc, char** argv) {
    int c;
    while ((c = getopt(argc, argv, "hf:vst:cmz")) != -1) {
        switch(c) {
            case 'h':
                print_usage();
                exit(0);
            case 'f':
                filename = optarg;
                break;
            case 'v':
                display_updates = true;
                break;
            case 's':
                send_buffer = true;
                break;
            case 't':
                num_of_threads = atoi(optarg);
                break;
            case 'c':
                cutoff = true;
                break;
            case 'm':
                use_simd = true;
                break;
            case 'z':
                compress = true;
                break;
        }
    }
}

int main (int argc, char** argv) {
    parseArgs(argc, argv);              // Parse Arguments
    signal(SIGINT, sigintHandler);      // Set interrupt signal
    
    int buff_size = 0, buff_size_sum = 0;
    short *buffer = (short *)malloc(sizeof(short) * BUF_SIZE);
    
    if (filename == NULL) {
        char pull_request[1] = {0};
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
            // if (timer)
            //     frame_start = TIME_NOW;
            
            // Wait for pull request
            if (recv(client_sock, pull_request, 1, 0) < 0) {
                std::cout << "Client disconnected" << std::endl;
                break;
            }
            if (pull_request[0] == 'Z') {          // Client requests color pointcloud (XYZRGB)
                // if (timer) {
                //     grab_frame_start = TIME_NOW;
                // }

                // Grab depth and color frames, and map each point to a color value
                auto frames = pipe.wait_for_frames();
                auto depth = frames.get_depth_frame();
                auto color = frames.get_color_frame();

                // if (timer) {
                //     grab_frame_end_calculate_start = TIME_NOW;
                // }

                auto pts = pc.calculate(depth);
                pc.map_to(color);                       // Maps color values to a point in 3D space

                // if (timer) {
                //     calculate_end = TIME_NOW;
                // }

                buff_size = sendXYZRGBPointcloud(pts, color, buffer);
            }
            else {                                     // Did not receive a correct pull request
                std::cerr << "Faulty pull request" << std::endl;
                exit(EXIT_FAILURE);
            }
        }

        close(client_sock);
        close(sockfd);
    }
    else {
        std::cout << "Reading Frames from File: " << filename << std::endl;
        
        rs2::config cfg;
        rs2::pipeline pipe;
        rs2::pointcloud pc;
        
        cfg.enable_device_from_file(filename);
        rs2::pipeline_profile selection = pipe.start(cfg);

        rs2::device device = pipe.get_active_profile().get_device();
        std::cout << "Camera Info: " << device.get_info(RS2_CAMERA_INFO_NAME) << " FW ver:" << device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
        if (num_of_threads) std::cout << "OpenMP Threads: " << num_of_threads << std::endl;
        
        //auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        //auto resolution = std::make_pair(depth_stream.width(), depth_stream.height());
        //auto intr = depth_stream.get_intrinsics();

        // rs2::pipeline pipe;
        // pipe.start();

        // const auto CAPACITY = 5; // allow max latency of 5 frames
        // rs2::frame_queue queue(CAPACITY);
        // std::thread t([&]() {
        //     while (true)
        //     {
        //         rs2::depth_frame frame;
        //         if (queue.poll_for_frame(&frame))
        //         {
        //             frame.get_data();
        //             // Do processing on the frame
        //         }
        //     }
        // });
        // t.detach();

        // while (true)
        // {
        //     auto frames = pipe.wait_for_frames();
        //     queue.enqueue(frames.get_depth_frame());
        // }
        //get_extrinsics(const rs2::stream_profile& from_stream, const rs2::stream_profile& to_stream)

        int i = 0, last_frame = 0;
        double duration_sum = 0;
        
        rs2::frameset frames;

        if (send_buffer) initSocket(PORT);
        
        while (true)
        {    
            
            if (!pipe.poll_for_frames(&frames))
            {
                continue;
            }
            else
            {

                if (!send_buffer && last_frame > frames.get_frame_number()) break;
                if (last_frame == frames.get_frame_number()) continue;
                
                //std::cout << "Got Frame # " << last_frame << std::endl;
                last_frame = frames.get_frame_number();
                i++;

                //use frames here
                
                                                                    // stairs.bag vs sample.bag
                rs2::video_frame color = frames.get_color_frame();  // 0.003 ms vs 0.001ms
                rs2::depth_frame depth = frames.get_depth_frame();  // 0.001ms vs 0.001ms
                rs2::points pts = pc.calculate(depth);              // 27ms vs 27ms            
                pc.map_to(color);       // 0.01ms vs 0.02ms  // Maps color values to a point in 3D space
                
                time_start = TIME_NOW;
                buff_size = sendXYZRGBPointcloud(pts, color, buffer);   // 86ms vs 9.7ms
                time_end = TIME_NOW;

                std::cout << "Frame Time: " << timeMilli(time_end - time_start).count() \
                    << " ms " << "FPS: " << 1000.0 / timeMilli(time_end - time_start).count() \
                    << "\t Buffer size: " << float(buff_size)/(1<<20) << " MBytes" << std::endl;
                duration_sum += timeMilli(time_end - time_start).count();
                buff_size_sum += buff_size;

            }
        }
        
        pipe.stop();
        
        if (send_buffer)
        {
            close(client_sock);
            close(sockfd);
        }

        // Use last Frame to display frame Info
        rs2::video_frame color = frames.get_color_frame();
        rs2::depth_frame depth = frames.get_depth_frame();
        rs2::points pts = pc.calculate(depth);

        std::cout << "\n### Video Frames H x W : " << color.get_height() << " x " << color.get_width() << std::endl;
        std::cout << "### Depth Frames H x W : " << depth.get_height() << " x " << depth.get_width() << std::endl;
        std::cout << "### # Points : " << pts.size() << std::endl;
        
        std::cout << "\n### Total Frames = " << i << std::endl;
        std::cout << "### AVG Frame Time: " << duration_sum / i << " ms" << std::endl;
        std::cout << "### AVG FPS: " << 1000.0 / (duration_sum / i) << std::endl;
        
        if (num_of_threads)
        {
            std::cout << "### OpenMP Threads : " << num_of_threads   << std::endl;
        }else
        {
            std::cout << "### Running Serialized" << std::endl;
        }

        if (compress)
        {
            std::cout << "\n### Sending Compressed Stream" << std::endl;
            std::cout << "### AVG Bytes/Frame: " << float(buff_size_sum) / (i*1000000) << " MBytes" << std::endl;
            std::cout << "### AVG Compression Ratio " << float(buff_size_sum) / ( (pts.size()/100) * 5 * sizeof(short) * i) << " %" << std::endl;
        }else
        {
            std::cout << "\n### AVG Bytes/Frame: " << float(buff_size_sum) / (i*1000000) << " MBytes" << std::endl;
            std::cout << "### AVG Filter Compress Ratio " << float(buff_size_sum) / ( (pts.size()/100) * 5 * sizeof(short) * i) << " %" << std::endl;
        }
    }

    free(buffer);
    return 0;
}

int copyPointCloudXYZRGBToBufferSIMD(rs2::points& pts, const rs2::video_frame& color, short * pc_buffer)
{
    const auto vert = pts.get_vertices();
    const rs2::texture_coordinate* tcrd = pts.get_texture_coordinates();
    const uint8_t* color_data = reinterpret_cast<const uint8_t*>(color.get_data());

    if (!initialized) {
        pts_size = pts.size();

        float w_f = color.get_width();
        float h_f = color.get_height();
        float w_min_f = w_f - 1;
        float h_min_f = h_f - 1;
        float cl_bp_f = color.get_bytes_per_pixel();
        float cl_sb_f = color.get_stride_in_bytes();

        w_v     = _mm256_broadcast_ss(&w_f);
        h_v     = _mm256_broadcast_ss(&h_f);
        w_min_v = _mm256_broadcast_ss(&w_min_f);
        h_min_v = _mm256_broadcast_ss(&h_min_f);
        cl_bp_v = _mm256_broadcast_ss(&cl_bp_f);
        cl_sb_v = _mm256_broadcast_ss(&cl_sb_f);

        initialized = true;
    }

// for (int j = 0; j < 8; j++) {
//     // float * val1 = (float *)&u_v;
//     // printf("%.3f ", val1[j]);
//     float * val2 = (float *)&w_v;
//     printf("%.3f|", val2[j]);
//     // if (color_idx[j] != 0) {
//     //     printf("%d:%d  \n", j, color_idx[j]);
//     // }
// }
// printf("\n");

    int count = 0;
    
    #pragma omp parallel for schedule(static, 10000) num_threads(num_of_threads)
    for (int i = 0; i < pts_size; i += 8) {

        int i1 = i;
        int i2 = i+1;
        int i3 = i+2;
        int i4 = i+3;
        int i5 = i+4;
        int i6 = i+5;
        int i7 = i+6;
        int i8 = i+7;

        // Performs calculations to get color indices corresponding to each point
        // Get x index
        __m256 u_v = _mm256_set_ps(tcrd[i8].u, tcrd[i7].u, tcrd[i6].u, tcrd[i5].u, tcrd[i4].u, tcrd[i3].u, tcrd[i2].u, tcrd[i1].u);
        u_v = _mm256_mul_ps(u_v, w_v);
        u_v = _mm256_round_ps(u_v, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC);
        u_v = _mm256_max_ps(u_v, zero_v);
        u_v = _mm256_min_ps(u_v, w_min_v);
        u_v = _mm256_mul_ps(u_v, cl_bp_v);

        // Get y index then calculate index in array
        __m256 v_v = _mm256_set_ps(tcrd[i1].v, tcrd[i2].v, tcrd[i3].v, tcrd[i4].v, tcrd[i5].v, tcrd[i6].v, tcrd[i7].v, tcrd[i8].v);
        v_v = _mm256_mul_ps(v_v, h_v);
        v_v = _mm256_round_ps(v_v, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC);
        v_v = _mm256_max_ps(v_v, zero_v);
        v_v = _mm256_min_ps(v_v, h_min_v);
        v_v = _mm256_fmadd_ps(v_v, cl_sb_v, u_v);

        // Convert simd vectors into integer array in memory
        __m256i color_idx_vi = _mm256_cvtps_epi32(v_v);
        __attribute__((aligned(32))) int color_idx[8];
        _mm256_store_si256((__m256i *)color_idx, color_idx_vi);


        // Point Transformation using SIMD instructions for Matrix Multiplication on xyz points
        __m256 x12 = _mm256_set_ps(vert[i2].x, vert[i2].x, vert[i2].x, vert[i2].x, vert[i].x, vert[i].x, vert[i].x, vert[i].x);
        __m256 y12 = _mm256_set_ps(vert[i2].y, vert[i2].y, vert[i2].y, vert[i2].y, vert[i].y, vert[i].y, vert[i].y, vert[i].y);
        __m256 z12 = _mm256_set_ps(vert[i2].z, vert[i2].z, vert[i2].z, vert[i2].z, vert[i].z, vert[i].z, vert[i].z, vert[i].z);
        __m256 v12 = _mm256_fmadd_ps(x12, mat_x_v, mat_d_v);
        v12 = _mm256_fmadd_ps(y12, mat_y_v, v12);
        v12 = _mm256_fmadd_ps(z12, mat_z_v, v12);
        v12 = _mm256_mul_ps(v12, conv_rate_v);

        // Store into memory
        __attribute__((aligned(16))) float v_temp12[8];
        _mm256_store_ps(v_temp12, v12);

        // pc_buffer[i * 5 + 0] = short(v_temp12[0]);
        // pc_buffer[i * 5 + 1] = short(v_temp12[1]);
        // pc_buffer[i * 5 + 2] = short(v_temp12[2]);
        // pc_buffer[i * 5 + 3] = color_data[color_idx[0]] + (color_data[color_idx[0] + 1] << 8);
        // pc_buffer[i * 5 + 4] = color_data[color_idx[0] + 2];
        // pc_buffer[i * 5 + 5] = short(v_temp12[4]);
        // pc_buffer[i * 5 + 6] = short(v_temp12[5]);
        // pc_buffer[i * 5 + 7] = short(v_temp12[6]);
        // pc_buffer[i * 5 + 8] = color_data[color_idx[1]] + (color_data[color_idx[1] + 1] << 8);
        // pc_buffer[i * 5 + 9] = color_data[color_idx[1] + 2];

        __m256 x34 = _mm256_set_ps(vert[i4].x, vert[i4].x, vert[i4].x, vert[i4].x, vert[i3].x, vert[i3].x, vert[i3].x, vert[i3].x);
        __m256 y34 = _mm256_set_ps(vert[i4].y, vert[i4].y, vert[i4].y, vert[i4].y, vert[i3].y, vert[i3].y, vert[i3].y, vert[i3].y);
        __m256 z34 = _mm256_set_ps(vert[i4].z, vert[i4].z, vert[i4].z, vert[i4].z, vert[i3].z, vert[i3].z, vert[i3].z, vert[i3].z);
        __m256 v34 = _mm256_fmadd_ps(x34, mat_x_v, mat_d_v);
        v34 = _mm256_fmadd_ps(y34, mat_y_v, v34);
        v34 = _mm256_fmadd_ps(z34, mat_z_v, v34);
        v34 = _mm256_mul_ps(v34, conv_rate_v);

        __attribute__((aligned(16))) float v_temp34[8];
        _mm256_store_ps(v_temp34, v34);

        // pc_buffer[i * 5 + 10] = short(v_temp34[0]);
        // pc_buffer[i * 5 + 11] = short(v_temp34[1]);
        // pc_buffer[i * 5 + 12] = short(v_temp34[2]);
        // pc_buffer[i * 5 + 13] = color_data[color_idx[2]] + (color_data[color_idx[2] + 1] << 8);
        // pc_buffer[i * 5 + 14] = color_data[color_idx[2] + 2];
        
        // //v4
        // pc_buffer[i * 5 + 15] = short(v_temp34[4]);
        // pc_buffer[i * 5 + 16] = short(v_temp34[5]);
        // pc_buffer[i * 5 + 17] = short(v_temp34[6]);
        // pc_buffer[i * 5 + 18] = color_data[color_idx[3]] + (color_data[color_idx[3] + 1] << 8);
        // pc_buffer[i * 5 + 19] = color_data[color_idx[3] + 2];

        __m256 x56 = _mm256_set_ps(vert[i6].x, vert[i6].x, vert[i6].x, vert[i6].x, vert[i5].x, vert[i5].x, vert[i5].x, vert[i5].x);
        __m256 y56 = _mm256_set_ps(vert[i6].y, vert[i6].y, vert[i6].y, vert[i6].y, vert[i5].y, vert[i5].y, vert[i5].y, vert[i5].y);
        __m256 z56 = _mm256_set_ps(vert[i6].z, vert[i6].z, vert[i6].z, vert[i6].z, vert[i5].z, vert[i5].z, vert[i5].z, vert[i5].z);
        __m256 v56 = _mm256_fmadd_ps(x56, mat_x_v, mat_d_v);
        v56 = _mm256_fmadd_ps(y56, mat_y_v, v56);
        v56 = _mm256_fmadd_ps(z56, mat_z_v, v56);
        v56 = _mm256_mul_ps(v56, conv_rate_v);

        __attribute__((aligned(16))) float v_temp56[8];
        _mm256_store_ps(v_temp56, v56);

         //v5
        // pc_buffer[i * 5 + 20] = short(v_temp56[0]);
        // pc_buffer[i * 5 + 21] = short(v_temp56[1]);
        // pc_buffer[i * 5 + 22] = short(v_temp56[2]);
        // pc_buffer[i * 5 + 23] = color_data[color_idx[4]] + (color_data[color_idx[4] + 1] << 8);
        // pc_buffer[i * 5 + 24] = color_data[color_idx[4] + 2];
        
        // //v6
        // pc_buffer[i * 5 + 25] = short(v_temp56[4]);
        // pc_buffer[i * 5 + 26] = short(v_temp56[5]);
        // pc_buffer[i * 5 + 27] = short(v_temp56[6]);
        // pc_buffer[i * 5 + 28] = color_data[color_idx[5]] + (color_data[color_idx[5] + 1] << 8);
        // pc_buffer[i * 5 + 29] = color_data[color_idx[5] + 2];

        __m256 x78 = _mm256_set_ps(vert[i8].x, vert[i8].x, vert[i8].x, vert[i8].x, vert[i7].x, vert[i7].x, vert[i7].x, vert[i7].x);
        __m256 y78 = _mm256_set_ps(vert[i8].y, vert[i8].y, vert[i8].y, vert[i8].y, vert[i7].y, vert[i7].y, vert[i7].y, vert[i7].y);
        __m256 z78 = _mm256_set_ps(vert[i8].z, vert[i8].z, vert[i8].z, vert[i8].z, vert[i7].z, vert[i7].z, vert[i7].z, vert[i7].z);
        __m256 v78 = _mm256_fmadd_ps(x78, mat_x_v, mat_d_v);
        v78 = _mm256_fmadd_ps(y78, mat_y_v, v78);
        v78 = _mm256_fmadd_ps(z78, mat_z_v, v78);
        v78 = _mm256_mul_ps(v78, conv_rate_v);

        __attribute__((aligned(16))) float v_temp78[8];
        _mm256_store_ps(v_temp78, v78);

        //v7
        // pc_buffer[i * 5 + 30] = short(v_temp78[0]);
        // pc_buffer[i * 5 + 31] = short(v_temp78[1]);
        // pc_buffer[i * 5 + 32] = short(v_temp78[2]);
        // pc_buffer[i * 5 + 33] = color_data[color_idx[6]] + (color_data[color_idx[6] + 1] << 8);
        // pc_buffer[i * 5 + 34] = color_data[color_idx[6] + 2];
        
        // //v8
        // pc_buffer[i * 5 + 35] = short(v_temp78[4]);
        // pc_buffer[i * 5 + 36] = short(v_temp78[5]);
        // pc_buffer[i * 5 + 37] = short(v_temp78[6]);
        // pc_buffer[i * 5 + 38] = color_data[color_idx[7]] + (color_data[color_idx[7] + 1] << 8);
        // pc_buffer[i * 5 + 39] = color_data[color_idx[7] + 2];


        if (cutoff) {
            // Check if x and z points are within range
            __m256 z_pts = _mm256_set_ps(vert[i].z, vert[i2].z, vert[i3].z, vert[i4].z, vert[i5].z, vert[i6].z, vert[i7].z, vert[i8].z);
            __m256 z_gt_lo = _mm256_cmp_ps(z_pts, z_lo_v, _CMP_GT_OS);
            __m256 z_le_hi = _mm256_cmp_ps(z_pts, z_hi_v, _CMP_LE_OS);
            __m256 z_mask = _mm256_and_ps(z_gt_lo, z_le_hi);

            __m256 x_pts = _mm256_set_ps(vert[i].x, vert[i2].x, vert[i3].x, vert[i4].x, vert[i5].x, vert[i6].x, vert[i7].x, vert[i8].x);
            __m256 x_gt_lo = _mm256_cmp_ps(x_pts, x_lo_v, _CMP_GT_OS);
            __m256 x_le_hi = _mm256_cmp_ps(x_pts, x_hi_v, _CMP_LE_OS);
            __m256 x_mask = _mm256_and_ps(x_gt_lo, x_le_hi);

            __m256 pt_mask = _mm256_and_ps(z_mask, x_mask);

            float pt_mask_f[8];
            _mm256_store_ps(pt_mask_f, pt_mask);

            //v1
            if (pt_mask_f[0] != 0) {
                pc_buffer[count * 5 + 0] = short(v_temp12[0]);
                pc_buffer[count * 5 + 1] = short(v_temp12[1]);
                pc_buffer[count * 5 + 2] = short(v_temp12[2]);
                pc_buffer[count * 5 + 3] = color_data[color_idx[0]] + (color_data[color_idx[0] + 1] << 8);
                pc_buffer[count * 5 + 4] = color_data[color_idx[0] + 2];

                count++;
            }
            
            //v2
            if (pt_mask_f[1] != 0) {
                pc_buffer[count * 5 + 0] = short(v_temp12[4]);
                pc_buffer[count * 5 + 1] = short(v_temp12[5]);
                pc_buffer[count * 5 + 2] = short(v_temp12[6]);
                pc_buffer[count * 5 + 3] = color_data[color_idx[1]] + (color_data[color_idx[1] + 1] << 8);
                pc_buffer[count * 5 + 4] = color_data[color_idx[1] + 2];

                count++;
            }
            
            //v3
            if (pt_mask_f[2] != 0) {
                pc_buffer[count * 5 + 0] = short(v_temp34[0]);
                pc_buffer[count * 5 + 1] = short(v_temp34[1]);
                pc_buffer[count * 5 + 2] = short(v_temp34[2]);
                pc_buffer[count * 5 + 3] = color_data[color_idx[2]] + (color_data[color_idx[2] + 1] << 8);
                pc_buffer[count * 5 + 4] = color_data[color_idx[2] + 2];

                count++;
            }

            //v4
            if (pt_mask_f[3] != 0) {
                pc_buffer[count * 5 + 0] = short(v_temp34[4]);
                pc_buffer[count * 5 + 1] = short(v_temp34[5]);
                pc_buffer[count * 5 + 2] = short(v_temp34[6]);
                pc_buffer[count * 5 + 3] = color_data[color_idx[3]] + (color_data[color_idx[3] + 1] << 8);
                pc_buffer[count * 5 + 4] = color_data[color_idx[3] + 2];

                count++;
            }

            //v5
            if (pt_mask_f[4] != 0) {
                pc_buffer[count * 5 + 0] = short(v_temp56[0]);
                pc_buffer[count * 5 + 1] = short(v_temp56[1]);
                pc_buffer[count * 5 + 2] = short(v_temp56[2]);
                pc_buffer[count * 5 + 3] = color_data[color_idx[4]] + (color_data[color_idx[4] + 1] << 8);
                pc_buffer[count * 5 + 4] = color_data[color_idx[4] + 2];

                count++;
            }
            
            //v6
            if (pt_mask_f[5] != 0) {
                pc_buffer[count * 5 + 0] = short(v_temp56[4]);
                pc_buffer[count * 5 + 1] = short(v_temp56[5]);
                pc_buffer[count * 5 + 2] = short(v_temp56[6]);
                pc_buffer[count * 5 + 3] = color_data[color_idx[5]] + (color_data[color_idx[5] + 1] << 8);
                pc_buffer[count * 5 + 4] = color_data[color_idx[5] + 2];

                count++;
            }
            
            //v7
            if (pt_mask_f[6] != 0) {
                pc_buffer[count * 5 + 0] = short(v_temp78[0]);
                pc_buffer[count * 5 + 1] = short(v_temp78[1]);
                pc_buffer[count * 5 + 2] = short(v_temp78[2]);
                pc_buffer[count * 5 + 3] = color_data[color_idx[6]] + (color_data[color_idx[6] + 1] << 8);
                pc_buffer[count * 5 + 4] = color_data[color_idx[6] + 2];

                count++;
            }

            //v8
            if (pt_mask_f[7] != 0) {
                pc_buffer[count * 5 + 0] = short(v_temp78[4]);
                pc_buffer[count * 5 + 1] = short(v_temp78[5]);
                pc_buffer[count * 5 + 2] = short(v_temp78[6]);
                pc_buffer[count * 5 + 3] = color_data[color_idx[7]] + (color_data[color_idx[7] + 1] << 8);
                pc_buffer[count * 5 + 4] = color_data[color_idx[7] + 2];

                count++;
            }
        }
        else {

            //v1
            pc_buffer[i * 5 + 0] = short(v_temp12[0]);
            pc_buffer[i * 5 + 1] = short(v_temp12[1]);
            pc_buffer[i * 5 + 2] = short(v_temp12[2]);
            pc_buffer[i * 5 + 3] = color_data[color_idx[0]] + (color_data[color_idx[0] + 1] << 8);
            pc_buffer[i * 5 + 4] = color_data[color_idx[0] + 2];
            
            //v2
            pc_buffer[i * 5 + 5] = short(v_temp12[4]);
            pc_buffer[i * 5 + 6] = short(v_temp12[5]);
            pc_buffer[i * 5 + 7] = short(v_temp12[6]);
            pc_buffer[i * 5 + 8] = color_data[color_idx[1]] + (color_data[color_idx[1] + 1] << 8);
            pc_buffer[i * 5 + 9] = color_data[color_idx[1] + 2];
            
            //v3
            pc_buffer[i * 5 + 10] = short(v_temp34[0]);
            pc_buffer[i * 5 + 11] = short(v_temp34[1]);
            pc_buffer[i * 5 + 12] = short(v_temp34[2]);
            pc_buffer[i * 5 + 13] = color_data[color_idx[2]] + (color_data[color_idx[2] + 1] << 8);
            pc_buffer[i * 5 + 14] = color_data[color_idx[2] + 2];
            
            //v4
            pc_buffer[i * 5 + 15] = short(v_temp34[4]);
            pc_buffer[i * 5 + 16] = short(v_temp34[5]);
            pc_buffer[i * 5 + 17] = short(v_temp34[6]);
            pc_buffer[i * 5 + 18] = color_data[color_idx[3]] + (color_data[color_idx[3] + 1] << 8);
            pc_buffer[i * 5 + 19] = color_data[color_idx[3] + 2];

            //v5
            pc_buffer[i * 5 + 20] = short(v_temp56[0]);
            pc_buffer[i * 5 + 21] = short(v_temp56[1]);
            pc_buffer[i * 5 + 22] = short(v_temp56[2]);
            pc_buffer[i * 5 + 23] = color_data[color_idx[4]] + (color_data[color_idx[4] + 1] << 8);
            pc_buffer[i * 5 + 24] = color_data[color_idx[4] + 2];
            
            //v6
            pc_buffer[i * 5 + 25] = short(v_temp56[4]);
            pc_buffer[i * 5 + 26] = short(v_temp56[5]);
            pc_buffer[i * 5 + 27] = short(v_temp56[6]);
            pc_buffer[i * 5 + 28] = color_data[color_idx[5]] + (color_data[color_idx[5] + 1] << 8);
            pc_buffer[i * 5 + 29] = color_data[color_idx[5] + 2];
            
            //v7
            pc_buffer[i * 5 + 30] = short(v_temp78[0]);
            pc_buffer[i * 5 + 31] = short(v_temp78[1]);
            pc_buffer[i * 5 + 32] = short(v_temp78[2]);
            pc_buffer[i * 5 + 33] = color_data[color_idx[6]] + (color_data[color_idx[6] + 1] << 8);
            pc_buffer[i * 5 + 34] = color_data[color_idx[6] + 2];
            
            //v8
            pc_buffer[i * 5 + 35] = short(v_temp78[4]);
            pc_buffer[i * 5 + 36] = short(v_temp78[5]);
            pc_buffer[i * 5 + 37] = short(v_temp78[6]);
            pc_buffer[i * 5 + 38] = color_data[color_idx[7]] + (color_data[color_idx[7] + 1] << 8);
            pc_buffer[i * 5 + 39] = color_data[color_idx[7] + 2];

        }
    }
    
    if (cutoff)
        return count;
    return pts_size;
}

// Converts the XYZ values into shorts for less memory overhead,
// and puts the XYZRGB values of each point into the buffer.
int copyPointCloudXYZRGBToBuffer(rs2::points& pts, const rs2::video_frame& color, short * pc_buffer) {

    const auto vertices = pts.get_vertices();
    const rs2::texture_coordinate* tex_coords = pts.get_texture_coordinates();
    const uint8_t* color_data = reinterpret_cast<const uint8_t*>(color.get_data());

    const int pts_size = pts.size();
    const int w = color.get_width();
    const int h = color.get_height();
    const int cl_bp = color.get_bytes_per_pixel();
    const int cl_sb = color.get_stride_in_bytes();
    const int w_min = w - 1;
    const int h_min = h - 1;
    
    // TODO Optimize return size

    // use 4 threads
    #pragma omp parallel for schedule(static, 10000) num_threads(num_of_threads)
    for (int i = 0; i < pts_size; i++) {

        if (cutoff)
        {
            if (!vertices[i].z) continue;
            if (!vertices[i].x) continue;
            if (vertices[i].z > 1.5) continue;
            if (!(-2 < vertices[i].x < 2)) continue;
        }
               
        int x = std::min(std::max(int(tex_coords[i].u*w + .5f), 0), w_min);
        int y = std::min(std::max(int(tex_coords[i].v*h + .5f), 0), h_min);
        
        int idx = x * cl_bp + y * cl_sb;
        
        pc_buffer[i * 5    ] = static_cast<short>(vertices[i].x * CONV_RATE);
        pc_buffer[i * 5 + 1] = static_cast<short>(vertices[i].y * CONV_RATE);
        pc_buffer[i * 5 + 2] = static_cast<short>(vertices[i].z * CONV_RATE);
        pc_buffer[i * 5 + 3] = color_data[idx] + (color_data[idx + 1] << 8);
        pc_buffer[i * 5 + 4] = color_data[idx + 2];
    }

    return pts_size;

}

int sendXYZRGBPointcloud(rs2::points pts, rs2::video_frame color, short * buffer) {
    int size;
    
    // Clean Buffer
    memset(buffer, 0, BUF_SIZE);

    //TODO some issues with the buffer offset, on the receiver buff+short but size is int

    if (use_simd)
    {
        size = copyPointCloudXYZRGBToBufferSIMD(pts, color, &buffer[0] + sizeof(short));
    }else
    {
        size = copyPointCloudXYZRGBToBuffer(pts, color, &buffer[0] + sizeof(short));
    }
    
    // Size in bytes of the payload
    size = 5 * size * sizeof(short);
    
    // if (compress)
    // {
    //     std::string comp_buff;
    //     int comp_size = snappy::Compress((const char*)buffer, size, &comp_buff);

    //     if (send_buffer)
    //     {
    //         // copy compressed buffer
    //         memcpy(&buffer[0] + sizeof(int), (char *)&comp_size, comp_size);
    //         size = comp_size;
    //     }else
    //     {
    //         return comp_size;
    //     }
    // }
    
    if (send_buffer)
    {   
        // Include the number of bytes in the payload
        memcpy(buffer, &size, sizeof(int));
        send(client_sock, (char *)buffer, size + sizeof(int), 0);
    }
    
    return size;
}