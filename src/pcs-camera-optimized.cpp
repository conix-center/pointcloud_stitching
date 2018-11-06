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

#include <sys/socket.h>
#include <netinet/in.h>

#include <librealsense2/rs.hpp>

#include <omp.h>
#include <immintrin.h>
#include <xmmintrin.h>

#define TIME_NOW    std::chrono::high_resolution_clock::now()
#define BUF_SIZE    5000000
#define CONV_RATE   1000.0
#define DOWNSAMPLE  1
#define PORT        8000



typedef std::chrono::high_resolution_clock clockTime;
typedef std::chrono::duration<double, std::milli> timeMilli;
typedef std::chrono::time_point<clockTime> timestamp;

char *filename = "samples.bag";

bool display_updates = false;
bool send_buffer = false;
bool cutoff = false;

int num_of_threads = 1;
int client_sock = 0;
int sockfd = 0;
timestamp time_start, time_end;

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

void sendXYZRGBPointcloud(rs2::points pts, rs2::video_frame color, short * buffer);

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
    while ((c = getopt(argc, argv, "hf:vst:c:")) != -1) {
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
    if (num_of_threads) std::cout << "OpenMP Threads: " << num_of_threads << std::endl;
    
    //auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    //auto resolution = std::make_pair(depth_stream.width(), depth_stream.height());
    //auto intr = depth_stream.get_intrinsics();


    //get_extrinsics(const rs2::stream_profile& from_stream, const rs2::stream_profile& to_stream)

    int i = 0, last_frame = 0;
    double duration_sum = 0;
    short *buffer = (short *)malloc(sizeof(short) * BUF_SIZE);
    
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
            sendXYZRGBPointcloud(pts, color, buffer);   // 86ms vs 9.7ms
            time_end = TIME_NOW;

            std::cout << "Frame Time: " << timeMilli(time_end - time_start).count() \
                << " ms " << "FPS: " << 1000.0 / timeMilli(time_end - time_start).count() \
                << std::endl;
            duration_sum += timeMilli(time_end - time_start).count();
        }
    }
    
    pipe.stop();
    
    if (send_buffer)
    {
        close(client_sock);
        close(sockfd);
    }

    free(buffer);

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
    
    if (num_of_threads){
        std::cout << "### OpenMP Threads : " << num_of_threads   << std::endl;
    }else{
        std::cout << "### Running Serialized" << std::endl;
    }

    return 0;
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
    
    

// 921 600 Points

#define USE_SIMD

#ifdef USE_SIMD

    const float conv_rate = CONV_RATE;
    const float point5f = .5f;

    const __m128 _conv_rate = _mm_broadcast_ss(&conv_rate);
    const __m128 _whwh      = _mm_set_ps(float(w), float(h), float(w), float(h)); // load v
    const __m128 _f5        = _mm_broadcast_ss(&point5f);
    const __m128i _zero     = _mm_set_epi32(0, 0, 0, 0);
    const __m128i _whwh_min = _mm_set_epi32(w_min, h_min, w_min, h_min);

    #pragma omp parallel for schedule(static, 10000) num_threads(num_of_threads)
    for (int i = 0; i < pts_size; i+=2) {

        int i1 = i;
        int i2 = i+1;

        auto v1 = vertices[i1];
        auto v2 = vertices[i2];
        //float v_temp[4];
        int v_temp1[4];
        int v_temp2[4];
        //short v_temp2[4];
        int xy1xy2[4];
        
        __m128 _v1 = _mm_set_ps(v1.x, v1.y, v1.z,0); // load v
        __m128 _v2 = _mm_set_ps(v2.x, v2.y, v2.z,0); // load v

        _v1 = _mm_mul_ps(_v1, _conv_rate); // multiply with _conv_rate
        _v2 = _mm_mul_ps(_v2, _conv_rate); // multiply with _conv_rate
        
        __m128i __v1 = _mm_cvttps_epi32(_v1); // floats to ints
        __m128i __v2 = _mm_cvttps_epi32(_v2); // floats to ints
        
        _mm_store_si128((__m128i*)v_temp1, __v1); // store ints
        _mm_store_si128((__m128i*)v_temp2, __v2);


        //Set cutoff range for pixel points, to lower data size, and omit outlying points
        // if (!v1.z) continue;
        // if (!v1.x) continue;
        // if (v1.z > 4) continue;
        // if (!(-2 < v1.x < 2)) continue;

        // if (!v2.z) continue;
        // if (!v2.x) continue;
        // if (v2.z > 4) continue;
        // if (!(-2 < v2.x < 2)) continue;


        // SIMD
        __m128 _xy1xy2 = _mm_set_ps(tex_coords[i1].u, tex_coords[i1].v, 
                                    tex_coords[i2].u, tex_coords[i2].u); // load v
        _xy1xy2 = _mm_fmadd_ps(_xy1xy2, _whwh, _f5);    // fma
        __m128i _xy1xy2i = _mm_cvttps_epi32(_xy1xy2);    // floats to ints
        _xy1xy2i = _mm_max_epi32(_xy1xy2i, _zero);
        _xy1xy2i = _mm_min_epi32(_xy1xy2i, _whwh_min);
        _mm_store_si128((__m128i*)xy1xy2, _xy1xy2i);
        int idx1 = xy1xy2[3] * cl_bp + xy1xy2[2] * cl_sb;
        int idx2 = xy1xy2[1] * cl_bp + xy1xy2[0] * cl_sb;

        // vs

        // NON SMID       
        // int x1 = std::min(std::max(int(tex_coords[i1].u*w + .5f), 0), w_min);
        // int y1 = std::min(std::max(int(tex_coords[i1].v*h + .5f), 0), h_min);
        // int x2 = std::min(std::max(int(tex_coords[i2].u*w + .5f), 0), w_min);
        // int y2 = std::min(std::max(int(tex_coords[i2].v*h + .5f), 0), h_min);
        // int idx1 = x1 * cl_bp + y1 * cl_sb;
        // int idx2 = x2 * cl_bp + y2 * cl_sb;

        
        pc_buffer[i * 5 + 0] = v_temp1[3];
        pc_buffer[i * 5 + 1] = v_temp1[2];
        pc_buffer[i * 5 + 2] = v_temp1[1];

        // pc_buffer[i * 5 + 0] = static_cast<short>(v.x * CONV_RATE);
        // pc_buffer[i * 5 + 1] = static_cast<short>(v.y * CONV_RATE);
        // pc_buffer[i * 5 + 2] = static_cast<short>(v.z * CONV_RATE);

        pc_buffer[i * 5 + 3] = color_data[idx1] + (color_data[idx1 + 1] << 8);
        pc_buffer[i * 5 + 4] = color_data[idx1 + 2];


        pc_buffer[i * 5 + 5] = v_temp2[3];
        pc_buffer[i * 5 + 6] = v_temp2[2];
        pc_buffer[i * 5 + 7] = v_temp2[1];

        pc_buffer[i * 5 + 8] = color_data[idx2] + (color_data[idx2 + 1] << 8);
        pc_buffer[i * 5 + 9] = color_data[idx2 + 2];


        //__m256d pd_a = _mm256_set_pd(0, tf_mat[8], tf_mat[4], tf_mat[0]);

    }
    
    return pts_size;

#else

    // TODO Optimize return size

    //for (int i = 0; i < pts.size() && (5 * size + 2) < BUF_SIZE; i++) {

    // use 4 threads
    #pragma omp parallel for schedule(static, 10000) num_threads(num_of_threads)
    for (int i = 0; i < pts_size; i++) {

        //Set cutoff range for pixel points, to lower data size, and omit outlying points
        // if (!vertices[i].z) continue;
        // if (!vertices[i].x) continue;
        // if (vertices[i].z > 4) continue;
        // if (!(-2 < vertices[i].x < 2)) continue;
                
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

#endif
}

void sendXYZRGBPointcloud(rs2::points pts, rs2::video_frame color, short * buffer) {
    // Add size of buffer to beginning of message
    int size = copyPointCloudXYZRGBToBuffer(pts, color, &buffer[0] + sizeof(short));
    size = 5 * size * sizeof(short);
    memcpy(buffer, &size, sizeof(int));

    if (send_buffer) send(client_sock, (char *)buffer, size + sizeof(int), 0);
}