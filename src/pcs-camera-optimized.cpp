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

#include "snappy.h"

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

char *filename = "samples.bag";

bool display_updates = false;
bool send_buffer = false;
bool cutoff = false;
bool use_simd = false;
bool compress = false;
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
    int buff_size = 0, buff_size_sum = 0;
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
            buff_size = sendXYZRGBPointcloud(pts, color, buffer);   // 86ms vs 9.7ms
            time_end = TIME_NOW;

            std::cout << "Frame Time: " << timeMilli(time_end - time_start).count() \
                << " ms " << "FPS: " << 1000.0 / timeMilli(time_end - time_start).count() \
                << "\t Buffer size: " << float(buff_size)/1000000 << " MBytes" << std::endl;
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

    return 0;
}

int copyPointCloudXYZRGBToBufferSIMD(rs2::points& pts, const rs2::video_frame& color, short * pc_buffer)
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

    // TODO CLEAN
    const float conv_rate = CONV_RATE;
    const float point5f = .5f;
    const float w_f = float(w);
    const float h_f = float(h);
    const float w_min_f = float(w_min);
    const float h_min_f = float(h_min);
    const float cl_bp_f = float(color.get_bytes_per_pixel());
    const float cl_sb_f = float(color.get_stride_in_bytes());

    // Float
    const __m128 _conv_rate = _mm_broadcast_ss(&conv_rate);
    const __m128 _cl_bp_f = _mm_broadcast_ss(&cl_bp_f);
    const __m128 _cl_sb_f = _mm_broadcast_ss(&cl_sb_f);
    const __m128 _w_min_f = _mm_broadcast_ss(&w_min_f);
    const __m128 _h_min_f = _mm_broadcast_ss(&h_min_f);
    const __m128 _f5 = _mm_broadcast_ss(&point5f);
    const __m128 _zero_f = _mm_setzero_ps();
    const __m128 _w = _mm_broadcast_ss(&w_f);
    const __m128 _h = _mm_broadcast_ss(&h_f);

    // integ
    const __m128i _zero = _mm_setzero_si128();
    const __m128i _w_min = _mm_set1_epi32(w_min);
    const __m128i _h_min = _mm_set1_epi32(h_min);
    const __m128i _cl_bp = _mm_set1_epi32(cl_bp);
    const __m128i _cl_sb = _mm_set1_epi32(cl_sb);
    
    
    #pragma omp parallel for schedule(static, 10000) num_threads(num_of_threads)
    for (int i = 0; i < pts_size; i += 4) {
        
        if (cutoff)
        {
            if (!vert[i].z) continue;
            if (!vert[i].x) continue;
            if (vert[i].z > 1.5) continue;
            if (!(-2 < vert[i].x < 2)) continue;
        }

        int i1 = i;
        int i2 = i+1;
        int i3 = i+2;
        int i4 = i+3;

        __attribute__((aligned(16))) int v_temp1[4];
        __attribute__((aligned(16))) int v_temp2[4];
        __attribute__((aligned(16))) int v_temp3[4];
        __attribute__((aligned(16))) int v_temp4[4];

        __attribute__((aligned(16))) int idx[4];
        __attribute__((aligned(16))) int idy[4];

        // load
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


        __m128 _v1 = _mm_set_ps(vert[i1].x, vert[i1].y, vert[i1].z, 0); // load v
        __m128 _v2 = _mm_set_ps(vert[i2].x, vert[i2].y, vert[i2].z, 0); // load v
        __m128 _v3 = _mm_set_ps(vert[i3].x, vert[i3].y, vert[i3].z, 0); // load v
        __m128 _v4 = _mm_set_ps(vert[i4].x, vert[i4].y, vert[i4].z, 0); // load v

        _v1 = _mm_mul_ps(_v1, _conv_rate); // multiply with _conv_rate
        _v2 = _mm_mul_ps(_v2, _conv_rate); // multiply with _conv_rate
        _v3 = _mm_mul_ps(_v3, _conv_rate); // multiply with _conv_rate
        _v4 = _mm_mul_ps(_v4, _conv_rate); // multiply with _conv_rate
        
        __m128i __v1 = _mm_cvttps_epi32(_v1); // floats to ints
        __m128i __v2 = _mm_cvttps_epi32(_v2); // floats to ints
        __m128i __v3 = _mm_cvttps_epi32(_v3); // floats to ints
        __m128i __v4 = _mm_cvttps_epi32(_v4); // floats to ints
        
        _mm_storeu_si128((__m128i*)v_temp1, __v1); // store ints
        _mm_storeu_si128((__m128i*)v_temp2, __v2); // store ints
        _mm_storeu_si128((__m128i*)v_temp3, __v3); // store ints
        _mm_storeu_si128((__m128i*)v_temp4, __v4); // store ints

        //v1
        if (cutoff && \
            (!vert[i].z) && \
            (!vert[i].x) && \
            (vert[i].z > 1.5) && \
            !(-2 < vert[i].x < 2) ) {
                
            pc_buffer[i * 5 + 0] = short(v_temp1[3]);
            pc_buffer[i * 5 + 1] = short(v_temp1[2]);
            pc_buffer[i * 5 + 2] = short(v_temp1[1]);
            pc_buffer[i * 5 + 3] = color_data[idx1] + (color_data[idx1 + 1] << 8);
            pc_buffer[i * 5 + 4] = color_data[idx1 + 2];
        }
        
        //v2
        if (cutoff && \
            (!vert[i+1].z) && \
            (!vert[i+1].x) && \
            (vert[i+1].z > 1.5) && \
            !(-2 < vert[i+1].x < 2) ) {
                
            pc_buffer[i * 5 + 5] = short(v_temp2[3]);
            pc_buffer[i * 5 + 6] = short(v_temp2[2]);
            pc_buffer[i * 5 + 7] = short(v_temp2[1]);
            pc_buffer[i * 5 + 8] = color_data[idx2] + (color_data[idx2 + 1] << 8);
            pc_buffer[i * 5 + 9] = color_data[idx2 + 2];
        }
        
        //v3
        if (cutoff && \
            (!vert[i+2].z) && \
            (!vert[i+2].x) && \
            (vert[i+2].z > 1.5) && \
            !(-2 < vert[i+2].x < 2) ){

            pc_buffer[i * 5 + 5] = short(v_temp3[3]);
            pc_buffer[i * 5 + 6] = short(v_temp3[2]);
            pc_buffer[i * 5 + 7] = short(v_temp3[1]);
            pc_buffer[i * 5 + 8] = color_data[idx3] + (color_data[idx3 + 1] << 8);
            pc_buffer[i * 5 + 9] = color_data[idx3 + 2];
        }

        //v4
        if (cutoff && \
            (!vert[i+3].z) && \
            (!vert[i+3].x) && \
            (vert[i+3].z > 1.5) && \
            !(-2 < vert[i+3].x < 2) ){

            pc_buffer[i * 5 + 5] = short(v_temp4[3]);
            pc_buffer[i * 5 + 6] = short(v_temp4[2]);
            pc_buffer[i * 5 + 7] = short(v_temp4[1]);
            pc_buffer[i * 5 + 8] = color_data[idx4] + (color_data[idx4 + 1] << 8);
            pc_buffer[i * 5 + 9] = color_data[idx4 + 2];
        }
    }
    
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
    
    if (compress)
    {
        std::string comp_buff;
        int comp_size = snappy::Compress((const char*)buffer, size, &comp_buff);

        if (send_buffer)
        {
            // copy compressed buffer
            memcpy(&buffer[0] + sizeof(int), (char *)&comp_size, comp_size);
            size = comp_size;
        }else
        {
            return comp_size;
        }
    }
    
    if (send_buffer)
    {   
        // Include the number of bytes in the payload
        memcpy(buffer, &size, sizeof(int));
        send(client_sock, (char *)buffer, size + sizeof(int), 0);
    }
    
    return size;
}