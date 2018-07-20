/*
 * Evan Li
 * pcs-multicamera-client.cpp
 *
 * Creates multiple TCP connections where each connection is sending
 * pointclouds in realtime to the client for post processing and 
 * visualization. Each pointcloud is rotated and translated through
 * a hardcoded transform that was precomputed from a camera registration
 * step. 
 */

#include <librealsense2/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdio.h>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <chrono>
#include <thread>

typedef pcl::PointCloud<pcl::PointXYZ> pointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> pointCloudXYZRGB;
typedef std::chrono::high_resolution_clock clockTime;
typedef std::chrono::time_point<clockTime> timePoint;
typedef std::chrono::duration<double, std::milli> timeMilli;

const int PORT = 8000;
const int BUF_SIZE = 4000000;
const int NUM_CAMERAS = 1;
const float CONV_RATE = 1000.0;
const char PULL_XYZ = 'Y';
const char PULL_XYZRGB = 'Z';
const std::string MQTT_SERVER_ADDR("tcp://192.168.0.113:1883");
const std::string MQTT_CLIENT_ID("sewing_machine");
const std::string IP_ADDRESS[2] = {"192.168.0.113", "192.168.0.113"}; 

bool fast = false;
bool timer = false;
bool save = false;
int framecount = 0;
int sockfd[NUM_CAMERAS];
Eigen::Matrix4f transform[2];
std::thread pcs_thread[NUM_CAMERAS];
pcl::visualization::PCLVisualizer viewer("Pointcloud stitching");

void sigintHandler(int dummy) {
    for (int i = 0; i < NUM_CAMERAS; i++) {
        close(sockfd[i]);
    }
}

void parseArgs(int argc, char** argv) {
    int c;
    while ((c = getopt(argc, argv, "hfts")) != -1) {
        switch(c) {
            case 'f':
                fast = true;
                break;
            case 't':
                timer = true;
                break;
            case 's':
                save = true;
                break;
            default:
            case 'h':
                std::cout << "\nMulticamera pointcloud stitching" << std::endl;
                std::cout << "Usage: pcs-multicamera-client [options]\n" << std::endl;
                std::cout << "Options:" << std::endl;
                std::cout << " -h (help)    Display command line options" << std::endl;
                std::cout << " -f (fast)    Increases the frame rate at the expense of color (Default speed is slow)" << std::endl;
                std::cout << " -t (timer)   Displays the runtime of certain functions" << std::endl;
                std::cout << " -s (save)    Saves 20 frames in a .ply format" << std::endl;
                exit(0);
        }
    }
}

// Setup mqtt subscriber client
// void initMQTTSubscriber() {
//     mqtt::client client(MQTT_SERVER_ADDR, MQTT_CLIENT_ID);
//     mqtt::connect_options conn_opts;
//     conn_opts.set_keep_alive_interval(1);
//     conn_opts.set_clean_session(true);
//     conn_opts.set_automatic_reconnect(true);

//     client.connect(conn_opts);
//     client.subscribe("camera_id", 1);

//     while (1) {

//     }
// }

// Create TCP socket with specific port and IP address.
int initSocket(int port, std::string ip_addr) {
    int sockfd;
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip_addr.c_str(), &serv_addr.sin_addr);

    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Couldn't create socket" << std::endl;
        exit(EXIT_FAILURE);
    }

    if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection failed." << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "Connection made at " << sockfd << std::endl;
    return sockfd;
}

// Sends pull request to socket to signal server to send pointcloud data.
void sendPullRequest(int sockfd, char pull_char) {
    if (send(sockfd, &pull_char, 1, 0) < 0) {
        std::cerr << "Pull request failure from sockfd: " << sockfd << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Helper function to read N bytes from the buffer to ensure that 
// the entire buffer has been read from.
void readNBytes(int sockfd, unsigned int n, void * buffer) {
    int total_bytes, bytes_read;
    total_bytes = 0;

    while (total_bytes < n) {
        if ((bytes_read = read(sockfd, buffer + total_bytes, n - total_bytes)) < 1) {
            std::cerr << "Receive failure" << std::endl;
            exit(EXIT_FAILURE);
        }

        total_bytes += bytes_read;
    }
}

// Parses the buffer and converts the short values into float points for
// each point in the cloud.
pointCloudXYZ::Ptr convertBufferToPointCloudXYZ(short * buffer, int size) {
    pointCloudXYZ::Ptr new_cloud(new pointCloudXYZ);

    new_cloud->width = size;
    new_cloud->height = 1;
    new_cloud->is_dense = false;
    new_cloud->points.resize(new_cloud->width);

    for (int i = 0; i < size; i++) {
        new_cloud->points[i].x = (float)buffer[i * 3 + 0] / CONV_RATE;
        new_cloud->points[i].y = (float)buffer[i * 3 + 1] / CONV_RATE;
        new_cloud->points[i].z = (float)buffer[i * 3 + 2] / CONV_RATE;
    }

    return new_cloud;
}

// Parses the buffer and converts the short values into float points and 
// puts the XYZ and RGB values into the pointcloud.
pointCloudXYZRGB::Ptr convertBufferToPointCloudXYZRGB(short * buffer, int size) {
    pointCloudXYZRGB::Ptr new_cloud(new pointCloudXYZRGB);

    new_cloud->width = size;
    new_cloud->height = 1;
    new_cloud->is_dense = false;
    new_cloud->points.resize(new_cloud->width);

    for (int i = 0; i < size; i++) {
        new_cloud->points[i].x = (float)buffer[i * 5 + 0] / CONV_RATE;
        new_cloud->points[i].y = (float)buffer[i * 5 + 1] / CONV_RATE;
        new_cloud->points[i].z = (float)buffer[i * 5 + 2] / CONV_RATE;
        new_cloud->points[i].r = (uint8_t)(buffer[i * 5 + 3] & 0xFF);
        new_cloud->points[i].g = (uint8_t)(buffer[i * 5 + 3] >> 8);
        new_cloud->points[i].b = (uint8_t)(buffer[i * 5 + 4] & 0xFF);

    }
    
    return new_cloud;
}

// Reads from the buffer and converts the data into a new XYZ pointcloud.
void updateCloudXYZ(int thread_num, int sockfd, pointCloudXYZ::Ptr cloud) {
    timePoint loop_start, loop_end, read_start, read_end_convert_start, convert_end;
    if (timer)
        loop_start = std::chrono::high_resolution_clock::now();

    short * cloud_buf = (short *)malloc(sizeof(short) * BUF_SIZE);
    int size;

    if (timer)
        read_start = std::chrono::high_resolution_clock::now();

    readNBytes(sockfd, sizeof(int), (void *)&size);
    readNBytes(sockfd, size, (void *)&cloud_buf[0]);
    // Send pull_XYZ request after finished reading 
    sendPullRequest(sockfd, PULL_XYZ);

    if (timer)
        read_end_convert_start = std::chrono::high_resolution_clock::now();

    *cloud = *convertBufferToPointCloudXYZ(&cloud_buf[0], size / sizeof(short) / 3);

    if (timer)
        convert_end = std::chrono::high_resolution_clock::now();

    free(cloud_buf);

    if (timer) {
        loop_end = std::chrono::high_resolution_clock::now();
        std::cout << "update XYZ: " << timeMilli(loop_end - loop_start).count() << " ms" << std::endl;
        std::cout << "   read:    " << timeMilli(read_end_convert_start - read_start).count() << " ms" << std::endl;
        std::cout << "   convert: " << timeMilli(convert_end - read_end_convert_start).count() << " ms" << std::endl;
    }
}

// Reads from the buffer and converts the data into a new XYZRGB pointcloud.
void updateCloudXYZRGB(int thread_num, int sockfd, pointCloudXYZRGB::Ptr cloud) {
    timePoint loop_start, loop_end, read_start, read_end_convert_start, convert_end;
    if (timer)
        loop_start = std::chrono::high_resolution_clock::now();

    short * cloud_buf = (short *)malloc(sizeof(short) * BUF_SIZE);
    int size;

    if (timer)
        read_start = std::chrono::high_resolution_clock::now();

    readNBytes(sockfd, sizeof(int), (void *)&size);
    readNBytes(sockfd, size, (void *)&cloud_buf[0]);
    // Send a pull_XYZRGB request after finished reading from buffer
    sendPullRequest(sockfd, PULL_XYZRGB);

    if (timer)
        read_end_convert_start = std::chrono::high_resolution_clock::now();

    *cloud = *convertBufferToPointCloudXYZRGB(&cloud_buf[0], size / sizeof(short) / 5);

    if (timer)
        convert_end = std::chrono::high_resolution_clock::now();

    free(cloud_buf);

    if (timer) {
        loop_end = std::chrono::high_resolution_clock::now();
        std::cout << "update XYZRGB: " << timeMilli(loop_end - loop_start).count() << " ms" << std::endl;
        std::cout << "      read:    " << timeMilli(read_end_convert_start - read_start).count() << " ms" << std::endl;
        std::cout << "      convert: " << timeMilli(convert_end - read_end_convert_start).count() << " ms" << std::endl;
    }
}

// Primary function to update the pointcloud viewer with an XYZ pointcloud. 
void runFastStitching() {
    timePoint loop_start, loop_end;
    timePoint transform_timer[NUM_CAMERAS * 2];
    std::vector <pointCloudXYZ::Ptr, Eigen::aligned_allocator <pointCloudXYZ::Ptr>> cloud_ptr(NUM_CAMERAS);
    std::vector <pointCloudXYZ::Ptr, Eigen::aligned_allocator <pointCloudXYZ::Ptr>> tf_clouds(NUM_CAMERAS);
    pointCloudXYZ::Ptr stitched_cloud(new pointCloudXYZ);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_handler(stitched_cloud, 255, 255, 255);

    // Initializing cloud pointers, pointcloud viewer, and sending pull 
    // requests to each camera server.
    for (int i = 0; i < NUM_CAMERAS; i++) {
        sendPullRequest(sockfd[i], PULL_XYZ);
        cloud_ptr[i] = pointCloudXYZ::Ptr(new pointCloudXYZ);
        tf_clouds[i] = pointCloudXYZ::Ptr(new pointCloudXYZ);
    }
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer.addPointCloud(stitched_cloud, cloud_handler, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");

    while (!viewer.wasStopped()) {
        if (timer)
            loop_start = std::chrono::high_resolution_clock::now();

        stitched_cloud->clear();

        // Spawn a thread for each camera and update pointcloud
        for (int i = 0; i < NUM_CAMERAS; i++) {
            pcs_thread[i] = std::thread(updateCloudXYZ, i, sockfd[i], cloud_ptr[i]);
        }

        // Wait for thread to finish running, perform transformation, then combine clouds
        for (int i = 0; i < NUM_CAMERAS; i++) {
            pcs_thread[i].join();

            if (timer)
                transform_timer[i * 2] = std::chrono::high_resolution_clock::now();

            pcl::transformPointCloud(*cloud_ptr[i], *tf_clouds[i], transform[i]);

            if (timer) {
                transform_timer[i * 2 + 1] = std::chrono::high_resolution_clock::now();
                std::cout << "transform[" << i << "] : " << timeMilli(transform_timer[i * 2 + 1] - transform_timer[i * 2]).count() << " ms" << std::endl;
            }

            *stitched_cloud += *tf_clouds[i];
        }

        if (save) {
            std::string filename("pointclouds/stitched_cloud_"  + std::to_string(framecount) + ".ply");
            pcl::io::savePLYFileBinary(filename, *stitched_cloud);
            std::cout << "Saved frame " << framecount << std::endl;
            framecount++;
            if (framecount == 20)
                save = false;
        }

        viewer.updatePointCloud(stitched_cloud, "cloud");
        viewer.spinOnce();

        if (timer) {
            loop_end = std::chrono::high_resolution_clock::now();
            std::cout << "TOTAL FRAME TXFER: " << timeMilli(loop_end - loop_start).count() << " ms\n" << std::endl;
            std::cout << "\n" << std::endl;
        }
    }
}

// Primary function to update the pointcloud viewer with an XYZRGB pointcloud. 
void runStitching() {
    timePoint loop_start, loop_end;
    timePoint transform_timer[NUM_CAMERAS * 2];
    std::vector <pointCloudXYZRGB::Ptr, Eigen::aligned_allocator <pointCloudXYZRGB::Ptr>> cloud_ptr(NUM_CAMERAS);
    std::vector <pointCloudXYZRGB::Ptr, Eigen::aligned_allocator <pointCloudXYZRGB::Ptr>> tf_clouds(NUM_CAMERAS);
    pointCloudXYZRGB::Ptr stitched_cloud(new pointCloudXYZRGB);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> cloud_handler(stitched_cloud);

    // Initializing cloud pointers, pointcloud viewer, and sending pull 
    // requests to each camera server.
    for (int i = 0; i < NUM_CAMERAS; i++) {
        cloud_ptr[i] = pointCloudXYZRGB::Ptr(new pointCloudXYZRGB);
        tf_clouds[i] = pointCloudXYZRGB::Ptr(new pointCloudXYZRGB);
        sendPullRequest(sockfd[i], PULL_XYZRGB);
    }
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer.addPointCloud(stitched_cloud, cloud_handler, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    while (!viewer.wasStopped()) {
        if (timer)
            loop_start = std::chrono::high_resolution_clock::now();

        stitched_cloud->clear();

        // Spawn a thread for each camera and update pointcloud
        for (int i = 0; i < NUM_CAMERAS; i++) {
            pcs_thread[i] = std::thread(updateCloudXYZRGB, i, sockfd[i], cloud_ptr[i]);
        }

        // Wait for thread to finish running, perform transformation, then combine clouds
        for (int i = 0; i < NUM_CAMERAS; i++) {
            pcs_thread[i].join();

            if (timer)
                transform_timer[i * 2] = std::chrono::high_resolution_clock::now();

            pcl::transformPointCloud(*cloud_ptr[i], *tf_clouds[i], transform[i]);

            if (timer) {
                transform_timer[i * 2 + 1] = std::chrono::high_resolution_clock::now();
                std::cout << "transform[" << i << "] : " << timeMilli(transform_timer[i * 2 + 1] - transform_timer[i * 2]).count() << " ms" << std::endl;
            }

            *stitched_cloud += *tf_clouds[i];
        }

        if (save) {
            std::string filename("pointclouds/stitched_cloud_"  + std::to_string(framecount) + ".ply");
            pcl::io::savePLYFileBinary(filename, *stitched_cloud);
            std::cout << "Saved frame " << framecount << std::endl;
            framecount++;
            if (framecount == 20)
                save = false;
        }

        viewer.updatePointCloud(stitched_cloud, "cloud");
        viewer.spinOnce();

        if (timer) {
            loop_end = std::chrono::high_resolution_clock::now();
            std::cout << "TOTAL FRAME TXFER: " << timeMilli(loop_end - loop_start).count() << " ms\n" << std::endl;
        }
    }
}

int main(int argc, char** argv) {
    parseArgs(argc, argv);

    /* Reminder: how transformation matrices work :
                 |-------> This column is the translation, which represents the location of the camera with respect to the origin
    | r00 r01 r02 x |  \
    | r10 r11 r12 y |   }-> Replace the 3x3 "r" matrix on the left with the rotation matrix
    | r20 r21 r22 z |  /
    |   0   0   0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)
    */
    transform[0] << -0.99574067,  0.02631655, -0.08836260,  0.02300000,
                     0.09219821,  0.28421870, -0.95431610,  2.05300000,
                     0.00000000, -0.95839823, -0.28543446,  1.84600000,
                     0.00000000,  0.00000000,  0.00000000,  1.00000000;

    transform[1] <<  0.99056815, -0.04332334,  0.12999166, -0.51300000,
                    -0.13694491, -0.34463012,  0.92869595, -1.86300000,
                     0.00456483, -0.93773833, -0.34731253,  1.90300000,
                     0.00000000,  0.00000000,  0.00000000,  1.00000000;

    // std::thread mqtt_thread = std::thread(initMQTTSubscriber);

    for (int i = 0; i < NUM_CAMERAS; i++) {
        sockfd[i] = initSocket(8000 + i, IP_ADDRESS[i]);
    }
    signal(SIGINT, sigintHandler);

    if (fast)
        runFastStitching();
    else
        runStitching();

    for (int i = 0; i < NUM_CAMERAS; i++) {
        close(sockfd[i]);
    }
    
    return 0;
}