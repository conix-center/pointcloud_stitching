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
#include <pcl/filters/voxel_grid.h>
#include "mqtt/client.h"

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

const int NUM_CAMERAS = 8;

const int CLIENT_PORT = 8000;
const int SERVER_PORT = 9000;
const int BUF_SIZE = 4000000;
const int STITCHED_BUF_SIZE = 32000000;
const float CONV_RATE = 1000.0;
const char PULL_XYZ = 'Y';
const char PULL_XYZRGB = 'Z';
const std::string MQTT_SERVER_ADDR("tcp://192.168.1.112:1883");
const std::string TOPIC("orientation");
const std::string MQTT_CLIENT_ID("sewing_machine");
const std::string IP_ADDRESS[NUM_CAMERAS] = {"192.168.1.128", "192.168.1.142", "192.168.1.138", "192.168.1.114", 
                                             "192.168.1.109", "192.168.1.113", "192.168.1.149", "192.168.1.131"};

bool fast = false;
bool timer = false;
bool save = false;
bool visual = false;
int downsample = 1;
int framecount = 0;
int server_sockfd = 0;
int client_sockfd = 0;
int sockfd_array[NUM_CAMERAS];
short * stitched_buf;
Eigen::Matrix4f transform[NUM_CAMERAS];
std::thread pcs_thread[NUM_CAMERAS];
pcl::visualization::PCLVisualizer viewer("Pointcloud Stitching");
mqtt::client client(MQTT_SERVER_ADDR, MQTT_CLIENT_ID);

// Exit gracefully by closing all open sockets
void sigintHandler(int dummy) {
    // client.disconnect();
    for (int i = 0; i < NUM_CAMERAS; i++) {
        close(sockfd_array[i]);
    }
    close(server_sockfd);
    close(client_sockfd);
}

// Parse arguments for extra runtime options
void parseArgs(int argc, char** argv) {
    int c;
    while ((c = getopt(argc, argv, "hftsvd:")) != -1) {
        switch(c) {
            // Displays the pointcloud without color, only XYZ values
            case 'f':
                fast = true;
                break;
            // Prints out the runtime of the main expensive functions
            case 't':
                timer = true;
                break;
            // Saves the first 20 frames in .ply 
            case 's':
                save = true;
                break;
            // Visualizes the pointcloud in real time
            case 'v':
                visual = true;
                break;
            // Sets downsampling factor by specified amount
            case 'd':
                downsample = atoi(optarg);
                break;
            default:
            case 'h':
                std::cout << "\nMulticamera pointcloud stitching" << std::endl;
                std::cout << "Usage: pcs-multicamera-client [options]\n" << std::endl;
                std::cout << "Options:" << std::endl;
                std::cout << " -h (help)        Display command line options" << std::endl;
                std::cout << " -f (fast)        Increases the frame rate at the expense of color" << std::endl;
                std::cout << " -t (timer)       Displays the runtime of certain functions" << std::endl;
                std::cout << " -s (save)        Saves 20 frames in a .ply format" << std::endl;
                std::cout << " -v (visualize)   Visualizes the pointclouds using PCL visualizer" << std::endl;
                std::cout << " -d (downsample)  Downsamples the stitched pointcloud by the specified integer" << std::endl;
                exit(0);
        }
    }
}

// Attempts to reconnect to MQTT broker
bool try_reconnect()
{
    constexpr int N_ATTEMPT = 30;

    for (int i = 0; i < N_ATTEMPT && !client.is_connected(); ++i) {
        try {
            client.reconnect();
            return true;
        }
        catch (const mqtt::exception&) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    return false;
}

// Setup mqtt subscriber client
void initMQTTSubscriber() {
    mqtt::connect_options conn_opts;
    conn_opts.set_keep_alive_interval(20);
    conn_opts.set_clean_session(true);
    conn_opts.set_automatic_reconnect(true);
    
    try {
        client.connect(conn_opts);
        client.subscribe(TOPIC, 1);

        while (1) {
            auto msg = client.consume_message();

            if (!msg) {
                if (!client.is_connected()) {
                    std::cout << "Lost connection. Attempting to reconnect" << std::endl;
                    if (try_reconnect()) {
                        client.subscribe(TOPIC, 1);
                        std::cout << "Reconnected" << std::endl;
                        continue;
                    }
                    else {
                        std::cout << "Reconnect failed." << std::endl;
                        break;
                    }
                }
                else
                    break;
            }

            std::cout << msg->get_topic() << ": " << msg->to_string() << std::endl;
        }

        client.disconnect();
    }
    catch (const mqtt::exception& exc) {
        std::cerr << exc.what() << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Create TCP socket with specific port and IP address.
int initSocket(int port, std::string ip_addr) {
    int sockfd;
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip_addr.c_str(), &serv_addr.sin_addr);

    // Create socket
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Couldn't create socket" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Connect to camera server
    if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection failed at " << ip_addr << "." << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "Connection made at " << sockfd << std::endl;
    return sockfd;
}

// Creates TCP server socket and connects to the visualizing computer.
int initServerSocket() {
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(SERVER_PORT);

    if ((server_sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
        std::cerr << "\nSocket fd not received." << std::endl;
        exit(EXIT_FAILURE);
    }

    if (bind(server_sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "\nBind failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    if (listen(server_sockfd, 3) < 0) {
        std::cerr << "\nListen failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "\nWaiting for VR Headset client..." << std::endl;

    if ((client_sockfd = accept(server_sockfd, NULL, NULL)) < 0) {
        std::cerr << "\nConnection failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "Established connection with unity client_sock: " << client_sockfd << std::endl;
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

    // Keep reading until N total_bytes have been read
    while (total_bytes < n) {
        if ((bytes_read = read(sockfd, buffer + total_bytes, n - total_bytes)) < 1) {
            std::cerr << "Receive failure" << std::endl;
            exit(EXIT_FAILURE);
        }

        total_bytes += bytes_read;
    }
}

// Parses the buffer and converts the short values into float points and 
// puts the XYZ and RGB values into the pointcloud.
pointCloudXYZRGB::Ptr convertBufferToPointCloudXYZRGB(short * buffer, int size) {
    int count = 0;
    pointCloudXYZRGB::Ptr new_cloud(new pointCloudXYZRGB);

    new_cloud->width = size / downsample;
    new_cloud->height = 1;
    new_cloud->is_dense = false;
    new_cloud->points.resize(new_cloud->width);

    for (int i = 0; i < size; i++) {
        if (i % downsample == 0) {
            new_cloud->points[count].x = (float)buffer[i * 5 + 0] / CONV_RATE;
            new_cloud->points[count].y = (float)buffer[i * 5 + 1] / CONV_RATE;
            new_cloud->points[count].z = (float)buffer[i * 5 + 2] / CONV_RATE;
            new_cloud->points[count].r = (uint8_t)(buffer[i * 5 + 3] & 0xFF);
            new_cloud->points[count].g = (uint8_t)(buffer[i * 5 + 3] >> 8);
            new_cloud->points[count].b = (uint8_t)(buffer[i * 5 + 4] & 0xFF);
            count++;
        }
    }
    
    return new_cloud;
}


int convertPointCloudXYZRGBToBuffer(pointCloudXYZRGB::Ptr cloud, short * buffer) {
    int size = 0;

    for (int i = 0; i < cloud->width; i++) {
        buffer[size * 5 + 0] = static_cast<short>(cloud->points[i].x * CONV_RATE);
        buffer[size * 5 + 1] = static_cast<short>(cloud->points[i].y * CONV_RATE);
        buffer[size * 5 + 2] = static_cast<short>(cloud->points[i].z * CONV_RATE);
        buffer[size * 5 + 3] = static_cast<short>(cloud->points[i].r) + static_cast<short>(cloud->points[i].g << 8);
        buffer[size * 5 + 4] = static_cast<short>(cloud->points[i].b);
      
        size++;
    }

    return size;
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

    // Read the first integer to determine the size being sent, then read in pointcloud
    readNBytes(sockfd, sizeof(int), (void *)&size);
    readNBytes(sockfd, size, (void *)&cloud_buf[0]);
    // Send a pull_XYZRGB request after finished reading from buffer
    sendPullRequest(sockfd, PULL_XYZRGB);

    if (timer)
        read_end_convert_start = std::chrono::high_resolution_clock::now();

    // Parse the pointcloud and perform transformation according to camera position
    *cloud = *convertBufferToPointCloudXYZRGB(&cloud_buf[0], size / sizeof(short) / 5);
    pcl::transformPointCloud(*cloud, *cloud, transform[thread_num]);

    if (timer)
        convert_end = std::chrono::high_resolution_clock::now();

    free(cloud_buf);

    if (timer) {
        loop_end = std::chrono::high_resolution_clock::now();
        std::cout << "   update XYZRGB: " << timeMilli(loop_end - loop_start).count() << " ms" << std::endl;
        std::cout << "      read:    " << timeMilli(read_end_convert_start - read_start).count() << " ms" << std::endl;
        std::cout << "      convert: " << timeMilli(convert_end - read_end_convert_start).count() << " ms" << std::endl;
    }
}

void send_stitchedXYZRGB(pointCloudXYZRGB::Ptr stitched_cloud) {
    char pull_request[1] = {0};

    // Wait for pull request
    if (recv(client_sockfd, pull_request, 1, 0) < 0) {
        std::cout << "Client disconnected" << std::endl;
        exit(0);
    }
    if (pull_request[0] == 'Z') {          // Client requests color pointcloud (XYZRGB)
        int size = convertPointCloudXYZRGBToBuffer(stitched_cloud, &stitched_buf[0] + sizeof(short));
        size = 5 * size * sizeof(short);
        memcpy(stitched_buf, &size, sizeof(int));
        
        write(client_sockfd, (char *)stitched_buf, size + sizeof(int));
    }
    else {                                      // Did not receive a correct pull request
        std::cerr << "Faulty pull request" << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Primary function to update the pointcloud viewer with an XYZRGB pointcloud. 
void runStitching() {
    timePoint loop_start, loop_end, stitch_start, stitch_end_viewer_start;
    std::vector <pointCloudXYZRGB::Ptr, Eigen::aligned_allocator <pointCloudXYZRGB::Ptr>> cloud_ptr(NUM_CAMERAS);
    pointCloudXYZRGB::Ptr stitched_cloud(new pointCloudXYZRGB);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> cloud_handler(stitched_cloud);

    if (visual) {
        viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
        viewer.addPointCloud(stitched_cloud, cloud_handler, "cloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    }

    // Initializing cloud pointers, pointcloud viewer, and sending pull 
    // requests to each camera server.
    for (int i = 0; i < NUM_CAMERAS; i++) {
        cloud_ptr[i] = pointCloudXYZRGB::Ptr(new pointCloudXYZRGB);
        sendPullRequest(sockfd_array[i], PULL_XYZRGB);
    }

    // Loop until the visualizer is stopped
    while (1) {
        if (timer)
            loop_start = std::chrono::high_resolution_clock::now();

        stitched_cloud->clear();

        if (timer)
            stitch_start = std::chrono::high_resolution_clock::now();

        // Spawn a thread for each camera and update pointcloud, and perform transformation, 
        for (int i = 0; i < NUM_CAMERAS; i++) {
            pcs_thread[i] = std::thread(updateCloudXYZRGB, i, sockfd_array[i], cloud_ptr[i]);
        }

        // Wait for thread to finish running, then add cloud to stitched cloud
        for (int i = 0; i < NUM_CAMERAS; i++) {
            pcs_thread[i].join();
            *stitched_cloud += *cloud_ptr[i];
        }

        if (timer)
            stitch_end_viewer_start = std::chrono::high_resolution_clock::now();
      
        // Update the pointcloud visualizer
        if (visual) {
            viewer.updatePointCloud(stitched_cloud, "cloud");
            viewer.spinOnce();

            if (viewer.wasStopped()) {
                exit(0);
            }
        }
        else {
            // std::thread send_thread(send_stitchedXYZRGB, stitched_cloud);
            // send_thread.detach();
            send_stitchedXYZRGB(stitched_cloud);
        }

        if (timer) {
            loop_end = std::chrono::high_resolution_clock::now();
            std::cout << "Stitching: " << timeMilli(stitch_end_viewer_start - stitch_start).count() << " ms" << std::endl;
            std::cout << "Update Viewer: " << timeMilli(loop_end - stitch_end_viewer_start).count() << " ms" << std::endl;
            std::cout << "TOTAL FRAME TXFER: " << timeMilli(loop_end - loop_start).count() << " ms\n" << std::endl;
        }

        if (save) {
            std::string filename("pointclouds/stitched_cloud_"  + std::to_string(framecount) + ".ply");
            pcl::io::savePLYFileBinary(filename, *stitched_cloud);
            std::cout << "Saved frame " << framecount << std::endl;
            framecount++;
            if (framecount == 20)
                save = false;
        }
    }
}

int main(int argc, char** argv) {
    parseArgs(argc, argv);

    stitched_buf = (short *)malloc(sizeof(short) * STITCHED_BUF_SIZE);

    /* Reminder: how transformation matrices work :
                 |-------> This column is the translation, which represents the location of the camera with respect to the origin
    | r00 r01 r02 x |  \
    | r10 r11 r12 y |   }-> Replace the 3x3 "r" matrix on the left with the rotation matrix
    | r20 r21 r22 z |  /
    |   0   0   0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)
    */

    transform[0] << -0.99978648,  0.01457570, -0.01464761,  0.00000000,
                     0.01960366,  0.44486850, -0.89538132,  3.80400000,
                    -0.00653455, -0.89547729, -0.44505924,  1.90200000,
                     0.00000000,  0.00000000,  0.00000000,  1.00000000;

    transform[1] << -0.70002499,  0.22321775, -0.67833535,  2.29000000,
                     0.71304871,  0.27045565, -0.64685028,  1.14000000,
                     0.03907116, -0.93649751, -0.34849084,  1.84000000,
                     0.00000000,  0.00000000,  0.00000000,  1.00000000;

    transform[2] <<  0.03266230,  0.33810649, -0.94054089,  3.56200000,
                     0.99946644, -0.01104923,  0.03073663, -0.51300000,
                     0.00000000, -0.94104299, -0.33828699,  1.85800000,
                     0.00000000,  0.00000000,  0.00000000,  1.00000000;

    transform[3] <<  0.76474240,  0.19157836, -0.61519656,  1.33000000,
                     0.64433615, -0.22737835,  0.73015753, -2.37300000,
                     0.00000000, -0.95477579, -0.29732673,  1.85700000,
                     0.00000000,  0.00000000,  0.00000000,  1.00000000;

    transform[4] <<  0.99946111, -0.00736853,  0.03198730, -0.56300000,
                    -0.03266213, -0.32020888,  0.94678374, -3.57500000,
                     0.00326621, -0.94731831, -0.32027699,  1.93800000,
                     0.00000000,  0.00000000,  0.00000000,  1.00000000;

    transform[5] <<  0.65307259, -0.33683790,  0.67825984, -2.21000000,
                    -0.75704435, -0.31344025,  0.57327050, -1.54400000,
                     0.01949470, -0.88786003, -0.45970047,  1.96700000,
                     0.00000000,  0.00000000,  0.00000000,  1.00000000;

    transform[6] <<  0.03272976, -0.42263462,  0.90570897, -3.56300000,
                    -0.99825772, -0.05833658,  0.00885238,  0.12100000,
                     0.04909464, -0.90442071, -0.42380762,  1.97000000,
                     0.00000000,  0.00000000,  0.00000000,  1.00000000;

    transform[7] << -0.79364388, -0.22495960,  0.56526327, -1.97600000,
                    -0.60824347,  0.27352615, -0.74513309,  1.81300000,
                     0.01301056, -0.93518801, -0.35391257,  1.96700000,
                     0.00000000,  0.00000000,  0.00000000,  1.00000000;

    //std::thread mqtt_thread = std::thread(initMQTTSubscriber);
    // mqtt_thread.join();

    // Init a TCP socket for each camera server

    for (int i = 0; i < NUM_CAMERAS; i++) {
        sockfd_array[i] = initSocket(CLIENT_PORT + i, IP_ADDRESS[i]);
    }

    if (!visual)
        initServerSocket();

    signal(SIGINT, sigintHandler);

    // if (fast)
    //     runFastStitching();
    // else
    //     runStitching();
    runStitching();

    // client.disconnect();
    for (int i = 0; i < NUM_CAMERAS; i++) {
        close(sockfd_array[i]);
    }
    close(server_sockfd);
    close(client_sockfd);
    
    return 0;
}


/*  Functions for no-color (XYZ only) pointclouds, functionality may not be 100%

    // Primary function to update the pointcloud viewer with an XYZ pointcloud. 
    void runFastStitching() {
        timePoint loop_start, loop_end, stitch_start, stitch_end_viewer_start;
        timePoint transform_timer[NUM_CAMERAS * 2];
        std::vector <pointCloudXYZ::Ptr, Eigen::aligned_allocator <pointCloudXYZ::Ptr>> cloud_ptr(NUM_CAMERAS);
        pointCloudXYZ::Ptr stitched_cloud(new pointCloudXYZ);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_handler(stitched_cloud, 255, 255, 255);

        // Initializing cloud pointers, pointcloud viewer, and sending pull 
        // requests to each camera server.
        for (int i = 0; i < NUM_CAMERAS; i++) {
            sendPullRequest(sockfd_array[i], PULL_XYZ);
            cloud_ptr[i] = pointCloudXYZ::Ptr(new pointCloudXYZ);
        }
        viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
        viewer.addPointCloud(stitched_cloud, cloud_handler, "cloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");

        // Loop until the visualizer is stopped (press 'q' to stop viewer)
        while (!viewer.wasStopped()) {
            if (timer)
                loop_start = std::chrono::high_resolution_clock::now();

            stitched_cloud->clear();

            if (timer)
                stitch_start = std::chrono::high_resolution_clock::now();

            // Spawn a thread for each camera and update pointcloud
            for (int i = 0; i < NUM_CAMERAS; i++) {
                pcs_thread[i] = std::thread(updateCloudXYZ, i, sockfd_array[i], cloud_ptr[i]);
            }

            // Wait for thread to finish running, then add cloud to stitched cloud
            for (int i = 0; i < NUM_CAMERAS; i++) {
                pcs_thread[i].join();
                *stitched_cloud += *cloud_ptr[i];
            }

            if (timer)
                stitch_end_viewer_start = std::chrono::high_resolution_clock::now();

            // Update the pointcloud visualizer
            if (visual) {
                viewer.updatePointCloud(stitched_cloud, "cloud");
                viewer.spinOnce();
            }

            if (timer) {
                loop_end = std::chrono::high_resolution_clock::now();
                std::cout << "Stitching: " << timeMilli(stitch_end_viewer_start - stitch_start).count() << " ms" << std::endl;
                std::cout << "Update Viewer: " << timeMilli(loop_end - stitch_end_viewer_start).count() << " ms" << std::endl;
                std::cout << "TOTAL FRAME TXFER: " << timeMilli(loop_end - loop_start).count() << " ms\n" << std::endl;
            }

            if (save) {
                std::string filename("pointclouds/stitched_cloud_"  + std::to_string(framecount) + ".ply");
                pcl::io::savePLYFileBinary(filename, *stitched_cloud);
                std::cout << "Saved frame " << framecount << std::endl;
                framecount++;
                if (framecount == 20)
                    save = false;
            }
        }
    }

    // Parses the buffer and converts the short values into float points for
    // each point in the cloud.
    pointCloudXYZ::Ptr convertBufferToPointCloudXYZ(short * buffer, int size) {
        int count = 0;
        pointCloudXYZ::Ptr new_cloud(new pointCloudXYZ);

        new_cloud->width = size / downsample;
        new_cloud->height = 1;
        new_cloud->is_dense = false;
        new_cloud->points.resize(new_cloud->width);

        for (int i = 0; i < size; i++) {
            if (i % downsample == 0) {
                new_cloud->points[count].x = (float)buffer[i * 3 + 0] / CONV_RATE;
                new_cloud->points[count].y = (float)buffer[i * 3 + 1] / CONV_RATE;
                new_cloud->points[count].z = (float)buffer[i * 3 + 2] / CONV_RATE;
                count++;
            }
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

        // Read the first integer to determine the size being sent, then read in pointcloud
        readNBytes(sockfd, sizeof(int), (void *)&size);
        readNBytes(sockfd, size, (void *)&cloud_buf[0]);
        // Send pull_XYZ request after finished reading 
        sendPullRequest(sockfd, PULL_XYZ);

        if (timer)
            read_end_convert_start = std::chrono::high_resolution_clock::now();

        // Parse the pointcloud and perform transformation according to camera position
        *cloud = *convertBufferToPointCloudXYZ(&cloud_buf[0], size / sizeof(short) / 3);
        pcl::transformPointCloud(*cloud, *cloud, transform[thread_num]);

        if (timer)
            convert_end = std::chrono::high_resolution_clock::now();

        free(cloud_buf);

        if (timer) {
            loop_end = std::chrono::high_resolution_clock::now();
            std::cout << "   update XYZ: " << timeMilli(loop_end - loop_start).count() << " ms" << std::endl;
            std::cout << "      read:    " << timeMilli(read_end_convert_start - read_start).count() << " ms" << std::endl;
            std::cout << "      convert: " << timeMilli(convert_end - read_end_convert_start).count() << " ms" << std::endl;
        }
    }
*/