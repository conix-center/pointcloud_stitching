#include <librealsense2/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

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

const int PORT = 8000;
const int BUF_SIZE = 4000000;
const float CONV_RATE = 1000.0;
// const std::string IP_ADDRESS = "127.0.0.1";
// const std::string IP_ADDRESS = "192.168.0.115";
const std::string IP_ADDRESS = "192.168.0.117";

short * cloud_buf;
int sockfd = 0;

void sigintHandler(int dummy) {
    close(sockfd);
    free(cloud_buf);
}

void initSocket(int port) {
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    inet_pton(AF_INET, IP_ADDRESS.c_str(), &serv_addr.sin_addr);

    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Couldn't create socket" << std::endl;
        exit(EXIT_FAILURE);
    }

    if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection failed." << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "Connection made at " << sockfd << std::endl;
}

pcl::visualization::PCLVisualizer initViewer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    pcl::visualization::PCLVisualizer viewer("Cloud");
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> cloud_handler(cloud);
    viewer.addPointCloud (cloud, cloud_handler, "cloud");
    // viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);     // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

    return viewer;
}

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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertBufferToPointCloud(short * buffer, int size) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

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

int main(int argc, char** argv) {
    int size;
    char pull_request = 'Z';
    cloud_buf = (short *)malloc(sizeof(short) * BUF_SIZE);

    initSocket(atoi(argv[1]));
    signal(SIGINT, sigintHandler);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::PCLVisualizer viewer = initViewer(cloud);

    while (!viewer.wasStopped ()) { 	// Display the visualiser until 'q' key is pressed
    	if (send(sockfd, &pull_request, 1, 0) < 0) {
            std::cerr << "Pull request failure" << std::endl;
            return 0;
    	}

        readNBytes(sockfd, sizeof(int), (void *)&size);
        readNBytes(sockfd, size, (void *)&cloud_buf[0]);

        cloud = convertBufferToPointCloud(&cloud_buf[0], size / sizeof(short) / 5);
        viewer.updatePointCloud(cloud, "cloud");
        viewer.spinOnce ();
    }

    close(sockfd);
    free(cloud_buf);
    return 0;
}