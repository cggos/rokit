#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <iomanip>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    ROS_INFO("map size: (%d, %d)", msg->info.width, msg->info.height);

    int width = msg->info.width;
    int height = msg->info.height;

    cv::Mat mat_map(height, width, CV_8UC1);

    for (int h = 0; h < height; ++h) {
        for (int w = 0; w < width; ++w) {
            int value = msg->data[h * width + w];
            if (value == -1) value = 128;
            mat_map.at<unsigned char>(h, w) = value;
        }
    }

    int threshold = 50;
    cv::Mat mat_edge;
    cv::Canny(mat_map, mat_edge, threshold, threshold * 3, 3);

    for (int h = 0; h < height; h += 3) {
        for (int w = 0; w < width; w += 3)
            std::cout << std::setw(4) << std::left << (int)mat_map.at<unsigned char>(h, w);
        std::cout << std::endl;
    }
    std::cout << std::endl;

    cv::imshow("", mat_edge);
    cv::waitKey(30);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "read_map");

    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap", 1000, map_cb);

    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}
