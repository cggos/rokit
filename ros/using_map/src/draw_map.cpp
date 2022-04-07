#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "draw_map");

    ros::NodeHandle n;

    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("nav_map", 1);

    int x_min = -10, x_max = 10;
    int y_min = -10, y_max = 10;

    nav_msgs::MapMetaData mmd;
    mmd.resolution = 0.05;
    mmd.origin.position.x = 0;
    mmd.origin.position.y = 0;
    mmd.origin.position.z = 0;
    mmd.origin.orientation.w = 0.0;
    mmd.width = x_max - x_min;
    mmd.height = y_max - y_min;

    nav_msgs::OccupancyGrid map;

    map.info = mmd;
    map.header.frame_id = "/my_frame";
    map.header.stamp = ros::Time::now();

    map.data.resize(mmd.width * mmd.height);

    for (int i = 0; i < map.data.size(); i++)
        map.data[i] = -1;

    map.data[0] = 0;
    map.data[209] = 50;
    map.data[399] = 100;

    ros::Rate r(1);
    while (ros::ok()) {
        map_pub.publish(map);

        r.sleep();
    }
}
