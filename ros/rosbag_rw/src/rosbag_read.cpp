/**
 * @file rosbag_read.cpp
 * @author Gavin Gao (cggos@outlook.com)
 * @brief ref: https://github.com/cggos/okvis_cg/blob/master/src/okvis_node_synchronous.cpp
 * @version 0.1
 * @date 2022-08-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/chunked_file.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "rosbag_read");

  rosbag::Bag bag(argv[1], rosbag::bagmode::Read);

  std::string imu_topic("/imu0");
  rosbag::View view_imu(bag, rosbag::TopicQuery(imu_topic));
  if (view_imu.size() == 0) {
    std::cerr << "no imu topic" << std::endl;
    return -1;
  }

  rosbag::View::iterator view_imu_iterator = view_imu.begin();
  std::cout << "No. IMU messages: " << view_imu.size() << std::endl;

  int numCameras = 2;

  std::vector<std::shared_ptr<rosbag::View> > view_cams_ptr;
  std::vector<rosbag::View::iterator> view_cam_iterators;
  for (size_t i = 0; i < numCameras; ++i) {
    std::string camera_topic("/cam" + std::to_string(i) + "/image_raw");
    std::shared_ptr<rosbag::View> view_ptr(new rosbag::View(bag, rosbag::TopicQuery(camera_topic)));
    if (view_ptr->size() == 0) {
      std::cout << "no camera topic" << std::endl;
      return 1;
    }
    view_cams_ptr.push_back(view_ptr);
    view_cam_iterators.push_back(view_ptr->begin());
    std::cout << "No. cam " << i << " messages: " << view_cams_ptr.back()->size() << std::endl;
  }

  // while (ros::ok()) {
  //   ros::spinOnce();

  //   for (size_t i = 0; i < numCameras; ++i) {
  //     sensor_msgs::ImageConstPtr msg1 = view_cam_iterators[i]->instantiate<sensor_msgs::Image>();

  //     do {
  //       sensor_msgs::ImuConstPtr msg = view_imu_iterator->instantiate<sensor_msgs::Imu>();
  //       view_imu_iterator++;
  //     } while (view_imu_iterator != view_imu.end());

  //     view_cam_iterators[i]++;
  //   }
  // }

  return 0;
}
