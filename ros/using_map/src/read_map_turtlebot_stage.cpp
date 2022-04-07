#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class UsingMap {
   public:
    UsingMap() {
        map_sub_.subscribe(nh_, "map", 10);
        odom_sub_.subscribe(nh_, "odom", 10);
        path_sub_.subscribe(nh_, "/move_base/NavfnROS/plan", 10);

        pub_path_ = nh_.advertise<nav_msgs::Path>("path", 1);

        approx_synchronizer_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(10), map_sub_, odom_sub_, path_sub_);
        approx_synchronizer_->registerCallback(boost::bind(&UsingMap::approx_sync_cb, this, _1, _2, _3));

        ROS_INFO("started readmap node!");
    }

   private:
    void lookup_tf(std::string str_from, std::string str_to, tf::StampedTransform &tf);
    void lookup_tf2(std::string str_from, std::string str_to, geometry_msgs::TransformStamped &tf2);
    void print_tf(const tf::Transform &tf, std::string str_msg);
    void navodom2tf(const nav_msgs::Odometry::ConstPtr &odom, tf::Transform &tf);

    void approx_sync_cb(
        const nav_msgs::OccupancyGrid::ConstPtr &map_msg,
        const nav_msgs::Odometry::ConstPtr &odom_msg,
        const nav_msgs::Path::ConstPtr &path_msg);

   private:
    typedef message_filters::sync_policies::ApproximateTime<
        nav_msgs::OccupancyGrid, nav_msgs::Odometry, nav_msgs::Path>
        MyApproxSyncPolicy;
    message_filters::Synchronizer<MyApproxSyncPolicy> *approx_synchronizer_;

    message_filters::Subscriber<nav_msgs::OccupancyGrid> map_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<nav_msgs::Path> path_sub_;

    ros::NodeHandle nh_;
    ros::Publisher pub_path_;

    tf::TransformListener tf_listener_;

    boost::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
};

void UsingMap::approx_sync_cb(
    const nav_msgs::OccupancyGrid::ConstPtr &map_msg,
    const nav_msgs::Odometry::ConstPtr &odom_msg,
    const nav_msgs::Path::ConstPtr &path_msg) {
    // get tf for map --> odom
    tf::StampedTransform tf_odom_map;
    lookup_tf("map", "odom", tf_odom_map);
    print_tf(tf_odom_map, "map --> odom");

    // get tf for odom --> local_map
    tf::Transform tf_lcmap_odom;
    navodom2tf(odom_msg, tf_lcmap_odom);
    tf_lcmap_odom.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_lcmap_odom = tf_lcmap_odom.inverse();
    print_tf(tf_lcmap_odom, "odom --> local_map");

    // get nav path and transform it from map to local map
    std::vector<geometry_msgs::PoseStamped> vec_poses;
    vec_poses = path_msg->poses;
    if (vec_poses.empty()) {
        ROS_ERROR("vec_poses is empty");
        return;
    }
    ROS_INFO("vec_poses size: %d", vec_poses.size());

    std::vector<cv::Point2d> vec_pt2d;
    vec_pt2d.resize(vec_poses.size());
    for (int i = 0; i < vec_poses.size(); ++i) {
        geometry_msgs::PoseStamped pose = vec_poses[i];

        tf::Vector3 v3_posiotion;
        v3_posiotion.setX(pose.pose.position.x);
        v3_posiotion.setY(pose.pose.position.y);
        v3_posiotion.setZ(pose.pose.position.z);

        tf::Vector3 v3_dst = tf_lcmap_odom * tf_odom_map * v3_posiotion;

        cv::Point2d pt2d;
        pt2d.x = v3_dst.getX();
        pt2d.y = v3_dst.getY();

        vec_pt2d[i] = pt2d;
    }

    // get and process local map
    ROS_INFO("map size: (%d, %d), resolution: %f",
             map_msg->info.width, map_msg->info.height, map_msg->info.resolution);

    int map_w = map_msg->info.width;
    int map_h = map_msg->info.height;

    float map_res_inv = 1.f / map_msg->info.resolution;

    cv::Mat mat_map(map_h, map_w, CV_8UC1);

    for (int h = 0; h < map_h; ++h) {
        for (int w = 0; w < map_w; ++w) {
            int value = map_msg->data[h * map_w + w];
            if (value == -1) value = 128;
            mat_map.at<unsigned char>(h, w) = value;
        }
    }

    int threshold = 50;
    cv::Mat mat_edge;
    cv::Canny(mat_map, mat_edge, threshold, threshold * 3, 3);

    for (int i = 0; i < vec_pt2d.size(); ++i) {
        cv::Point2d pt2d = vec_pt2d[i];
        int x = int(pt2d.x * map_res_inv + map_w * 0.5);
        int y = int(pt2d.y * map_res_inv + map_h * 0.5);
        if (x >= 0 && x < map_w && y >= 0 && y < map_h)
            mat_edge.at<unsigned char>(y, x) = 128;
    }

    // publish nav path
    nav_msgs::Path path_msg_out;

    path_msg_out.header.frame_id = "base_laser_link";
    path_msg_out.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.orientation.x = 0.0;
    pose_stamped.pose.orientation.y = 0.0;
    pose_stamped.pose.orientation.z = 0.0;
    pose_stamped.pose.orientation.w = 1.0;

    pose_stamped.pose.position.x = 0;
    pose_stamped.pose.position.y = 0;

    path_msg_out.poses.emplace_back(pose_stamped);

    pub_path_.publish(path_msg_out);

    for (int h = 0; h < map_h; h += 3) {
        for (int w = 0; w < map_w; w += 3)
            std::cout << std::setw(4) << std::left << (int)mat_map.at<unsigned char>(h, w);
        std::cout << std::endl;
    }
    std::cout << std::endl;

    cv::imshow("", mat_edge);
    cv::waitKey(30);
}

void UsingMap::lookup_tf(std::string str_from, std::string str_to, tf::StampedTransform &tf) {
    std::string msg_error = "";
    ros::Time timestamp = ros::Time(0);
    ;

    if (tf_listener_.canTransform(str_to, str_from, timestamp, &msg_error)) {
        tf_listener_.lookupTransform(str_to, str_from, timestamp, tf);
    } else {
        ROS_WARN_THROTTLE(10.0,
                          "The tf from '%s' to '%s' does not seem to be available, will assume it as identity!",
                          str_from.c_str(), str_to.c_str());
        ROS_ERROR("tf error: %s", msg_error.c_str());
        tf.setIdentity();
    }
}

void UsingMap::lookup_tf2(std::string str_from, std::string str_to, geometry_msgs::TransformStamped &tf2) {
    try {
        if (tf2_buffer_->canTransform(str_to, str_from, ros::Time(0)))
            tf2 = tf2_buffer_->lookupTransform(str_to, str_from, ros::Time(0));
        else
            ROS_ERROR("tf2 error: cannot transform from %s to %s", str_from.c_str(), str_to.c_str());
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("tf2 TransformException: %s", ex.what());
        ros::Duration(1.0).sleep();
    }

    geometry_msgs::Quaternion gm_q = tf2.transform.rotation;
    geometry_msgs::Vector3 gm_v3 = tf2.transform.translation;

    Eigen::Quaternion<double> q_rotation(gm_q.w, gm_q.x, gm_q.y, gm_q.z);
    Eigen::Vector3d v3_translation(gm_v3.x, gm_v3.y, gm_v3.z);

    Eigen::Matrix4d m4_tf = Eigen::Matrix4d::Identity();
    m4_tf.block<3, 3>(0, 0) = q_rotation.matrix();
    m4_tf.block<3, 1>(0, 3) = v3_translation;

    // tf::Quaternion tf_q = transform_stamped.getRotation();
    // tf::Vector3 tf_v3   = transform_stamped.getOrigin();
    // tf::vectorTFToEigen(tf_v3, v3_translation);
    // tf::quaternionTFToEigen(tf_q, q_rotation);
}

void UsingMap::print_tf(const tf::Transform &tf, std::string str_msg) {
    double rr, rp, ry;
    tf.getBasis().getRPY(rr, rp, ry);
    rr = rr * M_PI_2 * 180.0;
    rp = rp * M_PI_2 * 180.0;
    ry = ry * M_PI_2 * 180.0;

    double px = tf.getOrigin().getX();
    double py = tf.getOrigin().getY();
    double pz = tf.getOrigin().getZ();

    double qx = tf.getRotation().getX();
    double qy = tf.getRotation().getY();
    double qz = tf.getRotation().getZ();
    double qw = tf.getRotation().getW();

    ROS_INFO(
        "\n%s tf :\n"
        "  Translation: [%f, %f, %f]\n"
        "  Rotation:\n"
        "    in Quaternion: [%f, %f, %f, %f]\n"
        "    in RPY(degree): [%f, %f, %f]\n",
        str_msg.c_str(), px, py, pz, qx, qy, qz, qw, rr, rp, ry);
}

void UsingMap::navodom2tf(const nav_msgs::Odometry::ConstPtr &odom, tf::Transform &tf) {
    tf::Vector3 v3_posiotion;
    v3_posiotion.setX(odom->pose.pose.position.x);
    v3_posiotion.setY(odom->pose.pose.position.y);
    v3_posiotion.setZ(odom->pose.pose.position.z);

    tf::Quaternion q_orientation;
    q_orientation.setX(odom->pose.pose.orientation.x);
    q_orientation.setY(odom->pose.pose.orientation.y);
    q_orientation.setZ(odom->pose.pose.orientation.z);
    q_orientation.setW(odom->pose.pose.orientation.w);

    tf.setIdentity();
    tf.setOrigin(v3_posiotion);
    tf.setRotation(q_orientation);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "read_map");

    UsingMap usingmap;

    ros::Rate r(30);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}
