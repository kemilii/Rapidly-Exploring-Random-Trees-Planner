#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class MapServer {
private:
    ros::NodeHandle nh;
    ros::Publisher map_pub;

public:
    MapServer() {
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
        std::string map_file_path;
        nh.param<std::string>("map_file", map_file_path, "/root/catkin_ws/src/rrt_planner_ros/resources/map1.png");
        publishMap(map_file_path);
    }

    // Read image and publish it as an OccupancyGrid.
    void publishMap(const std::string& file_path) {
        cv::Mat image = cv::imread(file_path, cv::IMREAD_GRAYSCALE);
        if (image.empty()) {
            ROS_ERROR("Failed to load map image: %s", file_path.c_str());
            return;
        }
        nav_msgs::OccupancyGrid map_msg;
        map_msg.header.stamp = ros::Time::now();
        map_msg.header.frame_id = "map";
        map_msg.info.width = image.cols;
        map_msg.info.height = image.rows;
        map_msg.info.resolution = 0.1;
        map_msg.info.origin.position.x = 0;
        map_msg.info.origin.position.y = 0;
        map_msg.info.origin.orientation.w = 1.0;
        map_msg.data.resize(image.cols * image.rows);

        for (int y = 0; y < image.rows; y++) {
            for (int x = 0; x < image.cols; x++) {
                int index = (image.rows - 1 - y) * image.cols + x;
                map_msg.data[index] = (image.at<uchar>(y, x) > 128) ? 0 : 100;
            }
        }
        map_pub.publish(map_msg);
        ROS_INFO("Map published from %s", file_path.c_str());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_server_node");
    MapServer map_server;
    ros::spin();
    ROS_INFO("Map Server Node started!");
    return 0;
}
