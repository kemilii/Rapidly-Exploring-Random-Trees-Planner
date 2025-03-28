#include "rrt_planner_ros/rrt_planner.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <random>
#include <limits>
#include <cmath>
#include <algorithm>

namespace rrt_planner {

    RRTPlanner::RRTPlanner(ros::NodeHandle* node)
            : nh_(node),
              private_nh_("~"),
              map_received_(false),
              init_pose_received_(false),
              goal_received_(false)
    {
        // Load parameters
        private_nh_.param("enable_visualization", enable_visualization_, true);
        private_nh_.param("max_iterations", max_iterations_, 1000);
        private_nh_.param("step_size", step_size_, 5.0);
        private_nh_.param("goal_threshold", goal_threshold_, 5.0);

        std::string map_topic, init_pose_topic, goal_topic, path_topic;
        private_nh_.param("map_topic", map_topic, std::string("/map"));
        private_nh_.param("init_pose_topic", init_pose_topic, std::string("/initialpose"));
        private_nh_.param("goal_topic", goal_topic, std::string("/move_base_simple/goal"));
        private_nh_.param("path_topic", path_topic, std::string("/rrt_path"));

        map_sub_ = nh_->subscribe(map_topic, 1, &RRTPlanner::mapCallback, this);
        init_pose_sub_ = nh_->subscribe(init_pose_topic, 1, &RRTPlanner::initPoseCallback, this);
        goal_sub_ = nh_->subscribe(goal_topic, 1, &RRTPlanner::goalCallback, this);
        path_pub_ = nh_->advertise<nav_msgs::Path>(path_topic, 1);

        ROS_INFO("RRTPlanner initialized. Waiting for map and poses...");
    }

    void RRTPlanner::mapCallback(const nav_msgs::OccupancyGrid::Ptr& msg)
    {
        map_grid_ = msg;
        map_received_ = true;
        buildMapImage();
        ROS_INFO("Map received.");
    }

    void RRTPlanner::initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        init_pose_ = *msg;
        init_pose_received_ = true;
        poseToPoint(init_point_, init_pose_.pose.pose);
        ROS_INFO("Initial pose: (%.2f, %.2f)", init_point_.x(), init_point_.y());
    }

    void RRTPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        goal_ = *msg;
        goal_received_ = true;
        poseToPoint(goal_point_, goal_.pose);
        ROS_INFO("Goal: (%.2f, %.2f)", goal_point_.x(), goal_point_.y());
    }

    void RRTPlanner::drawGoalInitPose()
    {
        // Show initial pose (green) and goal (red)
        if(!map_ || !init_pose_received_ || !goal_received_) return;
        drawCircle(init_point_, 3, cv::Scalar(0, 255, 0));
        drawCircle(goal_point_, 3, cv::Scalar(0, 0, 255));
        displayMapImage(1);
    }

    void RRTPlanner::plan()
    {
        // Main RRT loop
        if(!map_received_ || !init_pose_received_ || !goal_received_)
        {
            ROS_WARN("Cannot plan without map, init pose, and goal.");
            return;
        }

        std::vector<Node> tree;
        Node start_node{ init_point_, -1 };
        tree.push_back(start_node);

        std::random_device rd;
        std::mt19937 gen(rd());
        double min_x = 0, max_x = map_grid_->info.height;
        double min_y = 0, max_y = map_grid_->info.width;
        std::uniform_real_distribution<> dis_x(min_x, max_x);
        std::uniform_real_distribution<> dis_y(min_y, max_y);

        int goal_index = -1;
        for (int i = 0; i < max_iterations_; i++)
        {
            Point2D random_point(dis_x(gen), dis_y(gen));
            int nearest_index = -1;
            double min_dist = std::numeric_limits<double>::max();

            for (size_t j = 0; j < tree.size(); j++)
            {
                double dx = tree[j].point.x() - random_point.x();
                double dy = tree[j].point.y() - random_point.y();
                double dist = std::sqrt(dx * dx + dy * dy);
                if(dist < min_dist)
                {
                    min_dist = dist;
                    nearest_index = j;
                }
            }

            double theta = std::atan2(random_point.y() - tree[nearest_index].point.y(),
                                      random_point.x() - tree[nearest_index].point.x());
            Point2D new_point(
                    tree[nearest_index].point.x() + step_size_ * std::cos(theta),
                    tree[nearest_index].point.y() + step_size_ * std::sin(theta)
            );

            if(!isPointUnoccupied(new_point)) continue;

            Node new_node{ new_point, nearest_index };
            tree.push_back(new_node);

            double dx = new_point.x() - goal_point_.x();
            double dy = new_point.y() - goal_point_.y();
            if(std::sqrt(dx * dx + dy * dy) < goal_threshold_)
            {
                Node final_node{ goal_point_, static_cast<int>(tree.size()) - 1 };
                tree.push_back(final_node);
                goal_index = tree.size() - 1;
                ROS_INFO("Goal reached after %d iterations", i);
                break;
            }
        }

        if(goal_index == -1)
        {
            ROS_WARN("Failed to reach goal in max iterations.");
            return;
        }

        // Reconstruct path
        std::vector<Point2D> path;
        int current_index = goal_index;
        while(current_index != -1)
        {
            path.push_back(tree[current_index].point);
            current_index = tree[current_index].parent;
        }
        std::reverse(path.begin(), path.end());

        nav_msgs::Path path_msg;
        path_msg.header.frame_id = map_grid_->header.frame_id;
        path_msg.header.stamp = ros::Time::now();
        for(const auto& p : path)
            path_msg.poses.push_back(pointToPose(p));
        path_pub_.publish(path_msg);

        ROS_INFO("Published path with %lu poses.", path_msg.poses.size());

        // Optional visualization
        for(size_t i = 1; i < path.size(); i++)
            drawLine(path[i-1], path[i], cv::Scalar(255, 0, 0), 1);
        displayMapImage(1);
    }

    void RRTPlanner::publishPath()
    {
        // Reserved for republishing if needed
    }

    bool RRTPlanner::isPointUnoccupied(const Point2D& p)
    {
        // Check if point is within bounds and not in an obstacle
        if(!map_grid_) return false;
        int x = static_cast<int>(p.x());
        int y = static_cast<int>(p.y());
        if(x < 0 || x >= static_cast<int>(map_grid_->info.height) ||
           y < 0 || y >= static_cast<int>(map_grid_->info.width))
            return false;
        int index = x * map_grid_->info.width + y;
        return (map_grid_->data[index] == 0);
    }

    void RRTPlanner::buildMapImage()
    {
        // Convert OccupancyGrid to OpenCV image
        if(!map_grid_) return;
        int height = map_grid_->info.height;
        int width = map_grid_->info.width;
        cv::Mat img(height, width, CV_8UC1);
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                int index = i * width + j;
                img.at<uchar>(i, j) = (map_grid_->data[index] == 0) ? 255 : 0;
            }
        }
        map_.reset(new cv::Mat(img));
    }

    int RRTPlanner::toIndex(int x, int y)
    {
        return x * map_grid_->info.width + y;
    }

    void RRTPlanner::poseToPoint(Point2D& point, const geometry_msgs::Pose& pose)
    {
        point.x(pose.position.x / map_grid_->info.resolution);
        point.y(pose.position.y / map_grid_->info.resolution);
    }

    geometry_msgs::PoseStamped RRTPlanner::pointToPose(const Point2D& point)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = point.x() * map_grid_->info.resolution;
        pose.pose.position.y = point.y() * map_grid_->info.resolution;
        pose.pose.orientation.w = 1.0;
        return pose;
    }

    void RRTPlanner::drawCircle(const Point2D& point, int radius, const cv::Scalar& color)
    {
        cv::circle(*map_, cv::Point(point.y(), point.x()), radius, color, -1);
    }

    void RRTPlanner::drawLine(const Point2D& start, const Point2D& end, const cv::Scalar& color, int thickness)
    {
        cv::line(*map_, cv::Point(start.y(), start.x()), cv::Point(end.y(), end.x()), color, thickness);
    }

    void RRTPlanner::displayMapImage(int wait_time)
    {
        cv::imshow("RRT Planner Map", *map_);
        cv::waitKey(wait_time);
    }

} // namespace rrt_planner
