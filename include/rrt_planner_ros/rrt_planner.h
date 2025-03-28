#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>
#include <vector>
#include <random>

namespace rrt_planner {

    class Point2D {
    public:
        Point2D() : x_(0), y_(0) {}
        Point2D(double x, double y) : x_(x), y_(y) {}
        double x() const { return x_; }
        double y() const { return y_; }
        void x(double x) { x_ = x; }
        void y(double y) { y_ = y; }
    private:
        double x_, y_;
    };

    struct Node {
        Point2D point;
        int parent;
    };

    class RRTPlanner {
    public:
        RRTPlanner(ros::NodeHandle* node);

        void mapCallback(const nav_msgs::OccupancyGrid::Ptr& msg);
        void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void drawGoalInitPose();
        void plan();
        void publishPath();

    private:
        int toIndex(int x, int y);
        void poseToPoint(Point2D& point, const geometry_msgs::Pose& pose);
        geometry_msgs::PoseStamped pointToPose(const Point2D& point);
        void drawCircle(const Point2D& point, int radius, const cv::Scalar& color);
        void drawLine(const Point2D& start, const Point2D& end, const cv::Scalar& color, int thickness);
        void displayMapImage(int wait_time);
        bool isPointUnoccupied(const Point2D& p);
        void buildMapImage();

        ros::NodeHandle* nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber map_sub_, init_pose_sub_, goal_sub_;
        ros::Publisher path_pub_;

        nav_msgs::OccupancyGrid::Ptr map_grid_;
        std::unique_ptr<cv::Mat> map_;

        geometry_msgs::PoseWithCovarianceStamped init_pose_;
        geometry_msgs::PoseStamped goal_;
        Point2D init_point_, goal_point_;

        bool map_received_;
        bool init_pose_received_;
        bool goal_received_;
        bool enable_visualization_;

        int max_iterations_;
        double step_size_;
        double goal_threshold_;
    };

} // namespace rrt_planner

#endif // RRT_PLANNER_H