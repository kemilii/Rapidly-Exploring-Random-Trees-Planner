class RRTPlanner {
private:
    bool enable_visualization_;

public:
    // Constructor: load visualization flag.
    RRTPlanner(ros::NodeHandle* node) {
        private_nh_.param("enable_visualization", enable_visualization_, true);
    }

    // Plan: display map if visualization is enabled.
    void plan() {
        if (enable_visualization_) {
            displayMapImage(1);
        }
    }

    // Return visualization status.
    bool isVisualizationEnabled() const {
        return enable_visualization_;
    }
};