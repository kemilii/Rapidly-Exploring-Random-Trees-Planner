# ROS RRT Path Planner

## Overview
This ROS package implements a Rapidly-Exploring Random Tree (RRT) path planning algorithm for 2D navigation. The planner can generate paths through complex environments using a probabilistic tree-based approach.

## Prerequisites
- ROS Melodic or Noetic
- OpenCV
- catkin build system

## Dependencies
- nav_msgs
- geometry_msgs
- OpenCV
- roscpp

## Installation
1. Clone the repository into your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone <your-repository-url>
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### Running the Nodes
1. Launch the RRT Planner:
```bash
roslaunch rrt_planner_ros rrt_planner.launch
```

### Specifying Map
- Place your map image in the `resources/` directory
- Modify the launch file to use your map:
```xml
<node name="map_server" pkg="map_server" type="map_server" 
      args="$(find rrt_planner_ros)/resources/your_map.png"/>
```

### Setting Start and Goal Positions
You can set start and goal positions using RViz or via ROS topics:

#### Using RViz
1. Open RViz
2. Use "2D Pose Estimate" to set initial pose
3. Use "2D Nav Goal" to set goal position

#### Using Command Line
- Initial Pose:
```bash
rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped '{pose:{pose:{position:{x: 10, y: 20}}}}'
```

- Goal Position:
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{pose:{position:{x: 50, y: 60}}}'
```

### Visualization Options
Control visualization through launch file or ROS parameters:
```xml
<param name="enable_visualization" value="true"/>
```

## Configuration Parameters
Edit `cfg/config.yaml` to adjust:
- `max_iterations`: Maximum RRT tree expansion iterations
- `step_size`: Distance between tree nodes
- `goal_threshold`: Proximity to goal for path completion
- `enable_visualization`: Toggle real-time visualization

## Map Conventions
- White pixels represent free space
- Black pixels represent obstacles

## Troubleshooting
- Ensure map is correctly loaded
- Check that start and goal positions are within map boundaries
- Verify ROS master is running

## Performance Tips
- Increase `max_iterations` for complex environments
- Adjust `step_size` based on map resolution

## Contributing
1. Fork the repository
2. Create your feature branch
3. Commit changes
4. Push to the branch
5. Create a Pull Request

## Acknowledgments
- Steven M. Lavalle (RRT Algorithm Original Work)
- Polybee Robotics for the challenge