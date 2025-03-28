cmake_minimum_required(VERSION 3.0.2)
project(rrt_planner_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        nav_msgs
        geometry_msgs
        cv_bridge
        image_transport
        )

find_package(OpenCV REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES rrt_planner
        CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs cv_bridge image_transport
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_library(rrt_planner
        src/rrt_planner.cpp
        )

add_executable(rrt_planner_node
        src/rrt_planner_node.cpp
        )

target_link_libraries(rrt_planner
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

target_link_libraries(rrt_planner_node
        rrt_planner
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

add_executable(map_server_node
        src/map_server_node.cpp
        src/map_server_node.cpp)

target_link_libraries(map_server_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )