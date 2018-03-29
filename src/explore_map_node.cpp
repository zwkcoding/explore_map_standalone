#include <ros/ros.h>

#include "explore_large_map/map_builder.hpp"


nav_msgs::Odometry global_vehicle_pose;
nav_msgs::OccupancyGrid local_map;
iv_slam_ros_msgs::TraversibleArea traversible_map;

bool receive_vehicle_pose = false, receive_traversible_map = false;
void vehiclePoseCallback(const nav_msgs::Odometry &sub_tem_global_vehicle_pose){
    global_vehicle_pose.header.stamp = sub_tem_global_vehicle_pose.header.stamp;
    global_vehicle_pose.header.frame_id = sub_tem_global_vehicle_pose.header.frame_id;
    global_vehicle_pose.pose.pose.position.x = sub_tem_global_vehicle_pose.pose.pose.position.x;//longitude of the current UGV positon
    global_vehicle_pose.pose.pose.position.y = sub_tem_global_vehicle_pose.pose.pose.position.y;//latitude of the current UGV positon
    global_vehicle_pose.pose.pose.position.z = sub_tem_global_vehicle_pose.pose.pose.position.z;// the following result is pose relative to the start point of UGV
    global_vehicle_pose.pose.pose.orientation.w = sub_tem_global_vehicle_pose.pose.pose.orientation.w;
    global_vehicle_pose.pose.pose.orientation.x = sub_tem_global_vehicle_pose.pose.pose.orientation.x;
    global_vehicle_pose.pose.pose.orientation.y = sub_tem_global_vehicle_pose.pose.pose.orientation.y;
    global_vehicle_pose.pose.pose.orientation.z = sub_tem_global_vehicle_pose.pose.pose.orientation.z;

    receive_vehicle_pose = true;
}

void localMapCall( const nav_msgs::OccupancyGridConstPtr &map) {
    local_map = *map;

    receive_traversible_map = true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "explore_large_map_node");
    ros::NodeHandle nh("~");

    ros::Subscriber local_map_sub = nh.subscribe("/explore_entry_map", 1, localMapCall);
    ros::Subscriber vehicle_global_pose_sub = nh.subscribe("/vehicle_global_pose_topic",1,vehiclePoseCallback);
    ros::Publisher map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/global_map", 1, true);
    ros::Publisher current_position_in_explore_map_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1, false);
    double map_width;
    double map_height;
    double map_resolution;

    nh.param<double>("map_width", map_width, 100);
    nh.param<double>("map_height", map_height, 100);
    nh.param<double>("map_resolution", map_resolution, 0.1);

    explore_global_map::MapBuilder map_builder(map_width, map_height, map_resolution);

    ros::Rate rate(10.0);
    while (nh.ok()) {
        // wait for msg
        while(!receive_vehicle_pose || !receive_traversible_map) {
            ros::spinOnce();
            rate.sleep();
        }
        // reset flag
        receive_vehicle_pose = receive_traversible_map = false;
        // build map
        auto start = std::chrono::system_clock::now();
        map_builder.grow(global_vehicle_pose, local_map);
        auto end = std::chrono::system_clock::now();
        auto msec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
        ROS_INFO_STREAM_THROTTLE(0.5,"global explore map build cost time [msec] :" << msec);
        map_publisher.publish(map_builder.getMap());
        nav_msgs::Odometry odom_global_vehicle_pose;
        odom_global_vehicle_pose = global_vehicle_pose;
        odom_global_vehicle_pose.header.frame_id = "/odom";
        odom_global_vehicle_pose.header.stamp = ros::Time::now();
        current_position_in_explore_map_pub.publish(odom_global_vehicle_pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
