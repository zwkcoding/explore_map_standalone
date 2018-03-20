#include <ros/ros.h>

#include "explore_large_map/map_builder.hpp"


nav_msgs::Odometry global_vehicle_pose;
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

void traversibleMapCallback(const iv_slam_ros_msgs::TraversibleArea &subed_msg){

    traversible_map.header.stamp = subed_msg.header.stamp;
    traversible_map.header.frame_id = subed_msg.header.frame_id;//which is "traversible_area_frame"
    traversible_map.resolution = subed_msg.resolution; //resolution of the submap
    traversible_map.submap_finished_flag = subed_msg.submap_finished_flag;//flag to decide whether the current submap is finished;
    traversible_map.triD_submap_pose_image_index_x = subed_msg.triD_submap_pose_image_index_x; // location in the pictrue of the submap pose.
    traversible_map.triD_submap_pose_image_index_y = subed_msg.triD_submap_pose_image_index_y;

    traversible_map.triD_submap_pose.position.x = subed_msg.triD_submap_pose.position.x;//longitude of the submap
    traversible_map.triD_submap_pose.position.y = subed_msg.triD_submap_pose.position.y;//latitude of the submap
    traversible_map.triD_submap_pose.position.z = subed_msg.triD_submap_pose.position.z;// position relative to the start point of UGV
    traversible_map.triD_submap_pose.orientation.w = subed_msg.triD_submap_pose.orientation.w;//the following is the pose in the world frame
    traversible_map.triD_submap_pose.orientation.x = subed_msg.triD_submap_pose.orientation.x;
    traversible_map.triD_submap_pose.orientation.y = subed_msg.triD_submap_pose.orientation.y;
    traversible_map.triD_submap_pose.orientation.z = subed_msg.triD_submap_pose.orientation.z;

    traversible_map.width = subed_msg.width;
    traversible_map.height = subed_msg.height;
    traversible_map.cells.assign(subed_msg.cells.begin(),subed_msg.cells.end());

    receive_traversible_map = true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "explore_large_map_node");
    ros::NodeHandle nh("~");
    ros::Subscriber publisher = nh.subscribe("/traversible_area_topic", 1, traversibleMapCallback);
    ros::Subscriber vehicle_global_pose_sub = nh.subscribe("/vehicle_global_pose_topic",1,vehiclePoseCallback);
    ros::Publisher map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/global_map", 1, true);

    double map_width;
    double map_height;
    double map_resolution;
    nh.param<double>("map_width", map_width, 1000);
    nh.param<double>("map_height", map_height, 1000);
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
        map_builder.grow(global_vehicle_pose, traversible_map);
        auto end = std::chrono::system_clock::now();
        auto msec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
        std::cout << "global explore map build cost time msec :" << msec << "\n";

        map_publisher.publish(map_builder.getMap());


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
