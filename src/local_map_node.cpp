//
// Created by kevin on 3/21/18.
//

#include <ros/ros.h>

#include "explore_large_map/map_builder.hpp"

using namespace explore_global_map;

nav_msgs::Odometry global_vehicle_pose;
iv_slam_ros_msgs::TraversibleArea traversible_map;
nav_msgs::OccupancyGrid local_map;
double first_x, first_y;

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

void project_to_vehicle(nav_msgs::Odometry &vehicle_pos, iv_slam_ros_msgs::TraversibleArea &map) {
    static int counter = 0;
    // get abosolute x,y
    explore_global_map::geographic_to_grid(vehicle_pos.pose.pose.position.x, vehicle_pos.pose.pose.position.y);
    explore_global_map::geographic_to_grid(map.triD_submap_pose.position.x, map.triD_submap_pose.position.y);

    // get diff, make calc small
    if(counter == 0) {
        first_x = vehicle_pos.pose.pose.position.x;
        first_y = vehicle_pos.pose.pose.position.y;

    }
    vehicle_pos.pose.pose.position.x -= first_x;
    vehicle_pos.pose.pose.position.y -= first_y;
    map.triD_submap_pose.position.x -= first_x;
    map.triD_submap_pose.position.y -= first_y;

    // reverse yaw and roll sequence
    geometry_msgs::Quaternion msg;
    msg = explore_global_map::reverse_yaw_roll(map.triD_submap_pose.orientation);
    map.triD_submap_pose.orientation = msg;
    msg = explore_global_map::reverse_yaw_roll(vehicle_pos.pose.pose.orientation);
    vehicle_pos.pose.pose.orientation = msg;

    // fill  map cell value into vehicle local map
    geometry_msgs::Point pt_local_submap;
    geometry_msgs::Pose ps_local_submap;
    geometry_msgs::Point pt_global_submap;
    geometry_msgs::Pose ps_global_submap;
    geometry_msgs::Pose ps_global_odom;
    int global_map_x, global_map_y;
    int ref_in_odom_x, ref_in_odom_y;
    tf::Transform all_trans;
    all_trans = worldToMap(vehicle_pos.pose.pose) * mapToWorld(map.triD_submap_pose);
    for(int i = 0; i < map.height; i++) {
        for(int j = 0; j < map.width; j++) {
            int index_in_tailored_map = i * map.width + j;
            // -->local_submap
            pt_local_submap.x = (j - map.triD_submap_pose_image_index_x) * map.resolution;
            pt_local_submap.y = (i - map.triD_submap_pose_image_index_y) * map.resolution;
            pt_local_submap.z = 0;  // todo
            ps_local_submap.position = pt_local_submap;
            ps_local_submap.orientation = map.triD_submap_pose.orientation;

            // -->global_submap
            {
                tf::Pose ps;
                tf::poseMsgToTF(ps_local_submap, ps);
                tf::poseTFToMsg(all_trans * ps, ps_global_odom);
            }

            global_map_x = floor((ps_global_odom.position.x  - local_map.info.origin.position.x) / local_map.info.resolution);
            global_map_y = floor((ps_global_odom.position.y  - local_map.info.origin.position.y) / local_map.info.resolution);
            int index_in_global_map = global_map_y * local_map.info.width + global_map_x;
            if(global_map_x > 0 && global_map_x < local_map.info.width &&
               global_map_y > 0 && global_map_y < local_map.info.height) {
                if(i == map.triD_submap_pose_image_index_y
                   && j == map.triD_submap_pose_image_index_x)
                {
                    ref_in_odom_x = global_map_x;
                    ref_in_odom_y = global_map_y;
                }
                if(map.cells[index_in_tailored_map] == 1) {
                    local_map.data[index_in_global_map] = 0;
                } else if(map.cells[index_in_tailored_map] == 2) {
                    local_map.data[index_in_global_map] = 100;
                } else {
                    local_map.data[index_in_global_map] = -1;
                }
            }
        }
    }
    counter++;

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "local_map_node");
    ros::NodeHandle nh("~");
    ros::Subscriber publisher = nh.subscribe("/traversible_area_topic", 1, traversibleMapCallback);
    ros::Subscriber vehicle_global_pose_sub = nh.subscribe("/vehicle_global_pose_topic",1,vehiclePoseCallback);
    ros::Publisher map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/local_map", 1, true);

    double map_width;
    double map_height;
    double map_resolution;
    nh.param<double>("map_width", map_width, 400);
    nh.param<double>("map_height", map_height, 800);
    nh.param<double>("map_resolution", map_resolution, 0.1);

    local_map.header.frame_id = "base_link";
    local_map.info.width = map_width;
    local_map.info.height = map_height;
    local_map.info.resolution = map_resolution;
    local_map.info.origin.position.x = -static_cast<double>(map_width) / 2 * map_resolution;
    local_map.info.origin.position.y = -static_cast<double>(map_height) / 2 * map_resolution;
    local_map.info.origin.orientation.w = 1.0;
    local_map.data.assign(map_width * map_height, -1);  // Fill with "unknown" occupancy.


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
        project_to_vehicle(global_vehicle_pose, traversible_map);
        local_map.header.stamp = ros::Time::now();
        map_publisher.publish(local_map);
        auto end = std::chrono::system_clock::now();
        auto msec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
        std::cout << "local map build cost time msec :" << msec << "\n";


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}