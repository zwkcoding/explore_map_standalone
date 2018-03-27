//
// Created by kevin on 3/21/18.
//

#include <ros/ros.h>

#include "explore_large_map/map_builder.hpp"
#include <map_ray_caster/map_ray_caster.h>
#define OPENCV_SHOW
using namespace explore_global_map;

nav_msgs::Odometry global_vehicle_pose;
iv_slam_ros_msgs::TraversibleArea traversible_map;
nav_msgs::OccupancyGrid local_map;
nav_msgs::OccupancyGrid cast_local_map;

double first_x, first_y;

map_ray_caster::MapRayCaster ray_caster;  //!< Ray casting with cache.

// vehicle position in local_map [m]
double vehicle_x_in_map;
double vehicle_y_in_map;

double far_distance;

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

void projectToVehicle(nav_msgs::Odometry &vehicle_pos, iv_slam_ros_msgs::TraversibleArea &map) {
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
                    // notice : deal method is different for explore_map
                    local_map.data[index_in_global_map] = 100;
                }
            }
        }
    }
    counter++;
    if(counter > 10000) {
        counter = 1; // not 0
    }


}


// clear 50m far front of vehicle, make it free for path plan
void clearFarRegion() {
    int width = local_map.info.width;
    int height = local_map.info.height;
    int clear_area_height_start = (vehicle_y_in_map + far_distance) / local_map.info.resolution;
    for(int i = 0; i < width; i++) {
        for(int j = clear_area_height_start; j < height; j++) {
            int index =  j * width + i;
            local_map.data[index] = 0;  // clear
        }
    }
}

void rayCasting() {


#ifdef OPENCV_SHOW
    int occ_map_width = local_map.info.width;
    int occ_map_height = local_map.info.height;
    cv::Mat occ_mat_src(occ_map_height, occ_map_width, CV_8UC1, cv::Scalar(127));
    for (int i = 0; i < occ_map_height; i++) {
        for (int j = 0; j < occ_map_width; j++) {
            int index = i * occ_map_width + j;
            if (local_map.data[index] == 0) {
                occ_mat_src.at<uchar>(i, j) = 255;
            } else if (local_map.data[index] == 100) {
                occ_mat_src.at<uchar>(i, j) = 0;
            } else {
                occ_mat_src.at<uchar>(i, j) = 0;
            }
        }
    }

    cv::Mat occ_mat_bgr;
    cv::cvtColor(occ_mat_src, occ_mat_bgr, CV_GRAY2BGR);
    cv::Point pt_e1,pt_e2;
    pt_e1.x = 250;
    pt_e1.y = 250;


#endif

    cast_local_map.data.assign(occ_map_width * occ_map_height, -1);  // Fill with "unknown" occupancy.

    static double angle_start = -M_PI;
    static double angle_end = angle_start + 2 * M_PI - 1e-6;
    int show_height = local_map.info.height - 500;
    for (double a = angle_start; a <= angle_end; a += M_PI / 180) {
        std::vector<size_t> ray_to_map_border;
        ray_to_map_border = ray_caster.getRayCastToMapBorder(a, 1250, 1250, 1.1 * M_PI / 180);
        for(auto &value: ray_to_map_border) {
            int row_shift = value / 1250 - 1250 / 2;
            int column_shift = value % 1250 - 1250 / 2;
            int current_column = 250 + column_shift;  // x
            int current_row = 250 + row_shift; // y
            // only reserve point in map
            if(current_column > 0 && current_column < local_map.info.width &&
               current_row > 0 && current_row < show_height) {
                int index = current_row * local_map.info.width + current_column;
                if(100 == local_map.data[index]) {
                    pt_e2.y = current_row;
                    pt_e2.x = current_column;
                    cv::line(occ_mat_bgr, pt_e1, pt_e2, cv::Scalar(255, 0, 0), 1, 8);

                    cast_local_map.data[index] = 100;
                    break;
                } else {
                    cast_local_map.data[index] = 0;
                }
            } else {
                if(current_column < 0) current_column = 0;
                else if(current_column > local_map.info.width) current_column = local_map.info.width;
                if(current_row < 0) current_row = 0;
                else if(current_row > show_height) current_row = show_height;

                pt_e2.y = current_row;
                pt_e2.x = current_column;
                cv::line(occ_mat_bgr, pt_e1, pt_e2, cv::Scalar(255, 0, 0), 1, 8);
                break;
            }

        }

    }


    cv::Mat occ_mat_bgr_r;
    cv::flip(occ_mat_bgr, occ_mat_bgr_r, 0);
    cv::namedWindow("casting_map", 0);
    cv::imshow("casting_map", occ_mat_bgr_r);
    cv::waitKey(1);

}






int main(int argc, char **argv) {
    ros::init(argc, argv, "local_map_node");
    ros::NodeHandle nh("~");
    ros::Subscriber publisher = nh.subscribe("/traversible_area_topic", 1, traversibleMapCallback);
    ros::Subscriber vehicle_global_pose_sub = nh.subscribe("/vehicle_global_pose_topic",1,vehiclePoseCallback);
    ros::Publisher map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/local_map", 1, true);
    ros::Publisher vehicle_footprint_pub_ = nh.advertise<visualization_msgs::Marker>("footprint", 10, false);

    double map_width;
    double map_height;
    double map_resolution;
    nh.param<double>("map_width", map_width, 50);
    nh.param<double>("map_height", map_height, 125);
    nh.param<double>("vehicle_x_in_map", vehicle_x_in_map, 25);  //[m]
    nh.param<double>("vehicle_y_in_map", vehicle_y_in_map, 25);
    nh.param<double>("far_distance", far_distance, 50);
    nh.param<double>("map_resolution", map_resolution, 0.1);

    local_map.header.frame_id = "base_link";
    local_map.info.width = static_cast<int>(map_width / map_resolution);
    local_map.info.height = static_cast<int>(map_height / map_resolution);
    local_map.info.resolution = map_resolution;
    local_map.info.origin.position.x = -static_cast<double>(vehicle_x_in_map);
    local_map.info.origin.position.y = -static_cast<double>(vehicle_y_in_map);
    local_map.info.origin.orientation.w = 1.0;
    local_map.data.assign(local_map.info.width * local_map.info.height, 100);  // Fill with obs occupancy.


//    cast_local_map.header.frame_id = "base_link";
//    cast_local_map.info.width = map_width;
//    cast_local_map.info.height = map_height;
//    cast_local_map.info.resolution = map_resolution;
//    cast_local_map.info.origin.position.x = -static_cast<double>(vehicle_x_in_map);
//    cast_local_map.info.origin.position.y = -static_cast<double>(vehicle_y_in_map);
//    cast_local_map.info.origin.orientation.w = 1.0;
//    cast_local_map.data.assign(map_width * map_height, -1);  // Fill with "unknown" occupancy.


    // Fill in the lookup cache.
    const double angle_start = -M_PI;
    const double angle_end = angle_start + 2 * M_PI - 1e-6;
    for (double a = angle_start; a <= angle_end; a += M_PI / 720)
    {
        ray_caster.getRayCastToMapBorder(a, 1250, 1250, 0.9 * M_PI / 720);
    }


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
        projectToVehicle(global_vehicle_pose, traversible_map);
        clearFarRegion();

        local_map.header.stamp = ros::Time::now();
//        cast_local_map.header.stamp = ros::Time::now();
        map_publisher.publish(local_map);
        auto end = std::chrono::system_clock::now();
        auto msec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
        std::cout << "local map build cost time msec :" << msec << "\n";

        // initial the local map again
        local_map.data.assign(local_map.info.width * local_map.info.height, 100);  // Fill with obs occupancy.

//
//        start = std::chrono::system_clock::now();
//        // ray casting
//        rayCasting();
//        end = std::chrono::system_clock::now();
//        msec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
//        std::cout << "ray cast cost time msec :" << msec << "\n";

        // show vehicle body
        {
            // displayFootprint
            visualization_msgs::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time();
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.scale.x = 2.8;
            marker.scale.y = 4.9;
            marker.scale.z = 2.0;
            marker.color.a = 0.3;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.frame_locked = true;

            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            vehicle_footprint_pub_.publish(marker);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}