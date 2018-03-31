//
// Created by kevin on 3/19/18.
//

#include "explore_large_map/map_builder.hpp"

//#define OPENCV_SHOW

namespace explore_global_map {

    explore_global_map::MapBuilder::MapBuilder(double width, double height, double resolution) :
            private_nh_("~"),
            global_map_height_(height),
            global_map_width_(width),
            start_flag_(false),
      tailored_submap_width_(50),
      tailored_submap_height_(75),
      tailored_submap_x2base_(25){
        map_.header.frame_id = "/odom";
        map_.info.width = static_cast<int> (width / resolution);
        map_.info.height = static_cast<int> (height / resolution);
        map_.info.resolution = resolution;
        map_.info.origin.orientation.w = 1.0;  // "odom"

        map_.info.origin.position.x = -width / 2;
        map_.info.origin.position.y = -height / 2;

        private_nh_.param<int>("unknown_value", unknown_value_, -1);
        map_.data.assign(map_.info.width * map_.info.height, unknown_value_);  // Fill with "unknown" occupancy.

        vehicle_footprint_pub_ = private_nh_.advertise<visualization_msgs::Marker>("footprint", 10, false);
    }

    void MapBuilder::grow(nav_msgs::Odometry &global_vehicle_pose,
                          nav_msgs::OccupancyGrid &local_map) {
        // get abosolute x,y
        geographic_to_grid(global_vehicle_pose.pose.pose.position.x, global_vehicle_pose.pose.pose.position.y);

        if (!start_flag_) {
            geometry_msgs::Quaternion msg;
            msg = adjustRPYConvention(global_vehicle_pose.pose.pose.orientation);
            global_vehicle_pose.pose.pose.orientation = msg;

            tf::Quaternion q;
            tf::quaternionMsgToTF(msg, q);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
            // todo
            if (1/*fabs(pitch) < 0.1 && fabs(roll) < 0.1*/) {
                initial_x_ = global_vehicle_pose.pose.pose.position.x;
                initial_y_ = global_vehicle_pose.pose.pose.position.y;
                global_vehicle_pose.pose.pose.position.x -= initial_x_;
                global_vehicle_pose.pose.pose.position.y -= initial_y_;
                start_flag_ = true;
            } else {
                ROS_WARN("current roll : %f, pitvh : %f ,Waiting for more flat position !", roll, pitch);
            }
        } else {
            global_vehicle_pose.pose.pose.position.x = global_vehicle_pose.pose.pose.position.x - initial_x_;
            global_vehicle_pose.pose.pose.position.y = global_vehicle_pose.pose.pose.position.y - initial_y_;

            // reverse yaw and roll sequence
            geometry_msgs::Quaternion msg;
            msg = adjustRPYConvention(global_vehicle_pose.pose.pose.orientation);
            global_vehicle_pose.pose.pose.orientation = msg;

            ROS_INFO_THROTTLE(0.5, "vehicle position in odom frame (%f[m], %f[m], %f[degree])",
                              global_vehicle_pose.pose.pose.position.x, global_vehicle_pose.pose.pose.position.y,
                              tf::getYaw(global_vehicle_pose.pose.pose.orientation) * 180 / M_PI);

            geometry_msgs::Pose abso_odom_vehicle_pose;
            geometry_msgs::Pose base_odom_pose;
            base_odom_pose.position.x = initial_x_;
            base_odom_pose.position.y = initial_y_;
            tf::quaternionTFToMsg(tf::createQuaternionFromYaw(0.0), base_odom_pose.orientation);
            tf::Pose ps;
            tf::poseMsgToTF(global_vehicle_pose.pose.pose, ps);
            tf::poseTFToMsg(mapToWorld(base_odom_pose) * ps, abso_odom_vehicle_pose);
            ROS_INFO_THROTTLE(0.5, "vehicle position in abs_odom frame (%f[m], %f[m], %f[degree])",
                              abso_odom_vehicle_pose.position.x, abso_odom_vehicle_pose.position.y,
                              tf::getYaw(abso_odom_vehicle_pose.orientation) * 180 / M_PI);
            ROS_INFO_THROTTLE(0.5, "odom_base_x: %f[m], odom_base_y : %f[m]", initial_x_, initial_y_);
            // publish marker in explore_map frame
            publishFootPrint(global_vehicle_pose.pose.pose, "/odom");
//            // broadcast tf tree between vehicle and explore map
            broadcastTransformBetweenVehicleAndExploreMap(global_vehicle_pose.pose.pose);
//            // broadcast tf tree betw explore map and odom
            broadcastTransformBetweenExploreMapAndOdom();


            // fill tailored submap cell value into odom global map
            geometry_msgs::Point pt_local;
            geometry_msgs::Point pt_global;

//            auto start = std::chrono::system_clock::now();

            tf::Transform all_trans;
            all_trans = mapToWorld(global_vehicle_pose.pose.pose) * mapToWorld(local_map.info.origin) ;
            for (int i = 0; i < local_map.info.height; i++) {
                for (int j = 0; j < local_map.info.width; j++) {
                    int index_in_local_map = i * local_map.info.width + j;
                    // -->local_submap
                    pt_local.x = j * local_map.info.resolution;
                    pt_local.y = i * local_map.info.resolution;
                    pt_local.z = 0;

                    // -->global_submap
                    {
                        tf::Point pt;
                        tf::pointMsgToTF(pt_local, pt);
                        tf::pointTFToMsg(all_trans * pt, pt_global);
                    }

                    int global_map_x = floor((pt_global.x - map_.info.origin.position.x) / map_.info.resolution);
                    int global_map_y = floor((pt_global.y - map_.info.origin.position.y) / map_.info.resolution);
                    int index_in_global_map = global_map_y * map_.info.width + global_map_x;
                    if (global_map_x > 0 && global_map_x < map_.info.width && global_map_y > 0 &&
                        global_map_y < map_.info.height) {

                        // todo use bayes inference
                       if (local_map.data[index_in_local_map] == 100) {
                            map_.data[index_in_global_map] = 100;
                        } else if(local_map.data[index_in_local_map] == 0){
                           // when view unknown cell is free, this free cells assignement are error!!!
                           map_.data[index_in_global_map] = 0;
                        } // only consider obs and free, ignore unknwon cell cover
//                            map_.data[index_in_global_map] = -1;
                    }
                }
            }
        }
    }

    void MapBuilder::tailorSubmap(const iv_slam_ros_msgs::TraversibleArea &traver_map,
                                  iv_slam_ros_msgs::TraversibleArea &tailored_submap) {

        auto submap_x = traver_map.triD_submap_pose_image_index_x;
        auto submap_y = traver_map.triD_submap_pose_image_index_y;
        int shift_cell_x = tailored_submap_width_ / traver_map.resolution / 2;
        int nagative_shift_cell_y = tailored_submap_x2base_ / traver_map.resolution;
        int positive_shift_cell_y = (tailored_submap_height_ - tailored_submap_x2base_) / traver_map.resolution;

        tailored_submap.header = traver_map.header;
        tailored_submap.resolution = traver_map.resolution;
        tailored_submap.triD_submap_pose = traver_map.triD_submap_pose;


        int start_index_x = submap_x - shift_cell_x < 0 ? 0 : submap_x - shift_cell_x;
        int end_index_x = submap_x + shift_cell_x > traver_map.width ? traver_map.width : submap_x + shift_cell_x ;
        int start_index_y = submap_y - nagative_shift_cell_y < 0 ? 0 : submap_y - nagative_shift_cell_y ;
        int end_index_y = submap_y + positive_shift_cell_y > traver_map.height ? traver_map.height : submap_y + positive_shift_cell_y;


        tailored_submap.width = end_index_x - start_index_x ;
        tailored_submap.height = end_index_y - start_index_y;
        tailored_submap.triD_submap_pose_image_index_x = submap_x - start_index_x;
        tailored_submap.triD_submap_pose_image_index_y = submap_y - start_index_y;
        tailored_submap.cells.assign(tailored_submap.width * tailored_submap.height, 0);  //0 mean unknown cell

//        ROS_INFO("tailor_map: width --> %d  height --> %d , ref position :(%d, %d)", tailored_submap.width, tailored_submap.height,
//                 submap_x - start_index_x, submap_y - start_index_y);

        uint8 value;
        int y_tailor = 0;
        for(int y = start_index_y; y < end_index_y; ++y, ++y_tailor) {
            int x_tailor = 0;
            for(int x = start_index_x; x < end_index_x; ++x, ++x_tailor) {
                value = traver_map.cells[y * traver_map.width + x];
//                if(value == 2) {
//                  ROS_INFO("""""""");
//                }
                tailored_submap.cells[y_tailor * tailored_submap.width + x_tailor] = value;

            }
        }
        //todo check bug
//        for(int i = 0; i < tailored_submap.height && start_index_y < end_index_y; i++, start_index_y++) {
//            for(int j = 0; j < tailored_submap.width && start_index_x < end_index_x; j++, start_index_x++) {
//                int index_in_tailor = i * tailored_submap.width + j;
//                int index_in_submap = start_index_y * traver_map.width + start_index_x;
//                ROS_INFO("---------%d", traver_map.cells[index_in_submap]);
//                if(traver_map.cells[index_in_submap] == 2) {
//                    tailored_submap.cells[index_in_tailor] = traver_map.cells[index_in_submap];
//                }
//            }
//        }

    }

    void MapBuilder::broadcastTransformBetweenVehicleAndExploreMap(geometry_msgs::Pose &current_pose) {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(current_pose.position.x,
                                        current_pose.position.y, current_pose.position.z));
        tf::Quaternion q;
        tf::quaternionMsgToTF(current_pose.orientation, q);
        transform.setRotation(q);
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "base_link"));

    }

    void MapBuilder::broadcastTransformBetweenExploreMapAndOdom() {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(initial_x_,
                                        initial_y_, 0));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
//        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/abso_odom", "/odom"));

    }

    void MapBuilder::publishFootPrint(const geometry_msgs::Pose &pose, const std::string &frame) {
        // displayFootprint
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame;
        marker.header.stamp = ros::Time();
        marker.ns = "global_map/footprint";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 4.9;
        marker.scale.y = 2.8;
        marker.scale.z = 2.0;
        marker.color.a = 0.3;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.frame_locked = true;

        marker.pose = pose;

        vehicle_footprint_pub_.publish(marker);
    }


}
