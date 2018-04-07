//
// Created by kevin on 3/19/18.
//

#include "explore_large_map/map_builder.hpp"

//#define OPENCV_SHOW
#define BayesUpdate

namespace explore_global_map {

    const double g_default_p_occupied_when_laser = 0.9;
    const double g_default_p_occupied_when_no_laser = 0.3;
    const double g_default_large_log_odds = 100;
    const double g_default_max_log_odds_for_belief = 20;


    explore_global_map::MapBuilder::MapBuilder(double width, double height, double resolution) :
            private_nh_("~"),
            global_map_height_(height),
            global_map_width_(width),
            global_map_frame_name_("/odom"),
            local_map_frame_name_("base_link"),
            abso_global_map_frame_name_("/abso_odom"),
            start_flag_(false),
            p_occupied_when_laser_(g_default_p_occupied_when_laser),
            p_occupied_when_no_laser_(g_default_p_occupied_when_no_laser),
            large_log_odds_(g_default_large_log_odds),
            max_log_odds_for_belief_(g_default_max_log_odds_for_belief),
      tailored_submap_width_(50),
      tailored_submap_height_(75),
      tailored_submap_x2base_(25) {

        private_nh_.param<std::string>("global_map_frame_name", global_map_frame_name_, "/odom");
        private_nh_.param<std::string>("local_map_frame_name", local_map_frame_name_, "base_link");
        private_nh_.param<std::string>("abso_global_map_frame_name", abso_global_map_frame_name_, "/abso_odom");

        private_nh_.param<double>("border_thickness", border_thickness_, 2);

        private_nh_.getParam("p_occupied_when_laser", p_occupied_when_laser_);
        if (p_occupied_when_laser_ <=0 || p_occupied_when_laser_ >= 1)
        {
            ROS_ERROR_STREAM("Parameter "<< private_nh_.getNamespace() <<
                                         "/p_occupied_when_laser must be within ]0, 1[, setting to default (" <<
                                         g_default_p_occupied_when_laser << ")");
            p_occupied_when_laser_ = g_default_p_occupied_when_laser;
        }
        private_nh_.getParam("p_occupied_when_no_laser", p_occupied_when_no_laser_);
        if (p_occupied_when_no_laser_ <=0 || p_occupied_when_no_laser_ >= 1)
        {
            ROS_ERROR_STREAM("Parameter "<< private_nh_.getNamespace() <<
                                         "/p_occupied_when_no_laser must be within ]0, 1[, setting to default (" <<
                                         g_default_p_occupied_when_no_laser << ")");
            p_occupied_when_no_laser_ = g_default_p_occupied_when_no_laser;
        }
        private_nh_.getParam("large_log_odds", large_log_odds_);
        if (large_log_odds_ <=0)
        {
            ROS_ERROR_STREAM("Parameter "<< private_nh_.getNamespace() << "/large_log_odds must be positive, setting to default (" <<
                                         g_default_large_log_odds << ")");
            large_log_odds_ = g_default_large_log_odds;
        }
        private_nh_.getParam("max_log_odds_for_belief", max_log_odds_for_belief_);
        try
        {
            std::exp(max_log_odds_for_belief_);
        }
        catch (std::exception)
        {
            ROS_ERROR_STREAM("Parameter "<< private_nh_.getNamespace() << "/max_log_odds_for_belief too large, setting to default (" <<
                                         g_default_max_log_odds_for_belief << ")");
            max_log_odds_for_belief_ = g_default_max_log_odds_for_belief;
        }

        map_.header.frame_id = global_map_frame_name_;
        map_.info.width = static_cast<int> (width / resolution);
        map_.info.height = static_cast<int> (height / resolution);
        map_.info.resolution = resolution;
        map_.info.origin.orientation.w = 1.0;  // "odom"

        map_.info.origin.position.x = -width / 2;
        map_.info.origin.position.y = -height / 2;

        private_nh_.param<int>("unknown_value", unknown_value_, -1);
        map_.data.assign(map_.info.width * map_.info.height, unknown_value_);  // Fill with "unknown" occupancy.

        // log_odds = log(occupancy / (1 - occupancy); prefill with
        // occupancy = 0.5, equiprobability between occupied and free.
        log_odds_.assign(map_.info.width * map_.info.height, 0);

        vehicle_footprint_pub_ = private_nh_.advertise<visualization_msgs::Marker>("/vehicle_in_global", 10, false);
    }

    void MapBuilder::grow(const nav_msgs::Odometry &vehicle_pose,
                          const nav_msgs::OccupancyGrid &local_map) {
        vehicle_pose_in_odom_map_ = vehicle_pose;
        // get abosolute x,y
        geographic_to_grid(vehicle_pose_in_odom_map_.pose.pose.position.x, vehicle_pose_in_odom_map_.pose.pose.position.y);

        if (!start_flag_) {
            geometry_msgs::Quaternion msg;
            msg = adjustRPYConvention(vehicle_pose_in_odom_map_.pose.pose.orientation);
            vehicle_pose_in_odom_map_.pose.pose.orientation = msg;

            tf::Quaternion q;
            tf::quaternionMsgToTF(msg, q);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
            // todo
            if (1/*fabs(pitch) < 0.1 && fabs(roll) < 0.1*/) {
                initial_x_ = vehicle_pose_in_odom_map_.pose.pose.position.x;
                initial_y_ = vehicle_pose_in_odom_map_.pose.pose.position.y;
                vehicle_pose_in_odom_map_.pose.pose.position.x -= initial_x_;
                vehicle_pose_in_odom_map_.pose.pose.position.y -= initial_y_;
                vehicle_pose_in_odom_map_.pose.pose.position.z = 0;  // set to 0
                start_flag_ = true;
            } else {
                ROS_WARN("current roll : %f, pitvh : %f ,Waiting for more flat position !", roll, pitch);
            }
        } else {
            vehicle_pose_in_odom_map_.pose.pose.position.x = vehicle_pose_in_odom_map_.pose.pose.position.x - initial_x_;
            vehicle_pose_in_odom_map_.pose.pose.position.y = vehicle_pose_in_odom_map_.pose.pose.position.y - initial_y_;
            vehicle_pose_in_odom_map_.pose.pose.position.z = 0;  // set to 0

            // reverse yaw and roll sequence
            geometry_msgs::Quaternion msg;
            msg = adjustRPYConvention(vehicle_pose_in_odom_map_.pose.pose.orientation);
            vehicle_pose_in_odom_map_.pose.pose.orientation = msg;

            ROS_INFO_THROTTLE(5, "vehicle position in odom frame (%f[m], %f[m], %f[degree])",
                              vehicle_pose_in_odom_map_.pose.pose.position.x, vehicle_pose_in_odom_map_.pose.pose.position.y,
                              tf::getYaw(vehicle_pose_in_odom_map_.pose.pose.orientation) * 180 / M_PI);

            geometry_msgs::Pose abso_odom_vehicle_pose;
            geometry_msgs::Pose base_odom_pose;
            base_odom_pose.position.x = initial_x_;
            base_odom_pose.position.y = initial_y_;
            tf::quaternionTFToMsg(tf::createQuaternionFromYaw(0.0), base_odom_pose.orientation);
            tf::Pose ps;
            tf::poseMsgToTF(vehicle_pose_in_odom_map_.pose.pose, ps);
            tf::poseTFToMsg(mapToWorld(base_odom_pose) * ps, abso_odom_vehicle_pose);
            ROS_INFO_THROTTLE(5, "vehicle position in abs_odom frame (%f[m], %f[m], %f[degree])",
                              abso_odom_vehicle_pose.position.x, abso_odom_vehicle_pose.position.y,
                              tf::getYaw(abso_odom_vehicle_pose.orientation) * 180 / M_PI);
            ROS_INFO_THROTTLE(10, "odom_base_x: %f[m], odom_base_y : %f[m]", initial_x_, initial_y_);
            // publish marker in explore_map frame
            publishFootPrint(vehicle_pose_in_odom_map_.pose.pose, global_map_frame_name_);
//            // broadcast tf tree between vehicle and explore map
            broadcastTransformBetweenVehicleAndExploreMap(vehicle_pose_in_odom_map_.pose.pose);
//            // broadcast tf tree betw explore map and odom
            broadcastTransformBetweenExploreMapAndOdom();


            // fill tailored submap cell value into odom global map
            geometry_msgs::Point pt_local;
            geometry_msgs::Point pt_global;

//            auto start = std::chrono::system_clock::now();

            tf::Transform all_trans;
            all_trans = mapToWorld(vehicle_pose_in_odom_map_.pose.pose) * mapToWorld(local_map.info.origin) ;
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
#ifdef BayesUpdate
                           updatePointOccupancy(true, true, index_in_global_map, map_.data, log_odds_);
#else
                            map_.data[index_in_global_map] = 100;
#endif

                        } else if(local_map.data[index_in_local_map] == 0){
#ifdef BayesUpdate
                           updatePointOccupancy(true, false, index_in_global_map, map_.data, log_odds_);
#else
                           // when view unknown cell is free, this free cells assignement are error!!!
                           map_.data[index_in_global_map] = 0;
#endif

                        } // only consider obs and free, ignore unknwon cell cover
//                            map_.data[index_in_global_map] = -1;
                    }
                }
            }
          
           // black border
            int thickness = static_cast<int>(border_thickness_ / map_.info.resolution);
            for (size_t y = 0; y < map_.info.height; y++) {
                if (y < thickness || y >= map_.info.height - thickness) {
                    for (size_t x = 0; x < map_.info.width; x++) {
                        unsigned int index = y * map_.info.width + x;
                        map_.data[index] = 100;  // black border
                    }
                    continue;
                }
                for (size_t x = 0; x < map_.info.width; x++) {
                    unsigned int index = y * map_.info.width + x;
                    if (x < thickness || x >= map_.info.width - thickness) {
                        map_.data[index] = 100;
                        continue;
                    }
                }
            }
        }
    }

    /** Update occupancy and log odds for a point
 *
 * @param[in] use_bayes true if consider error of sensor
 * @param[in] occupied true if the point was measured as occupied
 * @param[in] idx pixel index
 * @param[in] ncol map width
 * @param[in,out] occupancy occupancy map to update
 * @param[in,out] log_odds log odds to update
 */
    void MapBuilder::updatePointOccupancy(bool use_bayes, bool occupied, size_t idx, std::vector<int8_t>& occupancy,
                                          std::vector<double>& log_odds) const  {
        if (idx >= occupancy.size())
        {
            return;
        }

        if (occupancy.size() != log_odds.size())
        {
            ROS_ERROR("occupancy and count do not have the same number of elements");
            return;
        }
        if(use_bayes) {
            // Update log_odds.
            double p;  // Probability of being occupied knowing current measurement.
            if (occupied) {
                p = p_occupied_when_laser_;
            } else {
                p = p_occupied_when_no_laser_;
            }
            // Original formula: Table 4.2, "Probabilistics robotics", Thrun et al., 2005:
            // log_odds[idx] = log_odds[idx] +
            //     std::log(p * (1 - p_occupancy) / (1 - p) / p_occupancy);
            // With p_occupancy = 0.5, this simplifies to:
            log_odds[idx] += std::log(p / (1 - p));
            if (log_odds[idx] < -large_log_odds_) {
                log_odds[idx] = -large_log_odds_;
            } else if (log_odds[idx] > large_log_odds_) {
                log_odds[idx] = large_log_odds_;
            }
            // Update occupancy.
            if (log_odds[idx] < -max_log_odds_for_belief_) {
                occupancy[idx] = 0;
            } else if (log_odds[idx] > max_log_odds_for_belief_) {
                occupancy[idx] = 100;
            } else {
                occupancy[idx] = static_cast<int8_t>(lround((1 - 1 / (1 + std::exp(log_odds[idx]))) * 100));
            }

            // assign status
            if(occupancy[idx] >= 0 && occupancy[idx] <= 30) {
                occupancy[idx] = 0;
            } else if(occupancy[idx] >= 80 && occupancy[idx] <= 100){
                occupancy[idx] = 100;
            } else {
                // unknown cells
                occupancy[idx] = -1;
            }
        }  // consider pure ideal cases
        else {
            if(occupied) {
                occupancy[idx] = 100;
            } else {
                occupancy[idx] = 0;
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
                                        current_pose.position.y, 0));
        tf::Quaternion q;
        tf::quaternionMsgToTF(current_pose.orientation, q);
        transform.setRotation(q);
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), global_map_frame_name_, local_map_frame_name_));

    }

    void MapBuilder::broadcastTransformBetweenExploreMapAndOdom() {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(initial_x_,
                                        initial_y_, 0));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
//        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), abso_global_map_frame_name_, global_map_frame_name_));

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
        marker.scale.y = 1.95;
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
