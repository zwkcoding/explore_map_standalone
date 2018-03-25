//
// Created by kevin on 3/19/18.
//

#ifndef EXPLORE_LARGE_MAP_MAP_BUILDER_HPP
#define EXPLORE_LARGE_MAP_MAP_BUILDER_HPP

#include <chrono>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <ros/ros.h>
#include <iv_slam_ros_msgs/TraversibleArea.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "explore_large_map/rigid_transform.h"
#include "explore_large_map/transform.h"

namespace explore_global_map {
    inline tf::Transform mapToWorld (const geometry_msgs::Pose &initial_pose) {
        tf::Transform world_to_map;
        tf::poseMsgToTF (initial_pose, world_to_map);
        return world_to_map;
    }

    inline tf::Transform worldToMap (const geometry_msgs::Pose &initial_pose) {
        return mapToWorld(initial_pose).inverse();
    }


    inline void geographic_to_grid(double &longitude, double &latitude) {
        double a=6378137;
        double e2= 0.0818192*0.0818192;

        cartographer::transform::GridZone zone =cartographer::transform:: UTM_ZONE_51;
        cartographer::transform::Hemisphere hemi = cartographer::transform:: HEMI_NORTH;
        double N = 0;
        double E = 0;
        cartographer::transform:: geographic_to_grid( a, e2, latitude * M_PI / 180, longitude * M_PI / 180,
                                                      &zone, &hemi,&N, &E);
        latitude = N;
        longitude = E;
    }

    inline geometry_msgs::Quaternion reverse_yaw_roll(const geometry_msgs::Quaternion &origin) {
        tf::Quaternion q;
        tf::quaternionMsgToTF(origin, q);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        // exchange roll and pitch, cuz different convention
        double temp = roll;
        roll = pitch;
        pitch = temp;
        tf::Quaternion q_new = tf::createQuaternionFromRPY(roll, pitch, yaw);
        geometry_msgs::Quaternion msg;
        tf::quaternionTFToMsg(q_new, msg);
        return msg;
    }

    class MapBuilder {

    public:
        MapBuilder(int width, int height, double resolution);

        void grow( nav_msgs::Odometry &global_vehicle_pose,
                   iv_slam_ros_msgs::TraversibleArea &traversible_map);

        nav_msgs::OccupancyGrid getMap() const {return map_;}

    private:

        void tailorSubmap(const iv_slam_ros_msgs::TraversibleArea &traver_map, iv_slam_ros_msgs::TraversibleArea &tailored_submap);

        void broadcastTransformBetweenVehicleAndExploreMap();
        void broadcastTransformBetweenExploreMapAndOdom();


        bool start_flag_;

        int tailored_submap_width_;
        int tailored_submap_height_;
        int tailored_submap_x2base_;

        double initial_x_, initial_y_;
        geometry_msgs::Pose initial_vehicle_pos_in_odom;
        geometry_msgs::Pose initial_vehicle_pos_in_explore_map;
        geometry_msgs::Pose current_odom_vehicle_pos_;

        tf::TransformBroadcaster br_;

        nav_msgs::OccupancyGrid map_; //!< local map with fixed orientation






    };
}

#endif //EXPLORE_LARGE_MAP_MAP_BUILDER_HPP
