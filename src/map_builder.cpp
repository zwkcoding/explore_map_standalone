//
// Created by kevin on 3/19/18.
//

#include "explore_large_map/map_builder.hpp"

#define OPENCV_SHOW

namespace explore_global_map {

    explore_global_map::MapBuilder::MapBuilder(int width, int height, double resolution)
    : start_flag_(false),
      tailored_submap_width_(60),
      tailored_submap_height_(40),
      tailored_submap_x2base_(10){
        map_.header.frame_id = "/odom";
        map_.info.width = width;
        map_.info.height = height;
        map_.info.resolution = resolution;
        map_.info.origin.position.x = -static_cast<double>(width) / 2 * resolution;
        map_.info.origin.position.y = -static_cast<double>(height) / 2 * resolution;
        map_.info.origin.orientation.w = 1.0;
        map_.data.assign(width * height, -1);  // Fill with "unknown" occupancy.


    }

    void MapBuilder::grow(nav_msgs::Odometry &global_vehicle_pose,
                          iv_slam_ros_msgs::TraversibleArea &traversible_map) {
#ifdef OPENCV_SHOW
        int map_width = traversible_map.width;
        int map_height = traversible_map.height;
        cv::Mat mat_src(map_height, map_width, CV_8UC1, cv::Scalar(127));
        for(int i = 0; i < map_height; i++) {
            for(int j = 0; j < map_width; j++) {
                int index = i * map_width + j;
                if(traversible_map.cells[index] == 1) {
                    mat_src.at<uchar>(i, j) = 255;
                } else if(traversible_map.cells[index] == 2) {
                    mat_src.at<uchar>(i,j) = 0;
                } else {
                    mat_src.at<uchar>(i,j) = 127;
                }
            }
        }
        int top_left_x = traversible_map.triD_submap_pose_image_index_x - tailored_submap_width_ / 2
                                                                          / traversible_map.resolution;
        int top_left_y = traversible_map.triD_submap_pose_image_index_y + (tailored_submap_height_ - tailored_submap_x2base_)
                                                                          /traversible_map.resolution;
        int bottom_right_x = traversible_map.triD_submap_pose_image_index_x + tailored_submap_width_ / 2
                                                                              / traversible_map.resolution;
        int bottom_right_y = traversible_map.triD_submap_pose_image_index_y - tailored_submap_x2base_ /traversible_map.resolution;

        cv::Point2i top_left(top_left_x, top_left_y), bottom_right(bottom_right_x, bottom_right_y);
        cv::Rect roi(top_left, bottom_right);
        cv::Mat image_roi = mat_src(roi);

        cv::Mat src_r, roi_r;
        cv::flip(mat_src, src_r, 1);
        cv::flip(image_roi, roi_r, 1);
        cv::imshow("source", src_r);
        cv::imshow("roi", roi_r);
        cv::waitKey(1);

#endif
        // get x,y
        geographic_to_grid(global_vehicle_pose.pose.pose.position.y, global_vehicle_pose.pose.pose.position.x);
        geographic_to_grid(traversible_map.triD_submap_pose.position.y, traversible_map.triD_submap_pose.position.x);
        iv_slam_ros_msgs::TraversibleArea tailored_submap;
        tailored_submap = tailorSubmap(traversible_map);

        // reverse yaw and roll sequence
        geometry_msgs::Quaternion msg;
        ROS_INFO_STREAM("reverse before: " << tailored_submap.triD_submap_pose.orientation);
        msg = reverse_yaw_roll(tailored_submap.triD_submap_pose.orientation);
        ROS_INFO_STREAM("reverse after: " << msg);
        tailored_submap.triD_submap_pose.orientation = msg;
        msg = reverse_yaw_roll(global_vehicle_pose.pose.pose.orientation);
        global_vehicle_pose.pose.pose.orientation = msg;

        if(!start_flag_) {

            tf::Quaternion q;
            tf::quaternionMsgToTF(msg, q);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
            // todo
            if (pitch < 0.1 && roll < 0.1) {
                initial_global_vehicle_pos_ = global_vehicle_pose.pose.pose;
                start_flag_ = true;
            } else {
                ROS_WARN("current roll : %f, pitvh : %f ,Waiting for more flat position !", roll, pitch);
            }

        } else {
            // get submap rpy
            tf::Quaternion q;
            tf::quaternionMsgToTF(tailored_submap.triD_submap_pose.orientation, q);
            tf::Matrix3x3 m(q);
            double submap_roll, submap_pitch, submap_yaw;
            m.getRPY(submap_roll, submap_pitch, submap_yaw);
            // get vehicle rpy
            tf::quaternionMsgToTF(global_vehicle_pose.pose.pose.orientation, q);
            tf::Matrix3x3 m0(q);
            double vehicle_roll, vehicle_pitch, vehilce_yaw;
            m0.getRPY(vehicle_roll, vehicle_pitch, vehilce_yaw);

            // reserve current submap roll and pitch
            vehicle_pitch = submap_pitch;
            vehicle_roll = submap_roll;
            tf::Quaternion q_new = tf::createQuaternionFromRPY(vehicle_roll, vehicle_pitch, vehilce_yaw);
            tf::quaternionTFToMsg(q_new, global_vehicle_pose.pose.pose.orientation);
            // -->global_odom
            tf::Pose ps;
            tf::poseMsgToTF(global_vehicle_pose.pose.pose, ps);
            tf::poseTFToMsg(worldToMap(initial_global_vehicle_pos_) * ps, current_odom_vehicle_pos_);
            ROS_INFO("vehicle position in odom frame (%f[m], %f[m])", current_odom_vehicle_pos_.position.x,
                     current_odom_vehicle_pos_.position.y);
            // broadcast tf tree
            broadcastTransformBetweenVehicleAndOdom();

#ifdef OPENCV_SHOW
            int tmap_width = tailored_submap.width;
            int tmap_height = tailored_submap.height;
            cv::Mat tmat_src(tmap_height, tmap_width, CV_8UC1, cv::Scalar(127));
            for(int i = 0; i < tmap_height; i++) {
                for(int j = 0; j < tmap_width; j++) {
                    int index = i * tmap_width + j;
                    if(tailored_submap.cells[index] == 1) {
                        tmat_src.at<uchar>(i, j) = 255;
                    } else if(tailored_submap.cells[index] == 2) {
                        tmat_src.at<uchar>(i,j) = 0;
                    } else {
                        tmat_src.at<uchar>(i,j) = 127;
                    }
                }
            }
            cv::Mat t_src_r;
            cv::flip(tmat_src, t_src_r, 1);
            cv::imshow("tailored_map", t_src_r);
            cv::waitKey(1);
#endif

            // fill tailored submap cell value into odom global map
            geometry_msgs::Point pt_local_submap;
            geometry_msgs::Point pt_global_submap;
            geometry_msgs::Pose ps_global_submap;
            geometry_msgs::Pose ps_global_odom;
            int global_map_x, global_map_y;
            for(int i = 0; i < tailored_submap.height; i++) {
                for(int j = 0; j < tailored_submap.width; j++) {
                    int index_in_tailored_map = i * tailored_submap.width + j;
                    // -->local_submap
                    pt_local_submap.x = (j - tailored_submap.triD_submap_pose_image_index_x) * tailored_submap.resolution;
                    pt_local_submap.y = (i - tailored_submap.triD_submap_pose_image_index_y) * tailored_submap.resolution;
                    // -->global_submap
                    tf::Point pt(pt_local_submap.x, pt_local_submap.y, 0);
                    tf::pointTFToMsg(mapToWorld(tailored_submap.triD_submap_pose) * pt, pt_global_submap);
                    ps_global_submap.position = pt_global_submap;
                    ps_global_submap.orientation = tailored_submap.triD_submap_pose.orientation;
                    // -->global_odom
                    tf::Pose ps;
                    tf::poseMsgToTF(ps_global_submap, ps);
                    tf::poseTFToMsg(worldToMap(initial_global_vehicle_pos_) * ps, ps_global_odom);
                    global_map_x = floor( ps_global_odom.position.x / map_.info.resolution)  - map_.info.origin.position.x;
                    global_map_y = floor( ps_global_odom.position.y / map_.info.resolution) - map_.info.origin.position.y ;
                    int index_in_global_map = global_map_y * map_.info.width + global_map_x;
                    if(global_map_x > 0 && global_map_x < map_.info.width &&
                            global_map_y > 0 && global_map_y < map_.info.height) {
                        if(tailored_submap.cells[index_in_tailored_map] == 0) {
                            map_.data[index_in_global_map] = -1;
                        } else if(tailored_submap.cells[index_in_tailored_map] == 1) {
                            map_.data[index_in_global_map] = 0;
                        } else {
                            map_.data[index_in_global_map] = 100;
                        }
                    }
                }
            }
#ifdef OPENCV_SHOW
            int occ_map_width = map_.info.width;
            int occ_map_height = map_.info.height;
            cv::Mat occ_mat_src(occ_map_width, occ_map_height, CV_8UC1, cv::Scalar(127));
            for(int i = 0; i < occ_map_height; i++) {
                for(int j = 0; j < occ_map_width; j++) {
                    int index = i * occ_map_width + j;
                    if(map_.data[index] == 0) {
                        occ_mat_src.at<uchar>(i, j) = 255;
                    } else if(map_.data[index] == 100) {
                        occ_mat_src.at<uchar>(i,j) = 0;
                    } else {
                        occ_mat_src.at<uchar>(i,j) = 127;
                    }
                }
            }

            cv::Mat occ_mat_bgr;
            cv::cvtColor(occ_mat_src, occ_mat_bgr, CV_GRAY2BGR);
            cv::Point2d current_pos;
            current_pos.x = (current_odom_vehicle_pos_.position.x - map_.info.origin.position.x) / map_.info.resolution;
            current_pos.y = (current_odom_vehicle_pos_.position.y - map_.info.origin.position.y) / map_.info.resolution;
            cv::circle(occ_mat_bgr, current_pos, 5, cv::Scalar(255, 0, 0), -1);
            cv::Mat occ_mat_bgr_r;
            cv::flip(occ_mat_bgr, occ_mat_bgr_r, 1);
            cv::imshow("global_map", occ_mat_bgr_r);
            cv::waitKey(1);
#endif
        }


    }

    iv_slam_ros_msgs::TraversibleArea MapBuilder::tailorSubmap(const iv_slam_ros_msgs::TraversibleArea &traver_map) {

        geometry_msgs::Pose initial_global_submap_pose = traver_map.triD_submap_pose;
        auto submap_x = traver_map.triD_submap_pose_image_index_x;
        auto submap_y = traver_map.triD_submap_pose_image_index_y;
        iv_slam_ros_msgs::TraversibleArea tailored_submap;

        tailored_submap.header = traver_map.header;
        tailored_submap.resolution = traver_map.resolution;
        tailored_submap.triD_submap_pose = traver_map.triD_submap_pose;
        tailored_submap.triD_submap_pose_image_index_x = tailored_submap_width_ / 2 / traver_map.resolution;
        tailored_submap.triD_submap_pose_image_index_y = tailored_submap_x2base_ / traver_map.resolution;
        tailored_submap.width = tailored_submap_width_ / traver_map.resolution;
        tailored_submap.height = tailored_submap_height_ / traver_map.resolution;
        tailored_submap.cells.reserve(tailored_submap.width * tailored_submap.height);

        int start_index_x = submap_x -  tailored_submap.width / 2;
        int end_index_x = submap_x + tailored_submap.width / 2;
        int start_index_y = submap_y - tailored_submap_x2base_ / traver_map.resolution;
        int end_index_y = submap_y + (tailored_submap_height_ - tailored_submap_x2base_) / traver_map.resolution;

        for(int i = 0; i < tailored_submap.height, start_index_y < end_index_y; i++, start_index_y++) {
            for(int j = 0; j < tailored_submap.width, start_index_x < end_index_x; j++, start_index_x++) {
                int index_in_tailor = i * tailored_submap.width + j;
                int index_in_submap = start_index_y * traver_map.width + start_index_x;
                tailored_submap.cells[index_in_tailor] = traver_map.cells[index_in_submap];
            }
        }

        return tailored_submap;

    }

    void MapBuilder::broadcastTransformBetweenVehicleAndOdom() {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(current_odom_vehicle_pos_.position.x,
        current_odom_vehicle_pos_.position.y, 0));
        tf::Quaternion q;
        tf::quaternionMsgToTF(current_odom_vehicle_pos_.orientation, q);
        transform.setRotation(q);
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "base_link"));

    }
}
