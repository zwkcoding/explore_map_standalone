//
// Created by kevin on 3/29/18.
//

#include <ros/ros.h>
#include "iv_slam_ros_msgs/TraversibleArea.h"
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "nav_msgs/Odometry.h"
#include "explore_large_map/rigid_transform.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "explore_large_map/transform.h"

#include <nav_msgs/OccupancyGrid.h>

nav_msgs::OccupancyGrid local_map;
nav_msgs::Odometry received_tem_global_vehicle_pose;
iv_slam_ros_msgs::TraversibleArea TwiDTraversibleArea;

void counterClockwiseRotatePoint(double ref_x, double ref_y, double angle, double &point_x, double &point_y) {
    double s = sin(angle);
    double c = cos(angle);

    // translate point back to origin:
    point_x -= ref_x;
    point_y -= ref_y;

    // rotate point
    double xnew = point_x * c - point_y * s;
    double ynew = point_x * s + point_y * c;

    // translate point back:
    point_x = xnew + ref_x;
    point_y = ynew + ref_y;
}

void vehicle_pose_topic_callback(const nav_msgs::Odometry &sub_tem_global_vehicle_pose) {
    received_tem_global_vehicle_pose.header.stamp = sub_tem_global_vehicle_pose.header.stamp;
    received_tem_global_vehicle_pose.header.frame_id = sub_tem_global_vehicle_pose.header.frame_id;
    received_tem_global_vehicle_pose.pose.pose.position.x = sub_tem_global_vehicle_pose.pose.pose.position.x;//longitude of the current UGV positon
    received_tem_global_vehicle_pose.pose.pose.position.y = sub_tem_global_vehicle_pose.pose.pose.position.y;//latitude of the current UGV positon
    received_tem_global_vehicle_pose.pose.pose.position.z = sub_tem_global_vehicle_pose.pose.pose.position.z;// the following result is pose relative to the start point of UGV
    received_tem_global_vehicle_pose.pose.pose.orientation.w = sub_tem_global_vehicle_pose.pose.pose.orientation.w;
    received_tem_global_vehicle_pose.pose.pose.orientation.x = sub_tem_global_vehicle_pose.pose.pose.orientation.x;
    received_tem_global_vehicle_pose.pose.pose.orientation.y = sub_tem_global_vehicle_pose.pose.pose.orientation.y;
    received_tem_global_vehicle_pose.pose.pose.orientation.z = sub_tem_global_vehicle_pose.pose.pose.orientation.z;

}

void testcallback(const iv_slam_ros_msgs::TraversibleArea &subed_msg) {

    TwiDTraversibleArea.header.stamp = subed_msg.header.stamp;
    TwiDTraversibleArea.header.frame_id = subed_msg.header.frame_id;//which is "traversible_area_frame"
    TwiDTraversibleArea.resolution = subed_msg.resolution; //resolution of the submap
    TwiDTraversibleArea.submap_finished_flag = subed_msg.submap_finished_flag;//flag to decide whether the current submap is finished;
    TwiDTraversibleArea.triD_submap_pose_image_index_x = subed_msg.triD_submap_pose_image_index_x; // location in the pictrue of the submap pose.
    TwiDTraversibleArea.triD_submap_pose_image_index_y = subed_msg.triD_submap_pose_image_index_y;

    TwiDTraversibleArea.triD_submap_pose.position.x = subed_msg.triD_submap_pose.position.x;//longitude of the submap
    TwiDTraversibleArea.triD_submap_pose.position.y = subed_msg.triD_submap_pose.position.y;//latitude of the submap
    TwiDTraversibleArea.triD_submap_pose.position.z = subed_msg.triD_submap_pose.position.z;// position relative to the start point of UGV
    TwiDTraversibleArea.triD_submap_pose.orientation.w = subed_msg.triD_submap_pose.orientation.w;//the following is the pose in the world frame
    TwiDTraversibleArea.triD_submap_pose.orientation.x = subed_msg.triD_submap_pose.orientation.x;
    TwiDTraversibleArea.triD_submap_pose.orientation.y = subed_msg.triD_submap_pose.orientation.y;
    TwiDTraversibleArea.triD_submap_pose.orientation.z = subed_msg.triD_submap_pose.orientation.z;

    TwiDTraversibleArea.width = subed_msg.width;
    TwiDTraversibleArea.height = subed_msg.height;
    TwiDTraversibleArea.cells.assign(subed_msg.cells.begin(), subed_msg.cells.end());
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "map_entry_node");
    ros::NodeHandle nh("~");
    ros::Subscriber test = nh.subscribe("/traversible_area_topic", 1, testcallback);
    ros::Subscriber vehicle_global_pose_topic_sub = nh.subscribe("/vehicle_global_pose_topic", 1,
                                                                 vehicle_pose_topic_callback);
    double a = 6378137;
    double e2 = 0.0818192 * 0.0818192;//e的平方
    cartographer::transform::GridZone zone = cartographer::transform::UTM_ZONE_51;
    cartographer::transform::Hemisphere hemi = cartographer::transform::HEMI_NORTH;


    ros::Publisher map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/explore_entry_map", 1, false);

    ros::Rate rate(100);

    double map_width;
    double map_height;
    double map_resolution;
    std::string map_frame_name;
    nh.param<double>("map_width", map_width, 100);
    nh.param<double>("map_height", map_height, 60);
    nh.param<double>("local_map_resolution", map_resolution, 0.2);
    nh.param<std::string>("local_map_frame_name", map_frame_name, "base_link");

    local_map.header.frame_id = map_frame_name;
    local_map.info.width = static_cast<int>(map_width / map_resolution);
    local_map.info.height = static_cast<int>(map_height / map_resolution);
    local_map.info.resolution = map_resolution;
    local_map.info.origin.position.x = -map_width / 2;
    local_map.info.origin.position.y = -map_height / 2;
    local_map.info.origin.orientation.w = 1.0;
    local_map.data.assign(local_map.info.width * local_map.info.height, 0);  // Fill with free occupancy.

    while (ros::ok()) {
        local_map.data.assign(local_map.info.width * local_map.info.height, 0);  // Fill with free occupancy.

        if (TwiDTraversibleArea.width == 0 && TwiDTraversibleArea.height == 0) {

            ros::spinOnce();
            continue;

        }
//     std::cout<<std::fixed<<std::setprecision(10)<< received_tem_global_vehicle_pose.pose.pose.position <<std::endl;
//     std::cout<< received_tem_global_vehicle_pose.pose.pose.orientation <<std::endl;
//     std::cout<<std::fixed<<std::setprecision(10)<<  TwiDTraversibleArea.triD_submap_pose.position <<std::endl;
//     std::cout<< TwiDTraversibleArea.triD_submap_pose.orientation <<std::endl;

        cartographer::transform::geographic_to_grid(a, e2,
                                                    (TwiDTraversibleArea.triD_submap_pose.position.y) * M_PI / 180,
                                                    (TwiDTraversibleArea.triD_submap_pose.position.x) * M_PI / 180,
                                                    &zone, &hemi, &(TwiDTraversibleArea.triD_submap_pose.position.y),
                                                    &(TwiDTraversibleArea.triD_submap_pose.position.x));

        cartographer::transform::Rigid3d TwiDTraversibleArea_pose = cartographer::transform::Rigid3d(
                Eigen::Vector3d(TwiDTraversibleArea.triD_submap_pose.position.x,
                                TwiDTraversibleArea.triD_submap_pose.position.y,
                                TwiDTraversibleArea.triD_submap_pose.position.z),
                Eigen::Quaternion<double>(TwiDTraversibleArea.triD_submap_pose.orientation.w,
                                          TwiDTraversibleArea.triD_submap_pose.orientation.x,
                                          TwiDTraversibleArea.triD_submap_pose.orientation.y,
                                          TwiDTraversibleArea.triD_submap_pose.orientation.z));

        cartographer::transform::geographic_to_grid(a, e2,
                                                    (received_tem_global_vehicle_pose.pose.pose.position.y) * M_PI /
                                                    180,
                                                    (received_tem_global_vehicle_pose.pose.pose.position.x) * M_PI /
                                                    180, &zone, &hemi,
                                                    &(received_tem_global_vehicle_pose.pose.pose.position.y),
                                                    &(received_tem_global_vehicle_pose.pose.pose.position.x));

        cartographer::transform::Rigid3d global_vehicle_pose = cartographer::transform::Rigid3d(
                Eigen::Vector3d(received_tem_global_vehicle_pose.pose.pose.position.x,
                                received_tem_global_vehicle_pose.pose.pose.position.y,
                                received_tem_global_vehicle_pose.pose.pose.position.z),
                Eigen::Quaternion<double>(received_tem_global_vehicle_pose.pose.pose.orientation.w,
                                          received_tem_global_vehicle_pose.pose.pose.orientation.x,
                                          received_tem_global_vehicle_pose.pose.pose.orientation.y,
                                          received_tem_global_vehicle_pose.pose.pose.orientation.z));


        IplImage *testimage = cvCreateImage(cvSize(TwiDTraversibleArea.width, TwiDTraversibleArea.height), IPL_DEPTH_8U,
                                            3);
        IplImage *showimage = cvCreateImage(cvSize(400, 800), IPL_DEPTH_8U, 3);
        int data_index = 0;
        cvZero(testimage);
        cvZero(showimage);

        int x_index2, y_index2;
        unsigned char *pdata;
        for (int i = 0; i < testimage->height; i++) {
            for (int j = 0; j < testimage->width; j++) {
                if ((TwiDTraversibleArea.cells.at(data_index) == 2)) {

                    cartographer::transform::Rigid3d tem_pose(Eigen::Vector3d(
                            ((data_index % testimage->width - TwiDTraversibleArea.triD_submap_pose_image_index_x) *
                             TwiDTraversibleArea.resolution),
                            ((data_index / testimage->width - TwiDTraversibleArea.triD_submap_pose_image_index_y) *
                             TwiDTraversibleArea.resolution), 0), Eigen::Quaternion<double>(1, 0, 0, 0));


                    cartographer::transform::Rigid3d global_pose = TwiDTraversibleArea_pose * tem_pose;

                    x_index2 = (global_vehicle_pose.inverse() * global_pose).translation().x() /
                               TwiDTraversibleArea.resolution;
                    y_index2 = (global_vehicle_pose.inverse() * global_pose).translation().y() /
                               TwiDTraversibleArea.resolution;

                    if (x_index2 > -200 && x_index2 < 200 && y_index2 > -400 && y_index2 < 400) {

                        pdata = (unsigned char *) (showimage->imageData +
                                                   (showimage->height - 1 - y_index2 - 400) * showimage->widthStep +
                                                   (x_index2 + 200) * 3);
                        pdata[0] = 254;
                        pdata[1] = 254;
                        pdata[2] = 254;
                    }

                    double rel_x = (global_vehicle_pose.inverse() * global_pose).translation().x();
                    double rel_y = (global_vehicle_pose.inverse() * global_pose).translation().y();

                    // clockwise rotate viewpoint, consistent with ros convention
                    counterClockwiseRotatePoint(0, 0, -M_PI_2, rel_x, rel_y);

                    int ind_x = floor((rel_x - local_map.info.origin.position.x) / local_map.info.resolution);
                    int ind_y = floor((rel_y - local_map.info.origin.position.y) / local_map.info.resolution);

                    if (ind_x > 0 && ind_x < local_map.info.width && ind_y > 0 && ind_y < local_map.info.height) {
                        int index_in_global_map = ind_y * local_map.info.width + ind_x;

                        local_map.data[index_in_global_map] = 100;
                    }


                } else if (0/*TwiDTraversibleArea.cells.at(data_index)==1*/) {

                    cartographer::transform::Rigid3d tem_pose(Eigen::Vector3d(
                            ((data_index % testimage->width - TwiDTraversibleArea.triD_submap_pose_image_index_x) *
                             TwiDTraversibleArea.resolution),
                            ((data_index / testimage->width - TwiDTraversibleArea.triD_submap_pose_image_index_y) *
                             TwiDTraversibleArea.resolution), 0), Eigen::Quaternion<double>(1, 0, 0, 0));


                    cartographer::transform::Rigid3d global_pose = TwiDTraversibleArea_pose * tem_pose;

                    x_index2 = (global_vehicle_pose.inverse() * global_pose).translation().x() /
                               TwiDTraversibleArea.resolution;
                    y_index2 = (global_vehicle_pose.inverse() * global_pose).translation().y() /
                               TwiDTraversibleArea.resolution;

                    if (x_index2 > -200 && x_index2 < 200 && y_index2 > -400 && y_index2 < 400) {

                        pdata = (unsigned char *) (showimage->imageData +
                                                   (showimage->height - 1 - y_index2 - 400) * showimage->widthStep +
                                                   (x_index2 + 200) * 3);
                        pdata[0] = 0;
                        pdata[1] = 254;
                        pdata[2] = 0;
                    }
                }
                data_index++;
            }
        }

        cvCircle(showimage, cvPoint(200, 400), 10, cvScalar(255, 0, 0), -1);

        cvShowImage("traversible area puslished", showimage);
        cvWaitKey(1);
        cvReleaseImage(&testimage);
        cvReleaseImage(&showimage);

        map_publisher.publish(local_map);

        ros::spinOnce();
        rate.sleep();
    }
}