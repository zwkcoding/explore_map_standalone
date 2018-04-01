#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "explore_large_map/map_builder.hpp"


nav_msgs::Odometry global_vehicle_pose;
nav_msgs::OccupancyGrid local_map;
geometry_msgs::PoseStamped global_goal_pose;

ros::WallTime last_receive_map_timestamp_;
ros::WallTime last_receive_position_timestamp_;

bool receive_vehicle_pose = false, receive_traversible_map = false, goal_flag = false;

void vehiclePoseCallback(const nav_msgs::Odometry &sub_tem_global_vehicle_pose){
    receive_vehicle_pose = true;
    last_receive_map_timestamp_ = ros::WallTime::now();  // tick time

    global_vehicle_pose.header.stamp = sub_tem_global_vehicle_pose.header.stamp;
    global_vehicle_pose.header.frame_id = sub_tem_global_vehicle_pose.header.frame_id;
    global_vehicle_pose.pose.pose.position.x = sub_tem_global_vehicle_pose.pose.pose.position.x;//longitude of the current UGV positon
    global_vehicle_pose.pose.pose.position.y = sub_tem_global_vehicle_pose.pose.pose.position.y;//latitude of the current UGV positon
    global_vehicle_pose.pose.pose.position.z = sub_tem_global_vehicle_pose.pose.pose.position.z;// the following result is pose relative to the start point of UGV
    global_vehicle_pose.pose.pose.orientation.w = sub_tem_global_vehicle_pose.pose.pose.orientation.w;
    global_vehicle_pose.pose.pose.orientation.x = sub_tem_global_vehicle_pose.pose.pose.orientation.x;
    global_vehicle_pose.pose.pose.orientation.y = sub_tem_global_vehicle_pose.pose.pose.orientation.y;
    global_vehicle_pose.pose.pose.orientation.z = sub_tem_global_vehicle_pose.pose.pose.orientation.z;

}

void localMapCall( const nav_msgs::OccupancyGridConstPtr &map) {
    receive_traversible_map = true;
    last_receive_position_timestamp_ = ros::WallTime::now();  // tick time

    local_map = *map;

}

geometry_msgs::Pose transformPose(geometry_msgs::Pose &pose, tf::Transform &tf){
    // Convert ROS pose to TF pose
    tf::Pose tf_pose;
    tf::poseMsgToTF(pose, tf_pose);

    // Transform pose
    tf_pose = tf * tf_pose;

    // normalize quaternion
    tf::Quaternion q = tf_pose.getRotation().normalize();
    tf_pose.setRotation(q);

    // Convert TF pose to ROS pose
    geometry_msgs::Pose ros_pose;
    tf::poseTFToMsg(tf_pose, ros_pose);
    return ros_pose;
}


void goalPoseCallback (const geometry_msgs::PoseStampedConstPtr &msg)
{
    if (!receive_traversible_map)
        return;

    ROS_INFO("Subcscribed goal pose!");
    double yaw = tf::getYaw(msg->pose.orientation);
    ROS_INFO("goal cell [in global_map coord]: [%f[m], %f[m], %f[degree]]", msg->pose.position.x, msg->pose.position.y, yaw * 180 / M_PI);

    global_goal_pose = *msg;
    goal_flag = true;
}

void global2Vehicle (const tf::TransformListener &tf_listner,
                     const geometry_msgs::PoseStamped &global_pose,
                     geometry_msgs::PoseStamped &local_pose) {
    std::string local_frame  = local_pose.header.frame_id;
    std::string goal_frame  = global_pose.header.frame_id;

    // Get transform (map to world in Autoware)
    tf::StampedTransform vehicle2world;
    try
    {
        tf_listner.lookupTransform(local_frame, goal_frame, ros::Time(0), vehicle2world);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    // Set pose in Global frame
    geometry_msgs::Pose msg_pose = global_pose.pose;
    local_pose.pose   = transformPose(msg_pose, vehicle2world);
    local_pose.header.stamp = ros::Time::now();
}

// todo push these into a class, wrapper
void resetFLag() {
    ros::WallTime end = ros::WallTime::now();
    double map_elapse_time = (end - last_receive_map_timestamp_).toSec() * 1000;
    double position_elapse_time = (end - last_receive_position_timestamp_).toSec() * 1000;
    if(map_elapse_time < 1000) {
        receive_traversible_map   = true;
    } else {
        receive_traversible_map   = false;
        ROS_ERROR("Receive local map delay exceed time!");
    }
    if(position_elapse_time < 1000) {
        receive_vehicle_pose = true;
    }else {
        receive_vehicle_pose   = false;
        ROS_ERROR("Receive vehicle position delay exceed time!");
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "explore_large_map_node");
    ros::NodeHandle nh("~");
    ros::NodeHandle n;

    double map_width;
    double map_height;
    double map_resolution;
    double reach_goal_distance;
    std::string re_vehicle_global_position_topic_name, re_local_map_topic_name, local_map_frame_name;
    std::string re_goal_pose_topic_name, se_vehicle_pose_in_odom_topic_name, se_vehicle_pose_in_odom_topic_frame_name;
    std::string se_global_map_topic_name;
    std::string se_local_goal_topic_name;
    nh.param<double>("map_width", map_width, 100);
    nh.param<double>("map_height", map_height, 100);
    nh.param<double>("map_resolution", map_resolution, 0.1);
    nh.param<double>("reach_goal_distance", reach_goal_distance, 0.5);

    nh.param<std::string>("receive_local_map_topic_name", re_local_map_topic_name, "/explore_entry_map");
    nh.param<std::string>("receive_vehicle_global_position_topic_name", re_vehicle_global_position_topic_name, "/vehicle_global_pose_topic");
    nh.param<std::string>("receive_goal_pose_topic_name", re_goal_pose_topic_name, "/move_base_simple/goal");
    nh.param<std::string>("send_vehicle_pose_in_odom_topic_name", se_vehicle_pose_in_odom_topic_name, "/odom");
    nh.param<std::string>("send_vehicle_pose_in_odom_topic_frame_name", se_vehicle_pose_in_odom_topic_frame_name, "/odom");

    nh.param<std::string>("send_global_map_topic_name", se_global_map_topic_name, "/global_map");
    nh.param<std::string>("send_local_goal_topic_name", se_local_goal_topic_name, "/local_search_goal");

    nh.param<std::string>("local_map_frame_name", local_map_frame_name, "base_link");


    ros::Subscriber local_map_sub = n.subscribe(re_local_map_topic_name, 1, localMapCall);
    ros::Subscriber vehicle_global_pose_sub = n.subscribe(re_vehicle_global_position_topic_name,1,vehiclePoseCallback);
    ros::Subscriber goal_pose_sub = n.subscribe(re_goal_pose_topic_name,1,goalPoseCallback);
    ros::Publisher map_publisher = n.advertise<nav_msgs::OccupancyGrid>(se_global_map_topic_name, 1, false);
    ros::Publisher current_position_in_explore_map_pub = n.advertise<nav_msgs::Odometry>(se_vehicle_pose_in_odom_topic_name, 1, false);
    ros::Publisher local_goal_pub = n.advertise<geometry_msgs::PoseStamped>(se_local_goal_topic_name, 1, false);

    tf::TransformListener tf_listener;
    geometry_msgs::PoseStamped local_goal_pose;
    local_goal_pose.header.frame_id = local_map_frame_name;

    explore_global_map::MapBuilder map_builder(map_width, map_height, map_resolution);

    ros::Rate rate(100.0);
    while (nh.ok()) {
        ros::spinOnce();

        // wait for msg
        if(!receive_vehicle_pose || !receive_traversible_map) {
            rate.sleep();
            continue;
        }
        // reset flag
        resetFLag();

        // build map
        auto start = std::chrono::system_clock::now();
        map_builder.grow(global_vehicle_pose, local_map);
        auto end = std::chrono::system_clock::now();
        auto msec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
        ROS_INFO_STREAM_THROTTLE(3,"global explore map build cost time [msec] :" << msec);
        map_publisher.publish(map_builder.getMap());

        nav_msgs::Odometry odom_global_vehicle_pose;
        // fixit global_vehicle_pose is in /odom frame , not /abso_odom frame
        odom_global_vehicle_pose = map_builder.getPositionInOdomMap();
        odom_global_vehicle_pose.header.frame_id = se_vehicle_pose_in_odom_topic_frame_name;
        odom_global_vehicle_pose.header.stamp = ros::Time::now();
        current_position_in_explore_map_pub.publish(odom_global_vehicle_pose);


        // reset goal flag and no publisher, waiting new goal pose input
        if(std::hypot(global_goal_pose.pose.position.x - global_vehicle_pose.pose.pose.position.x,
                      global_goal_pose.pose.position.y - global_vehicle_pose.pose.pose.position.y) < reach_goal_distance) {
            goal_flag = false;
        }
        if(goal_flag) {
            global2Vehicle(tf_listener, global_goal_pose, local_goal_pose); // update local goal pose
            local_goal_pub.publish(local_goal_pose);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
