#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "explore_large_map/map_builder.hpp"


nav_msgs::Odometry global_vehicle_pose;
nav_msgs::OccupancyGrid local_map;
iv_slam_ros_msgs::TraversibleArea traversible_map;

geometry_msgs::PoseStamped local_goal_pose;
geometry_msgs::PoseStamped global_goal_pose;

bool receive_vehicle_pose = false, receive_traversible_map = false, goal_flag = false;
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
    // TODO: what frame do we use?
    std::string local_frame  = "base_link";
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
    local_pose.header.frame_id = local_frame;
    local_pose.header.stamp = ros::Time::now();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "explore_large_map_node");
    ros::NodeHandle nh("~");

    ros::Subscriber local_map_sub = nh.subscribe("/explore_entry_map", 1, localMapCall);
    ros::Subscriber vehicle_global_pose_sub = nh.subscribe("/vehicle_global_pose_topic",1,vehiclePoseCallback);
    ros::Subscriber goal_pose_sub = nh.subscribe("/move_base_simple/goal",1,goalPoseCallback);
    ros::Publisher map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/global_map", 1, true);
    ros::Publisher current_position_in_explore_map_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1, false);
    ros::Publisher local_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_search_goal", 1, false);

    double map_width;
    double map_height;
    double map_resolution;

    nh.param<double>("map_width", map_width, 100);
    nh.param<double>("map_height", map_height, 100);
    nh.param<double>("map_resolution", map_resolution, 0.1);

    tf::TransformListener tf_listener;

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

        if(goal_flag) {
            global2Vehicle(tf_listener, global_goal_pose, local_goal_pose); // update local goal pose
            local_goal_pub.publish(local_goal_pose);
        }

        // reset goal flag and no publisher, waiting new goal pose input
        if(std::hypot(global_goal_pose.pose.position.x - global_vehicle_pose.pose.pose.position.x,
                      global_goal_pose.pose.position.y - global_vehicle_pose.pose.pose.position.y) < 1.0) {
            goal_flag = false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
