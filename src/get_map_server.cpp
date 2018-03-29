#include <ros/ros.h>
#include <map>
#include <std_msgs/Float64.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <boost/thread.hpp>

using namespace ros;

nav_msgs::GetMap::Response local_map_, global_map_;
boost::mutex local_map_mutex, global_map_mutex;
bool got_local_map, got_global_map;
int g_obstacle_threshold;

// todo could make changes on map(ROI, replace value...)here
void localMapUpdate(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
	ROS_DEBUG("Updating a local map");
	boost::mutex::scoped_lock map_lock (local_map_mutex);

	local_map_.map.info.width = map->info.width;
	local_map_.map.info.height = map->info.height;
	local_map_.map.info.resolution = map->info.resolution;

	local_map_.map.info.map_load_time = map->info.map_load_time;

	local_map_.map.info.origin.position.x = map->info.origin.position.x;
	local_map_.map.info.origin.position.y = map->info.origin.position.y;
	local_map_.map.info.origin.position.z = map->info.origin.position.z;

	local_map_.map.info.origin.orientation.x = map->info.origin.orientation.x;
	local_map_.map.info.origin.orientation.y = map->info.origin.orientation.y;
	local_map_.map.info.origin.orientation.z = map->info.origin.orientation.z;
	local_map_.map.info.origin.orientation.w = map->info.origin.orientation.w;

	local_map_.map.data.resize(map->info.width*map->info.height);

	// todo local_map frontier leave empty for dubins curves
    /**
     * local_map_ ： 100 --> OBSTACLE/UNKNOWN
     *                0 --> FREE
     *
     */
	for(int i=0; i<map->data.size(); i++) {
        if(map->data[i] == -1) {
            local_map_.map.data[i] = map->data[i];
		} else if(map->data[i] <= g_obstacle_threshold) {
            local_map_.map.data[i] = 0;
		} else {
            local_map_.map.data[i] = 100;
		}
    }

	local_map_.map.header.stamp = map->header.stamp;
	local_map_.map.header.frame_id = map->header.frame_id;

	got_local_map = true;
}

void globalMapUpdate(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
	ROS_DEBUG("Updating a global map");
	boost::mutex::scoped_lock map_lock (global_map_mutex);

	global_map_.map.info.width = map->info.width;
	global_map_.map.info.height = map->info.height;
	global_map_.map.info.resolution = map->info.resolution;

	global_map_.map.info.map_load_time = map->info.map_load_time;

	global_map_.map.info.origin.position.x = map->info.origin.position.x;
	global_map_.map.info.origin.position.y = map->info.origin.position.y;
	global_map_.map.info.origin.position.z = map->info.origin.position.z;

	global_map_.map.info.origin.orientation.x = map->info.origin.orientation.x;
	global_map_.map.info.origin.orientation.y = map->info.origin.orientation.y;
	global_map_.map.info.origin.orientation.z = map->info.origin.orientation.z;
	global_map_.map.info.origin.orientation.w = map->info.origin.orientation.w;

	global_map_.map.data.resize(map->info.width*map->info.height);

	// todo local_map frontier leave empty for dubins curves
	/**
     * local_map_ ： 100 --> OBSTACLE/UNKNOWN
     *                0 --> FREE
     *
     */
	for(int i=0; i<map->data.size(); i++) {
		if(map->data[i] == -1) {
			global_map_.map.data[i] = map->data[i];
		} else if(map->data[i] <= g_obstacle_threshold) {
			global_map_.map.data[i] = 0;
		} else {
			global_map_.map.data[i] = 100;
		}
	}

	global_map_.map.header.stamp = map->header.stamp;
	global_map_.map.header.frame_id = map->header.frame_id;

	got_global_map = true;
}


bool localMapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
	boost::mutex::scoped_lock map_lock (local_map_mutex);
	if(got_local_map && local_map_.map.info.width && local_map_.map.info.height)
	{
		res = local_map_;
		return true;
	}
	else
		return false;
}

bool globalMapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
	boost::mutex::scoped_lock map_lock (global_map_mutex);
	if(got_global_map && global_map_.map.info.width && global_map_.map.info.height)
	{
		res = global_map_;
		return true;
	}
	else
		return false;
}


int main(int argc, char **argv)
{
	init(argc, argv, "map_server");
	NodeHandle n;
    ros::NodeHandle private_nh_("~");
    int obstacle_value = 0;
    private_nh_.param<int>("LETHAL_OBSTACLE", obstacle_value, 80);
    g_obstacle_threshold = obstacle_value;
	Subscriber local_sub = n.subscribe("/local_map", 10, localMapUpdate);
	Subscriber global_sub = n.subscribe("/global_map", 10, globalMapUpdate);

	ServiceServer local_map_srv = n.advertiseService("local_map", localMapCallback);
	ServiceServer global_map_srv = n.advertiseService("global_map", globalMapCallback);


	spin();

	return 0;
}
