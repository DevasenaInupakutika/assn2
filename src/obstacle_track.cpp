#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include "boost/thread/mutex.hpp"

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>

//decay rate - positive integer - lower is faster decay, higher is slower
#define DECAY_RATE 15

using namespace std;

/*
 * Obstacle tracker that outputs a map
 */

// current robot laser pose, relative to /map
tf::StampedTransform laserTrans;
tf::StampedTransform baseLinkTrans;

// current gates map
nav_msgs::OccupancyGrid gates_map;
bool obtained_gates_map = false;

// our current representation of the map plus obstacles
nav_msgs::OccupancyGrid obstacle_map;

// publisher for the occupancy map
ros::Publisher map_pub;

//mutex locks for the gates map and robot pose
boost::mutex gates_map_lock;
boost::mutex pose_lock;

//test publisher/other crap
ros::Publisher test_pub;

void drawTestCircle(double x, double y, double z){

        /////// TEST STUFF ///////////////////////////////

        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        //marker.header.frame_id = "/laser";
        marker.header.stamp = ros::Time::now();

        uint32_t shape = visualization_msgs::Marker::SPHERE;
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;

        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.0;

        marker.color.r = 1.0f;
        marker.color.g = 0.5f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5;
        marker.lifetime = ros::Duration();
        // publish the marker
        test_pub.publish(marker);

}

/*
 *  Helper method to decay previously found local obstacles
 */ 
void decayMap(){

    //loop through map values
    for(uint32_t i = 0; i < gates_map.info.width*gates_map.info.height; i++){

        //if this value isn't equal to the gates map value
        if(gates_map.data[i] != obstacle_map.data[i]){

            int diff = gates_map.data[i] - obstacle_map.data[i];
            int todecay = diff/DECAY_RATE;
            if(abs(todecay) < 1) todecay = 1;

            obstacle_map.data[i] = obstacle_map.data[i] + todecay;

        }
    }

}

/* Called when receiving a new map broadcast message */
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){

    //ROS_INFO("omg received a map!");
    //float w = msg->info.width;
    //ROS_INFO("%f", w);
    //map_pub.publish(*msg);

    boost::mutex::scoped_lock lock(gates_map_lock);
    gates_map = *msg;
    
    //deal with getting it for the first time
    if(!obtained_gates_map){
        obtained_gates_map = true;
        obstacle_map = gates_map;
    }

} 

/* Called when receiving a new laser scan message */
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    //if we haven't gotten the gates map yet, return
    if(!obtained_gates_map) return;

	// convenience vars
	int numPts = msg->ranges.size();
	//int centerIdx = numPts / 2;

    //get robot's position in map frame
    tf::Vector3 bl_origin = tf::Vector3(0.0, 0.0, 0.0);
    tf::Vector3 robotBase = baseLinkTrans * bl_origin;
    int base_x = floor(robotBase.getX() / obstacle_map.info.resolution  + 0.5);
    int base_y = floor(robotBase.getY() / obstacle_map.info.resolution  + 0.5);

    //remember which vars we mark as occupied this iteration
    set<uint32_t> occupied; 
	
	// loop through scan range
	for (int i = 0; i < numPts; i++) {
	//for (int i = centerIdx; i < centerIdx + 1; i++) {

		double distance = msg->ranges[i]; 
		double angle = msg->angle_min + i * msg->angle_increment;
		// bounds check
		if (distance < msg->range_min || distance > msg->range_max) {
            //ROS_INFO("distance = %f", distance);
			continue;
		}
		
		// get coordinates of point in laser frame
		double x = distance * cos(angle);	
        double y = distance * sin(angle);   
        double z = 0.24;
        tf::Vector3 laser_point = tf::Vector3(x,y,z);

        //convert the point to the map frame
        tf::Vector3 map_point = laserTrans * laser_point; 

        //drawTestCircle(map_point.getX(), map_point.getY(), map_point.getZ());
        //drawTestCircle(robotBase.getX(), robotBase.getY(), robotBase.getZ());

        //figure out what the closest pixel in the map is
        int obs_x = floor(map_point.getX() / obstacle_map.info.resolution  + 0.5);
        int obs_y = floor(map_point.getY() / obstacle_map.info.resolution  + 0.5);

        //mark all pixels in the map between the obstacle and the robot as free
        //by taking hops 1 unit long along the slope between the robot and point
        double y_off = 1.0*(obs_y - base_y);
        double x_off = 1.0*(obs_x - base_x);
        double slope_angle = 0.0;
        if(x_off != 0.0) slope_angle = atan2(y_off, x_off);
        int numPointsBetween = floor(sqrt(y_off*y_off + x_off*x_off));

        for(int j = 1; j < numPointsBetween; j++){
            int tofree_x = base_x + floor(0.5 + j*cos(slope_angle));
            int tofree_y = base_y + floor(0.5 + j*sin(slope_angle));
            uint32_t pix_num_tofree = obstacle_map.info.width * tofree_y + tofree_x;
            //bound check
            if(pix_num_tofree < obstacle_map.info.width * obstacle_map.info.height){

                //also, only white out this pixel if we didn't just black it out this iteration
                if(occupied.count(pix_num_tofree) == 0){
                    obstacle_map.data[pix_num_tofree] = 0;
                }
            }
        }

        //mark the obstacle in the corresponding pixel in the map
        uint32_t pixel_to_mark = obstacle_map.info.width * obs_y + obs_x;
        if(pixel_to_mark < obstacle_map.info.width * obstacle_map.info.height){
            obstacle_map.data[pixel_to_mark] = 100;
            occupied.insert(pixel_to_mark);
        }
        /*
        else{
            //ROS_INFO("OMG pixel to mark is greater than it should be!");
            //ROS_INFO("point = (%f %f), pixel = (%d %d)", map_point.getX(), map_point.getY(), obs_x, obs_y);
        }
        */
        
    }

    //publish our completed map
    map_pub.publish(obstacle_map);
}

int main(int argc, char** argv) {
	// initialize ros
	ros::init(argc, argv, "obstacle_track");
	ros::NodeHandle n;
	
	// subscribe to laser scans and the gates map
	ros::Subscriber scanSub = n.subscribe("scan", 1000, scanCallback);
    ros::Subscriber mapSub = n.subscribe("map", 10, mapCallback); 
	
	// setup and collect initial transform before we start
	tf::TransformListener listener;
	listener.waitForTransform("/laser", "/map", ros::Time(), ros::Duration(1.5));

    // advertise the published map
    map_pub = n.advertise<nav_msgs::OccupancyGrid>("/obstacle_map", 1);

    //advertise test publisher
    test_pub = n.advertise<visualization_msgs::Marker>("test_circle", 1000);
	
    // initalize laser to map transform
    /*
	try {
		listener.lookupTransform("/laser", "/map", ros::Time(), laserTrans);
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}
    */
	
	// set frequency
	ros::Rate loop_rate(5);

	while (ros::ok()) {

		    // update laserTrans
		    try {
		        listener.lookupTransform("/map", "/laser", ros::Time(), laserTrans);
		        listener.lookupTransform("/map", "/base_link", ros::Time(), baseLinkTrans);
                //ROS_INFO("found a transform!");
		    } catch (tf::TransformException ex) {
			    ROS_ERROR("%s",ex.what());
		    }

        // decay the map
        decayMap();

		// check for laser scans and gates map updates
		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
};

// old stuff

        //ROS_INFO("x and y and z  in laser frame = %f %f %f",x, y, z);
        //ROS_INFO("used a transform with frames of %s %s with time %d", laserTrans.frame_id_.c_str(), laserTrans.child_frame_id_.c_str(), laserTrans.stamp_.sec);
        //ROS_INFO("x and y and z in map frame = %f %f %f", map_point.getX(), map_point.getY(), map_point.getZ()); 
        //ROS_INFO("map width + height = %d %d %f", gates_map.info.width, gates_map.info.height, gates_map.info.resolution);
