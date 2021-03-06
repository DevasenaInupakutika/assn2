#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <vector>
#include "boost/thread/mutex.hpp"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include "controller.h"
#include "tfhelper.h"
#include "assn2/Point2d.h"
#include "assn2/Path.h"

using namespace std;

TFHelper *tfHelper;
TurtlebotController controller;

/* called when a new laser scan is received */
class kinectLaserScanReceiver{

private:

     ros::NodeHandle n;
     ros::Subscriber sub;
     
public:

     kinectLaserScanReceiver(ros::NodeHandle node):n(node){

             sub = n.subscribe("/scan",1,&kinectLaserScanReceiver::scanCallback,this);

}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	controller.updateScan(msg);	
}

};

class pathReceiver{

private:

     ros::NodeHandle n;
     ros::Subscriber sub;

public:
     pathReceiver(ros::NodeHandle node):n(node){
            sub = n.subscribe("/path",1,&pathReceiver::pathCallback,this);

}

void pathCallback(const assn2::Path::ConstPtr& msg) {
	controller.updatePathMsg(msg);	
	cout << "got a path with " << msg->length << " points!" << endl;
}

};

/* Called when receiving a new laser scan message */
class goalReceiver {

private:
       ros::NodeHandle n;
       ros::Subscriber sub;

public:
       goalReceiver(ros::NodeHandle node):n(node){
           sub = n.subscribe("/goal",1,&goalReceiver::goalCallback,this);
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
	tf::Pose goalPose = tfHelper->extractPose(msg);
	controller.setGoal(goalPose);

}

};

int main(int argc, char** argv) {
	// initialize ros
	ros::init(argc, argv, "final_node");
	ros::NodeHandle n;
	tf::TransformListener listener;
	
	// collect transforms for a while
	listener.waitForTransform("/base_link", "/map", ros::Time(), ros::Duration(1.5));
	TFHelper helper;	
	helper.setListener(&listener);
	controller.setHelper(&helper);
	tfHelper = &helper;
	
        kinectLaserScanReceiver laser(n);
        ROS_INFO("Laser Scan Receiver created");

        goalReceiver goal(n);
        ROS_INFO("Goal Receiver created");

        pathReceiver path(n);
        ROS_INFO("Path Receiver created");
	
	/* subscribe to laser scans */
	//ros::Subscriber scanSub = n.subscribe("scan", 1, scanCallback);
	/* subscribe to rviz goals */
	//ros::Subscriber goalSub = n.subscribe("goal", 1, goalCallback);
	/* subscribe to global paths */
	//ros::Subscriber pathSub = n.subscribe("path", 1, pathCallback);
	
        ros::Publisher markerPub = n.advertise<visualization_msgs::Marker>("controller/trajectory_rollout", 1);
	ros::Publisher trajectoryPub = n.advertise<visualization_msgs::Marker>("controller/best_trajectory", 1);
	ros::Publisher pathPub = n.advertise<visualization_msgs::Marker>("controller/path", 1);
	ros::Publisher localPub = n.advertise<visualization_msgs::Marker>("controller/local_goal", 1);
	ros::Publisher localPathPub = n.advertise<visualization_msgs::Marker>("controller/local_path", 1);
	ros::Publisher goalPub = n.advertise<visualization_msgs::Marker>("controller/goal_pos", 1);
	ros::Publisher cmdPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        geometry_msgs::Twist vel;

        //Initial Plan.
        //vel.linear.x = -0.3;
        //vel.angular.z = 0.0;
        //Setting Initial Trivial Goals.
	controller.setGoal(helper.transformOrigin("base_link","map"));
        controller.updateRobotState();

	ros::Rate loop_rate(10);

	while (ros::ok()) {
		// update messages
		ros::spinOnce();

		// get next control message
		RobotCmd cmd = controller.nextTimestep();

		vel.linear.x = ((-1) * cmd.first);
		vel.angular.z = cmd.second;
		cmdPub.publish(vel);
		controller.setCmd(cmd);
		cout << ((-1) * cmd.first) << ", " << cmd.second << endl;
		// update visualization
		markerPub.publish(controller.getTrajectories());
		trajectoryPub.publish(controller.getBestTrajectory());
		pathPub.publish(controller.getPath());
		localPub.publish(controller.getLocalGoal());
		localPathPub.publish(controller.getLocalPath());
		goalPub.publish(controller.getGoalViz());

		loop_rate.sleep();
	}
};
