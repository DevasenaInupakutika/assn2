#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include "boost/thread/mutex.hpp"
#include <LinearMath/btMatrix3x3.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>


using namespace std;

// current robot pose, relative to /odom
tf::StampedTransform robotPose;

// current goal pose
// TODO: clean up these types. why are they different??
// RESPONSE: one of them is a pose and one of them is a transform
// you could replace tf::Stamped<tf::Pose> with StampedPose, which
// I think is just a typedef
tf::Stamped<tf::Pose> goalPose;

// current linear/angular velocity of the robot
// these values are used to send out the cmd_vel
// messages to the robot each iteration
double linear = -0.3;
double angular = 0;

// true if the robot's forward path is blocked
bool pathBlocked = false;

// used to get the robot unstuck
int itcount = 0;

/* Called when receiving a new laser scan message */
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	// convenience vars
	int numPts = msg->ranges.size();
	//int centerIdx = numPts / 2;
	
	// check to see if any point blocks the robot
	// i.e. is the point within a rectangular window
	
	double robotRadius = 0.16; // meters
	double cutoffDist = 1.0; // meters (subtract about 0.08 to get true distance)
	
	double cutoffAngle = atan2(robotRadius, cutoffDist);
	bool foundAny = false;
	bool blocked = false;
	double blockAngle = 0;
	
	// loop through scan range
	for (int i = 0; i < numPts; i++) {
		double distance = msg->ranges[i]-0.08; 
		double angle = msg->angle_min + i * msg->angle_increment;
		// bounds check
		if (distance < msg->range_min || distance > msg->range_max) {
			continue;
		}
		foundAny = true;		
		
		// x-coordinate of point
		double forward = distance * cos(angle);		
		if (abs(angle) > cutoffAngle) {
			double lCutoff = abs(robotRadius / sin(angle));
			if (distance < lCutoff) {
				cout << "blocked at angle: " << angle << endl;
				blocked = true;
				blockAngle = angle;
			}
		} else if (forward < cutoffDist) {
			cout << "forward too small: " << angle << endl;
			blocked = true;
			blockAngle = angle;
		}
	}
	
	// TODO: move this to the controller code
	// update control appropriately
	if (foundAny && blocked) {
		linear = 0;
		itcount = 0;
		pathBlocked = true;
		
		// rotate away from obstacle
		if (blockAngle >= 0) angular = -0.5;
		else angular = 0.5;
	} else if (pathBlocked) {
		// once unblocked, try moving forward 8 times
		if (itcount < 8) {
			// move forward a little bit
			itcount++;
			linear = -0.3;
                        angular = 0.0;
		} else {
			// resume normal control
			pathBlocked = false;
		}
	}
}

/* Called when receiving a new laser scan message */
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
	cout << "got a new goal" << endl;
	// update goalPose
	tf::poseStampedMsgToTF(*msg, goalPose);
}

int main(int argc, char** argv) {
	// initialize ros
	ros::init(argc, argv, "turtlebot_controller");
	ros::NodeHandle n;
	
	/* subscribe to laser scans */
	ros::Subscriber scanSub = n.subscribe("scan", 1, scanCallback);
	/* subscribe to rviz goals */
	ros::Subscriber goalSub = n.subscribe("goal", 1, goalCallback);
	/* publish cmd velocities */
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);	
	
	/* transform listener */
	tf::TransformListener listener;
	
	// collect transforms for a while
	listener.waitForTransform("/odom", "/base_link", ros::Time(), ros::Duration(1.5));
	
	// set initial goal to be the robot's pose
	try {
		// lookup current robot pose wrt odom
		listener.lookupTransform("/odom", "/base_link", ros::Time(), robotPose);
		goalPose.setRotation(robotPose.getRotation());
		goalPose.setOrigin(robotPose.getOrigin());
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}
	
	// publish frequency
	ros::Rate loop_rate(30);
	
	while (ros::ok()) {
		try {
			// update robotPose
			listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), robotPose);
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
		}

		// check for laser scans
		ros::spinOnce();

		// Controller, version 0
		// Navigates in a series of phases:
		// (0) If blocked, get unblocked
		// (1) If misaligned, rotate towards goal
		// (2) If facing goal, move towards goal
		// (3) If at goal position, rotate into final orientation
		/* --------------------------------------------------------- */
		// convert goal pose to a message type
		geometry_msgs::PoseStamped goalPoseMsg;
		tf::poseStampedTFToMsg(goalPose, goalPoseMsg);
		goalPoseMsg.header.frame_id = "/odom";
		goalPoseMsg.header.stamp = ros::Time();

               	// get goal in base_link frame
		geometry_msgs::PoseStamped relativePoseMsg;
		listener.transformPose("/base_footprint", goalPoseMsg, relativePoseMsg);
		
		// extract goal pose from the message
		tf::Stamped<tf::Pose> relativePose;
		tf::poseStampedMsgToTF(relativePoseMsg, relativePose);
		
		// get the RPY and offset of the current goal
		// (relative to robot's current position)
		double roll, pitch, yaw;
		tf::Matrix3x3(relativePose.getRotation()).getRPY(roll, pitch, yaw);
		tf::Vector3 offset = relativePose.getOrigin();
		
		// compute the 2-D angle from robot position to goal position
		double angle = atan2(relativePose.getOrigin().getY(), relativePose.getOrigin().getX());
	
		// if blocked, appropriate velocities are already set
		if (!pathBlocked) {
			// far away from goal?
			if (sqrt(offset.getX()*offset.getX() + offset.getY()*offset.getY()) > 0.05) {
				// not lined up?
				if (abs(angle) > 0.3) {
					cout << "rotating towards target\n";
					if (abs(angle) > 0.05) {
						if (angle > 0) {
							angular = 1;
						} else {
							angular = -1;
						}
					}
					linear = 0;
				} else {
					cout << "moving toward target" << endl;
					if (offset.getX() > 0.05) {
						linear = -0.3;
					} else {
						linear = 0.3;
					}
					// adjust course?
					if (abs(angle) > 0.05) {
						cout << "adjusting course\n" << endl;
						if (angle > 0) {
							angular = 0.2;
						} else {
							angular = -0.2;
						}
					}
				}
			} else {
				// rotate into place?
				if (abs(yaw) > 0.1) {
					cout << "yaw into place\n" << endl;
					if (yaw > 0) {
						angular = 0.8;
					} else {
						angular = -0.8;
					}
				} else {
					cout << "done!" << endl;
					angular = 0;
				}
				linear = 0;
			}
		}
		/* ---------------------------------------------------- */

		// send out a new control message
                double x = robotPose.getOrigin().x();
                double y = robotPose.getOrigin().y();

                double gx = goalPose.getOrigin().x();
                double gy = goalPose.getOrigin().y();

                cout<< "Robot is at position: "<< "( "<< x << "," << y <<") "<< endl;
                cout<< "Goal coordinates are: "<<"( "<< gx << "," << gy <<") "<< endl;

                geometry_msgs::Twist vel;
		vel.linear.x = linear;
		vel.angular.z = angular;
		pub.publish(vel);

                loop_rate.sleep();
	}
	return 0;
};
