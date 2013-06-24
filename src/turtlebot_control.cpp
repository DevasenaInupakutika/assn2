#include <ros/ros.h>

#include <math.h>
#include <iostream>
#include "boost/thread/mutex.hpp"

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <fl/fuzzylite.h>
#include <fl/Engine.h>
#include <fl/variable/OutputVariable.h>
#include <fl/variable/InputVariable.h>
#include <fl/rule/mamdani/MamdaniRule.h>
#include <fl/rule/RuleBlock.h>
#include <fl/term/Trapezoid.h>
#include <fl/term/Rectangle.h>

using namespace std;

//Current linear/angular velocity of the robot
// these values are used to send out the cmd_vel
// messages to the robot each iteration
double linear = -0.3;
double angular = 0;

//Fuzzy Variables.
fl::Engine* engine = new fl::Engine("Control");
fl::InputVariable* depth_avg = new fl::InputVariable;
fl::OutputVariable* fangular = new fl::OutputVariable;
fl::OutputVariable* flinear = new fl::OutputVariable;

// true if the robot's forward path is blocked
bool pathBlocked = false;

// used to get the robot unstuck
int itcount = 0;

/* Called when receiving a new laser scan message */
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

	// convenience vars
	int numPts = msg->ranges.size();

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
		double distance = msg->ranges[i] - 0.08;
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

                        //Beginning of Fuzzy Inference System.

                  else if(distance >= 1.0)
                        {
                           float ang_out = 0.0;
                           float lin_out = 0.0;
                           fl::scalar in;
                           if(forward > distance)
                              in = distance;
                           else
                              in = forward;
                           depth_avg->setInput(in);
                           engine->process();
                           float out1 = fangular->defuzzify();
                           float out2 = flinear->defuzzify();
                           if(in < 0.8)
                              out2=0.0;
                           else if(in > 1.5)
                              out1=0.0;
                               lin_out=(-1*out2);
                               ang_out = out1;
                           linear = lin_out;
                           angular = ang_out;
                           ROS_INFO(" The robot linear and angular velocities are: %f %f ",linear,angular);
                         }

	}


	if (foundAny && blocked) {
		linear = 0;
		itcount = 0;
		pathBlocked = true;

		// rotate away from obstacle
		if (blockAngle >= 0) angular = -0.5;
		else angular = 0.5;
	} else if (pathBlocked) {
		// once unblocked, try moving forward 8 times.
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

int main(int argc, char** argv) {
	// initialize ros
	ros::init(argc, argv, "turtlebot_control");
	ros::NodeHandle n;

	/* subscribe to laser scans */
	ros::Subscriber scanSub = n.subscribe("scan", 1, scanCallback);

	/* publish cmd velocities */
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	//Publish frequency
	ros::Rate loop_rate(10);

        //Fuzzy Variables and Rules Definition.
        depth_avg->setName("DepthAverage");
        depth_avg->setRange(0.000,8.000);
        depth_avg->addTerm(new fl::Trapezoid("NEAR",0.000,0.500,0.800,1.500));
        depth_avg->addTerm(new fl::Trapezoid("FAR",0.800,1.500,7.000,8.000));
        engine->addInputVariable(depth_avg);

        fangular->setName("AngularVelocity");
        fangular->setRange(0.000,1.000);
        fangular->setDefaultValue(0);
        fangular->addTerm(new fl::Rectangle("LOW",0.000,0.200));
        fangular->addTerm(new fl::Rectangle("HIGH",0.600,1.000));
        engine->addOutputVariable(fangular);

        flinear->setName("LinearVelocity");
        flinear->setRange(0.000,0.300);
        flinear->setDefaultValue(0);
        flinear->addTerm(new fl::Rectangle("SLOW",0.000,0.100));
        flinear->addTerm(new fl::Rectangle("FAST",0.100,0.300));
        engine->addOutputVariable(flinear);

        fl::RuleBlock* ruleblock1=new fl::RuleBlock;
        ruleblock1->addRule(fl::MamdaniRule::parse("if DepthAverage is NEAR then AngularVelocity is HIGH", engine));
        ruleblock1->addRule(fl::MamdaniRule::parse("if DepthAverage is FAR then AngularVelocity is LOW", engine));
        engine->addRuleBlock(ruleblock1);

        fl::RuleBlock* ruleblock2=new fl::RuleBlock;
        ruleblock2->addRule(fl::MamdaniRule::parse("if DepthAverage is NEAR then LinearVelocity is SLOW", engine));
        ruleblock2->addRule(fl::MamdaniRule::parse("if DepthAverage is FAR then LinearVelocity is FAST", engine));
        engine->addRuleBlock(ruleblock2);

        engine->configure("Minimum", "Maximum", "AlgebraicProduct", "AlgebraicSum", "Centroid");

        while (ros::ok()) {

		// check for laser scans
		ros::spinOnce();

		// Controller, version 0
		// Navigates in below phase:
		// (0) If blocked, get unblocked

		/* --------------------------------------------------------- */

		// send out a new control message
                geometry_msgs::Twist vel;
        	vel.linear.x = linear;
		vel.angular.z = angular;
		pub.publish(vel);
		loop_rate.sleep();
	}
	return 0;
}
