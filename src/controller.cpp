#include "controller.h"
#include <iostream>
#include <cmath>
#include "assn2/Point2d.h"
#include "assn2/Path.h"

#include <geometry_msgs/Point.h>

/*
#include <fl/fuzzylite.h>
#include <fl/Engine.h>
#include <fl/variable/OutputVariable.h>
#include <fl/variable/InputVariable.h>
#include <fl/rule/mamdani/MamdaniRule.h>
#include <fl/rule/RuleBlock.h>
#include <fl/term/Trapezoid.h>
#include <fl/term/Rectangle.h> */

#define PI 3.1415926
#define FIXED_FRAME "/map"
#define NOT_ADMISSIBLE -10000
#define EPSILON 1e-6

/* ---| math | --- */

double absf(double val) {
	return val < 0 ? -val : val;
}

double minf(double a, double b) {
	return a < b ? a : b;
}

double maxf(double a, double b) {
	return a < b ? b : a;
}

double sgnf(double a) {
	return a < 0 ? -1 : 1;
}

double getHeading(tf::Vector3 direction) {
	return atan2(direction.getY(), direction.getX());
}

double pointLineDistance(tf::Vector3 pt, tf::Vector3 a, tf::Vector3 b) {
	tf::Vector3 diff = (b - a);
	if (diff.length() < EPSILON) {
		return (pt - a).length();
	}

	tf::Vector3 dir = diff.normalized();
	double sc = dir.getX() * (pt.getY() - a.getY()) - dir.getY() * (pt.getX() - a.getX());
	return absf(sc);
}

/* returns a value in (-PI, PI] */
double headingDifference(double fromHeading, double toHeading) {
	double diff = toHeading - fromHeading;
	while (diff <= -PI) {
		diff += 2 * PI;
	}
	while (diff > PI) {
		diff -= 2 * PI;
	}
	return diff;
}

/**
 * Constructs a new controller.
 */
TurtlebotController::TurtlebotController() {
	vMin = 0.0;
	vMax = 0.3;
	vAccel = 2;
	vStepSize = 0.05;
	
	wMin = -0.3;
	wMax = 0.3;
	wAccel = 2;
	wStepSize = 0.2;
	rotateInPlaceThresh = 0.4;
	
	rolloutTime = 1.0; // seconds
	targetTime = 1.0; // seconds
	robotRadius = 0.16; // meters
	
	obstacleDeviation = 2.0;
	resolution = 0.05; // meters
	horizon = 1.0;
	
	potentialWeight = 4;
	headingWeight = 0.2;
	pathDistanceWeight = 5;//0.2;//0.3;//50;
	velocityWeight = 0.1;
	obstacleWeight = 0.5;//0.5;// 0.5;
	
	escaping = false;
	collisionMargin = 0.07;
	goalThreshold = 0.08;
	robot.v = 0;
	robot.w = 0;
	decayHalfLife = 0.15; // meters
       
        clear_path = false;
        count = 0;

        //Fuzzy Variables.
        
        /*fl::Engine* engine = new fl::Engine("Control");
        fl::InputVariable*  depth_avg = new fl::InputVariable;
        fl::OutputVariable* fangular = new fl::OutputVariable;
        fl::OutputVariable* flinear = new fl::OutputVariable;
        fl::RuleBlock* ruleblock1=new fl::RuleBlock;	
        fl::RuleBlock* ruleblock2=new fl::RuleBlock;
        
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


        ruleblock1->addRule(fl::MamdaniRule::parse("if DepthAverage is NEAR then AngularVelocity is HIGH", engine));
        ruleblock1->addRule(fl::MamdaniRule::parse("if DepthAverage is FAR then AngularVelocity is LOW", engine));
        engine->addRuleBlock(ruleblock1);

        ruleblock2->addRule(fl::MamdaniRule::parse("if DepthAverage is NEAR then LinearVelocity is SLOW", engine));
        ruleblock2->addRule(fl::MamdaniRule::parse("if DepthAverage is FAR then LinearVelocity is FAST", engine));
        engine->addRuleBlock(ruleblock2);

        engine->configure("Minimum", "Maximum", "AlgebraicProduct", "AlgebraicSum", "Centroid"); */

}

/* Uses the dynamic window method to return the next control message */
RobotCmd TurtlebotController::nextTimestep() {
	boost::mutex::scoped_lock lock(update_mutex);
	// first, update robot pose, etc.
	updateRobotState();
	updateLocalGoal();
	
	// at goal?
	double goalDist = (goalPose.getOrigin() - robot.position).length();

        std::cout<<"Distance to Goal is:"<<goalDist<<std::endl;

	if (goalDist < goalThreshold) {
		std::cout << "at goal!" << std::endl;
		return RobotCmd(0, 0);
	}
	
	double timestep = maxf(0.1,minf(targetTime, goalDist / vMax));
	double maxScore = -1e6;
	optV = 0;
	optW = 0;
	
	double vWindowMin = maxf(vMin, robot.v - vAccel);
	double vWindowMax = minf(vMax, robot.v + vAccel);
	double wWindowMin = maxf(wMin, robot.w - wAccel);
	double wWindowMax = minf(wMax, robot.w + wAccel);
	double v = vWindowMin;
	while (v <= vWindowMax) {
		double w = wWindowMin;
		while (w <= wWindowMax) {
			// optimize
			double score = evaluate(v, w, timestep);
                        if ((absf(v) > EPSILON || absf(w) >= rotateInPlaceThresh) && score > maxScore) {
				maxScore = score;
				optV = v;
				optW = w;
			}
			w += wStepSize;
		}
		v += vStepSize;
	}
       
	if (absf(optV) < EPSILON) {
		if (!escaping) {
			escaping = true;
			escapeSgn = optW >= 0 ? 1 : -1;
		}
		optW = wMax * escapeSgn;
	} else {
		escaping = false;
	}
	
	return RobotCmd(optV, optW);
}	

void TurtlebotController::updateLocalGoal() {
	localGoalPts.clear();
	double goalDist = (goalPose.getOrigin() - robot.position).length();
	double curHorizon = minf(horizon, goalDist);
	
	unsigned int idx = robot.pathIdx;
	//int ptCount = 0;
	double deviation = 0;
	bool isUp = false;

        	
	// extend to the horizon
	while ((pathPts[idx] - robot.position).length() <= curHorizon && idx < pathPts.size()) {
		if (idx > 0) {
			double heading = getHeading(pathPts[idx] - pathPts[idx - 1]);
			tf::Vector3 perp = unitVector(heading + PI / 2);
		
			double deviation1 = 0;
			tf::Vector3 position1 = pathPts[idx];
			double feasible1 = false;
			while (deviation1 <= obstacleDeviation) {
				if (lineOfSight(robot.position, position1)) {
					feasible1 = true;
					break;
				}
				deviation1 += resolution;
				position1 += perp * resolution;
			}
			
			double deviation2 = 0;
			tf::Vector3 position2 = pathPts[idx];
			bool feasible2 = false;
			while (deviation2 <= obstacleDeviation) {
				if (lineOfSight(robot.position, position2)) {
					feasible2 = true;
					break;
				}
				deviation2 += resolution;
				position2 -= perp * resolution;
			}
			
			
			if (!goingDown && feasible1 && deviation1 <= deviation2) {
				localGoalPts.push_back(position1);
				deviation = deviation1;
				isUp = true;
			} else if (!goingUp && feasible2 && deviation2 <= deviation1) {
				localGoalPts.push_back(position2);
				deviation = deviation2;
				isUp = false;
			}
		}
		idx++;
	}
	if (localGoalPts.size() > 0) {
		tf::Vector3 localGoal = localGoalPts[localGoalPts.size() - 1];
		double goalHeading = getHeading(localGoal - robot.position);
		robot.headingError = headingDifference(robot.heading, goalHeading);
		robot.navPotential = (localGoal - robot.position).length();
	} else {
		robot.headingError = 0;
	}
	
	if (goingUp || goingDown) {
		followCount++;
	} else if (deviation > EPSILON) {
		goingUp = isUp;
		goingDown = !isUp;
	}
	if (followCount >= 8) {
		followCount = 0;
		goingUp = false;
		goingDown = false;
	}
}

bool TurtlebotController::lineOfSight(tf::Vector3 pt1, tf::Vector3 pt2) {
	tf::Vector3 curPt = pt1;
	tf::Vector3 diff = pt2 - pt1;
	
	// same point?
	if (diff.length() < EPSILON) {
		return true;
	}
	
	tf::Vector3 dir = diff.normalized();
	while ((pt2 - curPt).length() > resolution) {
		double dist = getDistanceToObstacle(curPt);
		if (dist <= robotRadius + collisionMargin) {
			return false;
		}
		curPt += dir * resolution;
	}
	return true;
}

double TurtlebotController::getDistanceToObstacle(tf::Vector3 position) {
	double minDist = horizon;
	for (unsigned int i = 0; i < scanPts.size(); i++) {
	double dist = (scanPts[i] - position).length();
		if (dist < minDist) {
			minDist = dist;
		}
	}
        return minDist;
}

/* evaluates a (linear,angular) pair for the next timestep. */
double TurtlebotController::evaluate(double v, double w, double time) {
	std::vector<double> features;
	std::vector<double> weights;

	RobotState state = rollout(v, w, time);
	double pathChange = robot.pathError - state.pathError;
	double headingChange = absf(robot.headingError) - absf(state.headingError);
	double potentialChange = robot.navPotential - state.navPotential;
	double obstacleScore = -pow(2, -1.0/decayHalfLife * (state.obstacleDistance + 0.03));
	double velocity = state.v;
	
	//std::cout << "(" << v << ", " << w << ")  " << obstacleDistance << std::endl;
	
	if (state.obstacleDistance <= 0) {
		return NOT_ADMISSIBLE;
	}
	
	/* --------- scoring function ------------- */
	features.push_back(pathChange);
	weights.push_back(pathDistanceWeight);
	
	features.push_back(headingChange);
	weights.push_back(headingWeight);
	
	features.push_back(potentialChange);
	weights.push_back(potentialWeight);
	
	features.push_back(velocity);
	weights.push_back(velocityWeight);
	
	features.push_back(obstacleScore);
	weights.push_back(obstacleWeight);
	
	/* ---------------------------------------- */
	
	// return weighted score
	double score = 0;
	for (unsigned int i = 0; i < features.size(); i++) {
		score += features[i] * weights[i];
	}
	//std::cout << "(" << v << ", " << w << ") " << "s: " << score << ", o: " << obstacleScore << ", h: " << headingChange << ", p:" << pathChange << ", g: " << potentialChange << std::endl;
	

	return score;
}

RobotState TurtlebotController::rollout(double v, double w, double time) {
	RobotState state;
	state.v = v;
	state.w = w;
	state.obstacleDistance = getObstacleDistance(v, w, time) - robotRadius - collisionMargin;

        //Fuzzy Variables.
        
        /*fl::Engine* engine = new fl::Engine("Control");
        fl::InputVariable*  depth_avg = new fl::InputVariable;
        fl::OutputVariable* fangular = new fl::OutputVariable;
        fl::OutputVariable* flinear = new fl::OutputVariable;
        
         if (state.obstacleDistance == 0.77)
        {
            float ang_out = 0.0;
            float lin_out = 0.0;
            fl::scalar in;
            in = state.obstacleDistance;
            depth_avg->setInput(in);
            engine->process();
            float out1 = fangular->defuzzify();
            float out2 = flinear->defuzzify();
            out2 = 0.0;
            out1 = w;
            lin_out = ((-1)*out2);
            ang_out = out1;
            
            state.v = lin_out;
            state.w = ang_out;
    
         }*/
       // std::cout<<"Distance from obstacle is: "<<state.obstacleDistance <<" and velocities are: "<< state.v << ", "<<state.w<<std::endl;

        /*if (state.obstacleDistance > 0.8){
        clear_path = true;
        count++;
        current_time = ros::Time::now();
        state.v = -0.3;
        state.w = 0.0;
        }*/

        if (state.obstacleDistance <= 0) {
                return state;
        } 
        
        /*if ((state.obstacleDistance <= 0.8) && (state.obstacleDistance >=0 )){
         clear_path = false;
         count = 0;
         state.v = 0.3;
         state.w = 0.0;
         
        }
  
        if(count == 1)
          ref_time = current_time;
        if((count >1) && (current_time - ref_time).toSec() > 2.0)
          clear_path = true;*/

              
  
	state.position = getTrajectoryPosition(v, w, time); // good
	state.heading = getTrajectoryHeading(v, w, time); //TODO param?
	state.pathIdx = getNearestPathPoint(state.position); // good
	
	if (localGoalPts.size() > 0) {
		tf::Vector3 localGoal = localGoalPts[localGoalPts.size() - 1];
		double goalHeading = getHeading(localGoal - state.position);
		state.headingError = headingDifference(state.heading, goalHeading);	
		state.pathError = pointLineDistance(state.position, robot.position, localGoal);
		state.navPotential = (localGoal - state.position).length();//getPathPotential(state.pathIdx);
	} else {
		state.headingError = 0;
		state.pathError = 0;
		state.navPotential = 0;
	}
	return state;
}

void TurtlebotController::updateRobotState() {
	tf::Pose robotPose = tfHelper->transformOrigin("/base_link",FIXED_FRAME);
        robot.position = robotPose.getOrigin();
	robot.heading = tf::getYaw(robotPose.getRotation());
	robot.pathIdx = getNearestPathPoint(robot.position);
	robot.pathError = 0;
        std::cout<< "Current Robot Position"<<robotPose.getOrigin().x()<< ","<<robotPose.getOrigin().y()<<std::endl;
}

int TurtlebotController::getNearestPathPoint(tf::Vector3 position) {
	double minDist = 0;
	int ptIdx = 0;
	for (unsigned int i = 0; i < pathPts.size(); i++) {
		tf::Vector3 pathPt = pathPts[i];
		double dist = (pathPt - position).length();
		if (i == 0 || dist < minDist) {
			minDist = dist;
			ptIdx = i;
		}
	}
	return ptIdx;
}

// returns the heading to the "best" path point from the given position
// "best" means: closest to goal with direct line of sight
double TurtlebotController::getPathHeading(tf::Vector3 position) {
	for (int i = pathPts.size() - 1; i >= 0; i--) {
		tf::Vector3 diff = pathPts[i] - position;
		double heading = getHeading(diff);
		for (unsigned int j = 0; j < scanPts.size(); j++) {
			double dist = distanceToSegment(scanPts[j], position, pathPts[i]);
			if (dist < 0) {
				return heading;
			}
			if (dist > robotRadius) {
				return heading;
			}
		}
	}
	// don't have line of sight to any goal point
	return robot.heading;
}

double TurtlebotController::getPathPotential(int idx) {
	tf::Vector3 pathPt = pathPts[idx];
	
	// approximation: straight-line distance
	return (goalPose.getOrigin() - pathPt).length();
}


double TurtlebotController::getTrajectoryHeading(double v, double w, double time) {
	double start = robot.heading;
	double end = start + w * time;
	while (end > PI) end -= 2*PI;
	while (end <= -PI) end += 2*PI;
	return end;
}

/* returns the position of the robot after choosing (v,w) for one timestep */
tf::Vector3 TurtlebotController::getTrajectoryPosition(double v, double w, double time) {
	//case 1: straight line
	if (absf(w) < EPSILON) {
		return robot.position + (v * time) * unitVector(robot.heading);
	}
	// case 2: constant curvature
	double r = v / w;
	double theta = w * time;
	tf::Vector3 offsetDir = unitVector(robot.heading + theta / 2);
	double offsetDist = 2 * absf(r) * absf(sin(theta / 2));
	return robot.position + offsetDist * offsetDir;
}

double TurtlebotController::getObstacleDistance(double v, double w, double time) {
	int numSteps = 10;
	double distSum = 0;
	
	for (int i = 1; i <= numSteps; i++) {
		tf::Vector3 curPt = getTrajectoryPosition(v, w, time / numSteps * i);
		double dist = getDistanceToObstacle(curPt);
		if (dist < robotRadius + collisionMargin) {
			return 0;
		}
		distSum += dist;
	}
	
	return distSum / numSteps;
}



// -------| geometry |---------

std::vector<double> TurtlebotController::intersectCircles(double r1, tf::Vector3 diff, double r2) {
	std::vector<double> pts;
	double d = diff.length();
	
	if (d > r1 + r2 || d < r1 - r2) {
		return pts;
	}
	
	double z = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
	double theta = acos(z / r1);
	double yaw = atan2(diff.getY(), diff.getX());
	
	pts.push_back(yaw + theta);
	if (absf(theta) > EPSILON) {
		pts.push_back(yaw - theta);
	}
	
	return pts;
}

double TurtlebotController::distanceToSegment(tf::Vector3 point, tf::Vector3 p1, tf::Vector3 p2) {
	if ((p2 - p1).length() < EPSILON) {
		return (p2 - point).length();
	}
	tf::Vector3 d = (p2 - p1).normalized();
	double beta = d.getX() * (point.getY() - p1.getY()) + d.getY() * (p1.getX() - point.getX());
	double alpha = (point.getX() - p1.getX() + beta * d.getY()) / d.getX();
	if (alpha < 0 || alpha > (p2 - p1).length()) {
		return -1;
	}
	return absf(beta);
}


/* --------| callbacks |----------- */

/* sets a new goal pose */
void TurtlebotController::setGoal(tf::Pose goalPose) {	
	boost::mutex::scoped_lock lock(update_mutex);
	this->goalPose = goalPose;
	std::cout << "got a new goal!" << std::endl;
	initialPose = tfHelper->transformOrigin("/base_link", FIXED_FRAME);
        updatePath();
}

/* Called when receiving a new laser scan message */
void TurtlebotController::updateScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
	boost::mutex::scoped_lock lock(update_mutex);
	// clear current obstacles
	scanPts.clear();
	
	// convenience vars
	int numPts = msg->ranges.size();
	//int centerIdx = numPts / 2;
	
	// loop through scan range
	for (int i = 0; i < numPts; i++) {
		double distance = msg->ranges[i] - 0.08; 
		double angle = msg->angle_min + i * msg->angle_increment;
		
		// bounds check
		if (distance < msg->range_min || distance > msg->range_max) {
			continue;
		}	
		
		// generate local obstacle
		tf::Vector3 pt = distance * unitVector(angle);// + width * unitVector(angle + PI/2);
		
		// convert to fixed frame coordinates
		tf::Vector3 fixedPt = tfHelper->transformPoint(pt,"/camera_depth_frame",FIXED_FRAME);
                scanPts.push_back(fixedPt);
	}
}

void TurtlebotController::updatePath() {
	pathPts.clear();
	tf::Pose robotPose = tfHelper->transformOrigin("/base_link", FIXED_FRAME);
        tf::Vector3 curPos = robotPose.getOrigin();
	tf::Vector3 diff = goalPose.getOrigin() - curPos;
	while (diff.length() > 0.5) {
		std::cout << diff.length() << " from goal" << std::endl;
		pathPts.push_back(curPos);
		tf::Vector3 dir = diff.normalized();
		curPos += 0.5 * dir;
		diff = goalPose.getOrigin() - curPos;
	}
	// last one
	pathPts.push_back(curPos);
	std::cout << "path has " << pathPts.size() << " points" << std::endl;
}

void TurtlebotController::updatePathMsg(const assn2::Path::ConstPtr& msg) {
	boost::mutex::scoped_lock lock(update_mutex);
	pathPts.clear();
	for (unsigned int i = 0; i < msg->length; i++) {
		assn2::Point2d pt = msg->path[i];
		pathPts.push_back(tf::Vector3(pt.x, pt.y, 0));
	}
}


visualization_msgs::Marker TurtlebotController::getBestTrajectory() {
	boost::mutex::scoped_lock lock(update_mutex);
	visualization_msgs::Marker points;
	points.header.frame_id = FIXED_FRAME;
	
        points.header.stamp = ros::Time::now();
	points.type = visualization_msgs::Marker::LINE_LIST;
	points.scale.x = 0.02;
	points.scale.y = 0.02;
	points.color.r = 1.0f;
	points.color.a = 1.0;
	
	int numPts = 11;
	for (int i = 0; i <= numPts; i++) {
		tf::Vector3 pos = getTrajectoryPosition(optV, optW, rolloutTime / numPts * i);
		geometry_msgs::Point pt;
		pt.x = pos.getX();
		pt.y = pos.getY();
		pt.z = 0;
		points.points.push_back(pt);
		if (i > 0 && i < numPts) {
			points.points.push_back(pt);
		}
	}
	return points;
}

visualization_msgs::Marker TurtlebotController::getPath() {
	boost::mutex::scoped_lock lock(update_mutex);
	visualization_msgs::Marker points;
	points.header.frame_id = FIXED_FRAME;
	
        points.header.stamp = ros::Time::now();
	points.type = visualization_msgs::Marker::LINE_LIST;
	points.scale.x = 0.03;
	points.scale.y = 0.03;
	points.color.b = 1.0f;
	points.color.a = 1.0;
	
	for (unsigned int i = 0; i < pathPts.size(); i++) {
		geometry_msgs::Point pt;
		pt.x = pathPts[i].getX();
		pt.y = pathPts[i].getY();
		pt.z = 0;
		points.points.push_back(pt);
		if (i == 0 && pathPts.size() % 2 == 1) {
			points.points.push_back(pt);
		}
	}
	return points;
}

visualization_msgs::Marker TurtlebotController::getLocalGoal() {
	boost::mutex::scoped_lock lock(update_mutex);
	visualization_msgs::Marker points;
	points.header.frame_id = FIXED_FRAME;
	points.header.stamp = ros::Time::now();
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.03;
	points.scale.y = 0.03;
	points.color.r = 1.0f;
	points.color.b = 1.0f;
	points.color.a = 0.5;
	
	for (unsigned int i = 0; i < localGoalPts.size(); i++) {
		geometry_msgs::Point pt;
		pt.x = localGoalPts[i].getX();
		pt.y = localGoalPts[i].getY();
		pt.z = 0;
		points.points.push_back(pt);
	}
	return points;
}

visualization_msgs::Marker TurtlebotController::getLocalPath() {
	boost::mutex::scoped_lock lock(update_mutex);
	visualization_msgs::Marker points;
	points.header.frame_id = FIXED_FRAME;
	points.header.stamp = ros::Time::now();
	points.type = visualization_msgs::Marker::LINE_LIST;
	points.scale.x = 0.02;
	points.scale.y = 0.02;
	points.color.r = 1.0f;
	points.color.g = 0.3f;
	points.color.b = 1.0f;
	points.color.a = 0.5;
	
	
	geometry_msgs::Point pt;
	pt.x = robot.position.getX();
	pt.y = robot.position.getY();
	pt.z = 0;
	points.points.push_back(pt);
	
	if (localGoalPts.size() > 0) {
		pt.x = localGoalPts[localGoalPts.size()-1].getX();
		pt.y = localGoalPts[localGoalPts.size()-1].getY();
		pt.z = 0;
	}
	points.points.push_back(pt);
	return points;
}


visualization_msgs::Marker TurtlebotController::getGoalViz() {
	boost::mutex::scoped_lock lock(update_mutex);
	visualization_msgs::Marker points;
	points.header.frame_id = FIXED_FRAME;
	points.header.stamp = ros::Time::now();
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.08;
	points.scale.y = 0.08;
	points.color.b = 1.0f;
	points.color.a = 0.5;
	
	geometry_msgs::Point pt;
	pt.x = goalPose.getOrigin().getX();
	pt.y = goalPose.getOrigin().getY();
	pt.z = 0;
	points.points.push_back(pt);
	return points;
}

visualization_msgs::Marker TurtlebotController::getTrajectories() {
	boost::mutex::scoped_lock lock(update_mutex);
	visualization_msgs::Marker points;
	points.header.frame_id = FIXED_FRAME;
	points.header.stamp = ros::Time::now();
	points.type = visualization_msgs::Marker::LINE_LIST;
	points.scale.x = 0.01;
	points.scale.y = 0.01;
	points.color.g = 1.0f;
	points.color.a = 0.2;
	
	double vWindowMin = maxf(vMin, robot.v - vAccel);
	double vWindowMax = minf(vMax, robot.v + vAccel);
	double wWindowMin = maxf(wMin, robot.w - wAccel);
	double wWindowMax = minf(wMax, robot.w + wAccel);
	
	double v = vWindowMin;
	while (v <= vWindowMax) {
		double w = wWindowMin;
		while (w <= wWindowMax) {
			int numPts = 11;
			for (int i = 0; i <= numPts; i++) {
				tf::Vector3 pos = getTrajectoryPosition(v, w, rolloutTime / numPts * i);
				geometry_msgs::Point pt;
				pt.x = pos.getX();
				pt.y = pos.getY();
				pt.z = 0;
				points.points.push_back(pt);
				if (i > 0 && i < numPts) {
					points.points.push_back(pt);
				}
			}
			w += wStepSize;
		}
		v += vStepSize;
	}
	return points;
}


/* sets the TFHelper */
void TurtlebotController::setHelper(TFHelper *helper) {
	boost::mutex::scoped_lock lock(update_mutex);
	tfHelper = helper;
}

/* returns a unit vector pointing in the right direction */
tf::Vector3 TurtlebotController::unitVector(double yaw) {
	return tf::Vector3(cos(yaw), sin(yaw), 0);
}

void TurtlebotController::setCmd(RobotCmd cmd) {
	boost::mutex::scoped_lock lock(update_mutex);
	robot.v = cmd.first;
	robot.w = cmd.second;
}
