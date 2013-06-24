
#include "tfhelper.h"
#include <geometry_msgs/PointStamped.h>
#include <tf/LinearMath/Quaternion.h>

void TFHelper::setListener(tf::TransformListener *listener) {
	this->listener = listener;
}

tf::Vector3 TFHelper::transformPoint(tf::Vector3 pt, std::string fixedFrame, std::string targetFrame) {
	// setup point message
	geometry_msgs::PointStamped ptStamped;
	ptStamped.header.frame_id = fixedFrame;
	ptStamped.header.stamp = ros::Time();
	tf::pointTFToMsg(pt, ptStamped.point);

	// transform
	geometry_msgs::PointStamped resultMsg;
	listener->transformPoint(targetFrame, ptStamped, resultMsg);
	
	// convert back
	tf::Stamped<tf::Point> resultPt;
	tf::pointStampedMsgToTF(resultMsg, resultPt);
	return resultPt;
}

tf::Pose TFHelper::transformPose(tf::Pose pose, std::string fixedFrame, std::string targetFrame) {
	// setup point message
	geometry_msgs::PoseStamped poseStamped;
	poseStamped.header.frame_id = fixedFrame;
	poseStamped.header.stamp = ros::Time();
	tf::poseTFToMsg(pose, poseStamped.pose);

	// transform
	geometry_msgs::PoseStamped resultMsg;
	bool found = false;
	while (!found) {
		try {
			listener->transformPose(targetFrame, poseStamped, resultMsg);
			found = true;
		} catch (tf::TransformException &ex) {
			ROS_INFO("%s", ex.what());
		}
	}
	
	
	// convert back
	tf::Stamped<tf::Pose> resultPose;
	tf::poseStampedMsgToTF(resultMsg, resultPose);
	return resultPose;
}

tf::Pose TFHelper::transformOrigin(std::string fixedFrame, std::string targetFrame) {
	tf::Pose origin;
	origin.setOrigin(tf::Vector3(0, 0, 0));
	origin.setRotation(tf::Quaternion(0, 0, 0, 1));
	return this->transformPose(origin, fixedFrame, targetFrame);
}

tf::Pose TFHelper::extractPose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
	tf::Stamped<tf::Pose> resultPose;
	tf::poseStampedMsgToTF(*msg, resultPose);
        std::cout<<"Goal Coordinates are: "<<resultPose.getOrigin().x()<<","<<resultPose.getOrigin().y()<<std::endl;
	return resultPose;
}
