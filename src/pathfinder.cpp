#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/OccupancyGrid.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "assn2/NavGrid.h"
#include "assn2/Point2d.h"
#include "assn2/Path.h"

using namespace std;

//basic point class. thanks evan!
class Point {

    public:
        double x;
        double y;
        Point() {}
        Point(double _x, double _y) {x=_x; y=_y;}
        //Point(double i,double x_dim, double y_dim) {x=i/y_dim; y=i%y_dim;}
        double dist(const Point &other_pt) const {
            return pow(pow(x - other_pt.x,2) +pow(y - other_pt.y,2),0.5);
        }
};

class IntPoint {

    public:
        int x;
        int y;
        IntPoint() {}
        IntPoint(int _x, int _y) {x=_x; y=_y;}
        IntPoint(int i,int x_dim, int y_dim) {x=i/y_dim; y=i%y_dim;}
        double dist(const Point &other_pt) const {
            return pow(pow(x - other_pt.x,2) +pow(y - other_pt.y,2),0.5);
        }
};

//global vars
ros::Publisher path_pub;
ros::Publisher marker_pub;
assn2::NavGrid nav_grid;
Point r_pos;
Point g_pos;
const double STEPSIZE = 0.8;

tf::Vector3 getCostPosVector(IntPoint vertex);
Point calcNormGrad(tf::Vector3 a, tf::Vector3 b, tf::Vector3 c);
Point calcSquareNormGrad(tf::Vector3 bl,tf::Vector3 br,tf::Vector3 tr,tf::Vector3 tl);

void test(){

    nav_grid.width = 30;
    nav_grid.height = 30;
    nav_grid.resolution = 1.0;
    nav_grid.goalX = 0;
    nav_grid.goalY = 0;
    g_pos.x = 1.1;
    g_pos.y = 1.1;
    r_pos.x = 2.2;
    r_pos.y = 25.2;
    vector<double> costs(900);
    for(int i = 0; i < 900; i++) costs[i] = i;
    costs[0] = 32;
    nav_grid.costs = costs;

}

/* update the robot's goal in map coordinates 
 * assumes that the goal pose is in the map frame 
 * but not in map coordinates
 * */
void updateGoal(){
   g_pos.x = nav_grid.goalX / nav_grid.resolution;
   g_pos.y = nav_grid.goalY / nav_grid.resolution;
}

/*
 *  Call back function for the nav grid
 */
void navFnCallback(const assn2::NavGrid::ConstPtr &msg) {

    //populate nav_grid
    ROS_INFO("populating nav grid....");
    nav_grid = *msg;
    updateGoal();
}


void initMarker(visualization_msgs::Marker &marker, string name, int type) {
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = name;
  marker.id = 0;
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = .01;
  marker.scale.y = .01;
  marker.scale.z = .01;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration(1000);
}

/*
 * This function visualizes the gradient at each center point
 *
 */
void publishGradField(){

    //uint32_t width = nav_grid.width;
    //uint32_t height = nav_grid.height;

    visualization_msgs::Marker marker;
    initMarker(marker, "grad_field", visualization_msgs::Marker::LINE_LIST);
    geometry_msgs::Point p;

    if (isinf(r_pos.x) || isnan(r_pos.x) || isinf(r_pos.y) || isnan(r_pos.y)) {
      return;
    }

    ROS_INFO("[publishGradField]\tx_min=%f,x_max=%f,y_min=%f,y_max=%f",r_pos.x - 10,r_pos.x + 10,r_pos.y - 10,r_pos.y + 10);
    for(uint32_t w = r_pos.x - 100; w < r_pos.x + 100; w ++){
        for(uint32_t h = r_pos.y - 100; h < r_pos.y + 100; h++){
        
            tf::Vector3 a = getCostPosVector(IntPoint(w,h));
            tf::Vector3 b = getCostPosVector(IntPoint(w+1,h));
            tf::Vector3 c = getCostPosVector(IntPoint(w,h+1));
            tf::Vector3 d = getCostPosVector(IntPoint(w+1,h+1));

            //Point grad = calcNormGrad(a, b, c);
            Point grad = calcSquareNormGrad(a,b,d,c);

            p.x = (w + 0.5)*nav_grid.resolution;
            p.y = (h + 0.5)*nav_grid.resolution;
            p.z = 0.0;

            marker.points.push_back(p);

            p.x = (w + 0.5 + grad.x)*nav_grid.resolution;
            p.y = (h + 0.5 + grad.y)*nav_grid.resolution;
            p.z = 0.0;
           
            marker.points.push_back(p);

        }
    }
    std::cout<<"[publishGradField]\tpublishing marker with " << marker.points.size()/2 << "lines.";
    marker_pub.publish(marker);

}


/*
 * This function publishes the path as a assn2::Path message
 * , which is in the normal map frame, not the map coordinates
 */ 
void publishPath(vector<Point> path, uint32_t length){

  
  visualization_msgs::Marker marker;
  //initMarker(marker,"goal_path",visualization_msgs::Marker::LINE_STRIP);
  initMarker(marker,"goal_path",visualization_msgs::Marker::POINTS);
  
  geometry_msgs::Point geo_pt; // for vizualization
  assn2::Path p;
  p.length = length;
  vector<assn2::Point2d> ppath(length);
  for(uint32_t i = 0; i < length; i ++){
    ppath[i].x = path[i].x * nav_grid.resolution;
    ppath[i].y = path[i].y * nav_grid.resolution;
    
    // visualization code
    geo_pt.x = ppath[i].x;
    geo_pt.y = ppath[i].y;
    geo_pt.z = 0.1;
    //ROS_INFO("x and y %f %f", geo_pt.x, geo_pt.y);
    marker.points.push_back(geo_pt);
  }
  p.path = ppath;
  
  marker_pub.publish(marker);
  path_pub.publish(p);
}

/* update the robot's pose in map coordinates */
void updateRobot(tf::StampedTransform map_bl_trans){

    tf::Vector3 bl_origin = tf::Vector3(0.0, 0.0, 0.0);
    tf::Vector3 robotBase = map_bl_trans * bl_origin;
    ROS_INFO("rbase = (%f %f)", robotBase.getX(), robotBase.getY());
    ROS_INFO("res = %f", nav_grid.resolution);
    r_pos.x = robotBase.getX() / nav_grid.resolution;
    r_pos.y = robotBase.getY() / nav_grid.resolution;
    ROS_INFO("rbase = (%f %f)", r_pos.x, r_pos.y);

}


//helper method to calculate distance between point and int point
double calcPointDist(IntPoint a, Point b) {
    return pow(pow(a.x - b.x,2) +pow(a.y - b.y,2),0.5);
}

int getMapInd(IntPoint a){
    //return nav_grid.height*(a.x) + a.y;
    return nav_grid.width*(a.y) + a.x;
}

Point calcSquareNormGrad(tf::Vector3 bl,tf::Vector3 br,tf::Vector3 tr,tf::Vector3 tl){
  //ROS_INFO("[calcSquareNormGrad]\tentering...");

    double dx = ((bl.getZ() - br.getZ()) + (tl.getZ() - tr.getZ()))/2.0;
    double dy = ((tl.getZ() - bl.getZ()) + (tr.getZ() - br.getZ()))/2.0;
    double total_dist = pow( pow(dx, 2) + pow(dy,2) , 0.5);

    if(total_dist != 0){
        Point p = Point(0.5*dx/total_dist, 0.5*dy/total_dist);
        return p;
    }

    //if were on a plateau, just move a small distance towards the goal
    double xd = (g_pos.x - r_pos.x);
    double yd = (g_pos.y - r_pos.y);
    if(abs(xd) > 1.0) xd = xd/(abs(xd));
    if(abs(yd) > 1.0) yd = yd/(abs(yd));
    Point toreturn = Point(xd, yd);
    return toreturn;
}

tf::Vector3 getCostPosVector(IntPoint vertex){
     
     
    tf::Vector3 v = tf::Vector3(1.0 * vertex.x, 1.0 * vertex.y, 0.0);
  
    //check if this vertex's point is in bounds
    if(((int)vertex.x) >= 0 && ((int)vertex.y) >= 0 && ((int)vertex.x) < nav_grid.width && ((int)vertex.y) < nav_grid.height){
        v.setZ(nav_grid.costs[getMapInd(vertex)]);
    } else { //if it isn't, set the cost to be real high
        v.setZ(1000.0);
    }
    return v;
}

/*
 * Get the x and y distance to move if you're moving along the slope of
 * the plane defined by points a, b, and c
 */
Point calcNormGrad(tf::Vector3 a, tf::Vector3 b, tf::Vector3 c){

    tf::Vector3 ab = b-a;
    tf::Vector3 ac = c-a;

    //ROS_INFO("ab %f %f %f", ab.getX(), ab.getY(), ab.getZ());
    //ROS_INFO("ac %f %f %f", ac.getX(), ac.getY(), ac.getZ());


    tf::Vector3 cross = ab.cross(ac);
    tf::Vector3 cross2 = ac.cross(ab);

    //ROS_INFO("cross %f %f %f", cross.getX(), cross.getY(), cross.getZ());
    //ROS_INFO("cross2 %f %f %f", cross2.getX(), cross2.getY(), cross2.getZ());

    //we want to use whichever cross product is sticking out positively in the y direction
    double xcomp = cross.getX() ;
    double ycomp = cross.getY() ;
    if(cross2.getZ() > 0){
        xcomp = cross2.getX();
        ycomp = cross2.getY();
    }

    double normF = pow( pow(xcomp, 2) + pow(ycomp, 2) , 0.5);

    //ROS_INFO("%f %f %f", xcomp, ycomp, normF);

    if(normF != 0.0){ //if were not on a plateau
        Point toreturn = Point(STEPSIZE * xcomp/normF, STEPSIZE * ycomp/normF);
        return toreturn;
    }
    
    //if were on a plateau, just move a small distance towards the goal
    double xd = (g_pos.x - r_pos.x);
    double yd = (g_pos.y - r_pos.y);
    if(abs(xd) > 1.0) xd = xd/(abs(xd));
    if(abs(yd) > 1.0) yd = yd/(abs(yd));
    Point toreturn = Point(xd, yd);
    return toreturn;
    
}

/*
 * Find the next point along the gradient
 */
Point getNextPoint(Point curP){

    IntPoint bl = IntPoint(floor(curP.x), floor(curP.y));
    IntPoint br = IntPoint(ceil(curP.x), floor(curP.y));
    IntPoint tr = IntPoint(ceil(curP.x), ceil(curP.y));
    IntPoint tl = IntPoint(floor(curP.x), ceil(curP.y));

    //if its right on the point
    if(ceil(curP.x) == curP.x){
        br.x = br.x + 1;
        tr.x = tr.x + 1;
    }
    if(ceil(curP.y) == curP.y){
        tr.y = tr.y + 1;
        tl.y = tl.y + 1;
    }

    tf::Vector3 a = getCostPosVector(bl);
    tf::Vector3 b = getCostPosVector(br);
    tf::Vector3 c = getCostPosVector(tl);
    tf::Vector3 d = getCostPosVector(tr);
    Point distToMove = calcSquareNormGrad(a,b,d,c);
 
/*
    //find the 3 defining points and 2 defining border vectors 
    //0 contains lower left, 1 lower right, 2 upper right, 3 upper left 
    vector<double> node_dists(4);
    vector<IntPoint> adjacent_nodes(4);
    IntPoint ll(floor(curP.x), floor(curP.y));
    IntPoint lr(ceil(curP.x), floor(curP.y));
    IntPoint ur(ceil(curP.x), ceil(curP.y));
    IntPoint ul(floor(curP.x), ceil(curP.y));
    adjacent_nodes[0] = ll;
    adjacent_nodes[1] = lr;
    adjacent_nodes[2] = ur;
    adjacent_nodes[3] = ul;
    for(int i = 0; i < 4; i++) { node_dists[i] = calcPointDist(adjacent_nodes[i], curP); }

    //now, find highest distance in node_dists, and eliminate that
    double highest = -999999.9;
    int highestI = -1;
    for(int i = 0; i<4; i++){
        if(node_dists[i] > highest){
            highest = node_dists[i];
            highestI = i;
        }
    }

    //ROS_INFO("curP = (%f %f), farthest=(%d %d)", curP.x, curP.y, adjacent_nodes[highestI].x, adjacent_nodes[highestI].y);

    //now, we can eliminate the index of the highest distance vector
    //and we have our defining 3 points of the plane
    adjacent_nodes[highestI] = adjacent_nodes[3];
    adjacent_nodes.pop_back();

    //finally, create the two bounding vectors of the
    //triangle plane and get the gradient of them!
    tf::Vector3 a = getCostPosVector(adjacent_nodes[0]);
    tf::Vector3 b = getCostPosVector(adjacent_nodes[1]);
    tf::Vector3 c = getCostPosVector(adjacent_nodes[2]);

    //ROS_INFO("a(%f %f %f)", a.getX(), a.getY(), a.getZ()); 
    //ROS_INFO("b(%f %f %f)", b.getX(), b.getY(), b.getZ()); 
    //ROS_INFO("c(%f %f %f)", c.getX(), c.getY(), c.getZ()); 

    Point distToMove = calcNormGrad(a, b, c);
    */

    //ROS_INFO("disttomove = %f %f", distToMove.x, distToMove.y);
    
    //return the next move
    Point nextMove = Point(curP.x + distToMove.x, curP.y + distToMove.y);
    return nextMove;
}

/*
 * main method to update the path
 *
 */
bool updatePath(){

    //init path object
    //int maxPathSize = nav_grid.width*nav_grid.height;
    int maxPathSize = 1500;
    vector<Point> path(maxPathSize);

    Point curPoint(r_pos.x, r_pos.y);
    int curPos = 0;

    ROS_INFO("starting pos=(%f %f), goal pos=(%f %f)", r_pos.x, r_pos.y, g_pos.x, g_pos.y);

    if(r_pos.x > DBL_MAX || r_pos.y > DBL_MAX) return false; 

    //iterate for a goal while cur point is more than 1 unit away
    while( curPoint.dist(g_pos) > 1.0 ) {
      //ROS_INFO("curpoint: (%f %f) distance to goal: %f",curPoint.x, curPoint.y, curPoint.dist(g_pos)); 

       //find the next point in the path following the gradient
       curPoint = getNextPoint(curPoint);
       path[curPos].x = curPoint.x;
       path[curPos].y = curPoint.y;

       ROS_INFO("(after) curpoint: (%f %f) distance to goal: %f",curPoint.x, curPoint.y, curPoint.dist(g_pos)); 

       curPos = curPos + 1;
       if(curPos >= maxPathSize) return false;
    }

    //if we've found something, publish it!
    ROS_INFO("about to publish the found path");
    publishPath(path, curPos);

    return true;
}

int main(int argc, char** argv) {

    ////// TESTING CODE ///////////
    /*
    tf::Vector3 a = tf::Vector3(1.0, 1.0, 10.0);
    tf::Vector3 b = tf::Vector3(2.0, 1.0, 10.0);
    tf::Vector3 c = tf::Vector3(1.0, 2.0, 20.0);
    //tf::Vector3 a = tf::Vector3(23.0, 28.0, 863.0);
    //tf::Vector3 b = tf::Vector3(24.0, 28.0, 864.0);
    //tf::Vector3 c = tf::Vector3(23.0, 29.0, 893.0);
    tf::Vector3 d = tf::Vector3(1.0, 0.0, 0.0);
    Point t = calcNormGrad(b,c,a);
    ROS_INFO("%f %f", t.x, t.y);
    Point t2 = calcNormGrad(a,b,d);
    ROS_INFO("%f %f", t2.x, t2.y);
   
    test();
    updatePath();
   return 0;
   */
    ////// END TESTING CODE ///////////

    ros::init(argc, argv, "pathfinder");
    ros::NodeHandle nh;

    //init publishers/subscribers
    ros::Subscriber nav_fn_sub = nh.subscribe("/nav_grid", 1, navFnCallback);
    path_pub = nh.advertise<assn2::Path>("path",1);
    marker_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    
    //set up transform listeners
    tf::TransformListener listener;
    
    //set loop rate
    ros::Rate rate(10);

    
    while(ros::ok()){

        //update the goal and nav function if there are new ones
        ros::spinOnce();
        publishGradField();

        //update transform from base_link to map
        tf::StampedTransform map_base_link_transform; 
        try {
            ROS_INFO("getting transform....");
            listener.lookupTransform("/map", "/base_link", ros::Time(0), map_base_link_transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }

        //update the robots position in map coordinates
        updateRobot(map_base_link_transform);

        //update the goals position in map coordinates
        updateGoal();

        //create and update the path
        bool foundPath = updatePath();
	
        if(!foundPath){
            ROS_WARN("uh oh, we failed to find a path");
        }

        rate.sleep();
    }

}
