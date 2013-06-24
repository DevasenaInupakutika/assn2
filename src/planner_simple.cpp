#include <queue>
#include "math.h"
#include "limits.h"
#include "float.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/OccupancyGrid.h"
#include "assn2/NavGrid.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "tf_util.h"

#define _USE_MATH_DEFINES

using namespace std;

const double OBSTACLE_COST = 100;
const double UNKNOWN_COST = 100;
const double FREE_SPACE_COST = 1;
const double DECAY_FLAT_DIST = 2; // this turns the cost map into configuration space
const double DECAY_RATE = 2;
const double INIT_LPN_COST = DBL_MAX;

int MAP_SIZE_X = -1;
int MAP_SIZE_Y = -1;
int MAP_CELLS = -1;
float MAP_RESOLUTION = -1;
tf::Stamped<tf::Pose> goal_pose;
nav_msgs::OccupancyGrid global_map;
ros::Publisher marker_pub;
ros::Publisher map_pub;

// basic datapoint class
class Point {
public:
  int x;
  int y;
  Point() {}
  Point(int _x, int _y) {x=_x; y=_y;}
  Point(int i) {x=i%MAP_SIZE_X; y=i/MAP_SIZE_X;}
  double dist(const Point &other_pt) const {
    return pow(pow(x - other_pt.x,2) +pow(y - other_pt.y,2),0.5);
  }
  bool isValid() {
    return (x >= 0) && (x < MAP_SIZE_X) && (y >= 0) && (y < MAP_SIZE_Y);
  }
  Point operator-(Point other_pt) {
    return Point(x - other_pt.x,y - other_pt.y);
  }
  Point operator+(Point other_pt) {
    return Point(x + other_pt.x,y + other_pt.y);
  }
  double norm() {return dist(Point(0,0));}
  double dot(Point other_pt) {return x*other_pt.x + y*other_pt.y;}
  static Point subtract(Point pt1, Point pt2) {
    Point difference(pt2.x-pt1.x,pt2.y-pt1.y);
    return difference;
  };
  static double angle(Point pt1, Point vertex, Point pt2) {
    Point edge1 = pt1 - vertex;
    Point edge2 = pt2 - vertex;
    // a \cdot b = ||a|| * ||b|| * cos(\theta)
    double theta = acos(edge1.dot(edge2) / (edge1.norm() * edge2.norm()));
    if (isnan(theta)) {
      ROS_INFO("[angle]\tpt1: (%d,%d), vertex: (%d,%d), pt2: (%d,%d), theta=%f",pt1.x,pt1.y,vertex.x,vertex.y,pt2.x,pt2.y,theta);
    }
    return theta;
  }
};

class PairComparator
{
public:
  PairComparator() {}
  bool operator() (const pair<Point,double> &lhs, const pair<Point,double> &rhs) const {
    return (lhs.second > rhs.second);
  }
};
  
// simple help function for 1d representation of 2d grid
template <class T>
T getVal(T* cost_map, int i,int j) {
  int index = MAP_SIZE_X*j + i;
  return cost_map[index];
}

// simple help function for 1d representation of 2d grid
template <class T>
void setVal(T* cost_map, int i,int j, T val) {
  int index = MAP_SIZE_X*j + i;
  cost_map[index] = val;
}
  
// simple help function for 1d representation of 2d grid
template <class T>
T getVal(vector<T> cost_map, int i,int j) {
  int index = MAP_SIZE_X*j + i;
  return cost_map[index];
}

// simple help function for 1d representation of 2d grid
template <class T>
void setVal(vector<T> cost_map, int i,int j, T val) {
  int index = MAP_SIZE_X*j + i;
  cost_map[index] = val;
}

void initMarker(visualization_msgs::Marker &marker, string name, int type) {
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = name;
  marker.id = 0;
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = .1;
  marker.scale.y = .1;
  marker.scale.z = .1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration(5);
}


void visualizeMap(double* map, string name, double scale_factor=1) {
  ROS_DEBUG("[visualizeMap]\tentering");
  visualization_msgs::Marker marker;
  geometry_msgs::Point p;
  for (int i =0; i < MAP_CELLS; i++) {
    Point convert_pt(i);
    p.x = convert_pt.x * MAP_RESOLUTION;
    p.y = convert_pt.y * MAP_RESOLUTION;
    p.z = map[i] * scale_factor;
    // big and small values seem to kill rviz
    if (p.z < 0.01) p.z = 0.01;
    if (p.z > 100) p.z = 100;
    marker.points.push_back(p);
  }
  marker_pub.publish(marker);
}

void testArrows() {
  visualization_msgs::Marker simpleMarker;
  initMarker(simpleMarker,"simpleMarker",visualization_msgs::Marker::POINTS);
  geometry_msgs::Point testP;
  testP.x = 10;
  testP.y = 10;
  testP.z = 0;
  simpleMarker.points.push_back(testP);
  marker_pub.publish(simpleMarker);

  visualization_msgs::Marker marker;
  initMarker(marker,"gradient",visualization_msgs::Marker::LINE_LIST);
  geometry_msgs::Point p;
  for (int i=0; i < 10; i++) {
    p.x = i;
    p.y = i;
    p.z = 0;
    marker.points.push_back(p);
    p.x = i + 1;
    p.y = i + 1;
    marker.points.push_back(p);
  }
  marker_pub.publish(marker);
}

void visualizeHeatMap(double* map, string name) {
  //ROS_INFO("[visualizeHeatMap]\tentering for map: %s", name.c_str());

  // find range
  double min = map[0];
  double max = map[0];
  for (int i=0; i < MAP_CELLS; i++) {
    if (map[i] != -1 || map[i] != DBL_MAX) {
      if (map[i] < min) min = map[i];
      if (map[i] > max) max = map[i];
    }
  }
  double range = max - min;
  ROS_INFO("[visualizeHeatMap]\tmin=%f, range=%f",min,range);

  visualization_msgs::Marker marker;
  initMarker(marker,name,visualization_msgs::Marker::POINTS);
  geometry_msgs::Point p;
  std_msgs::ColorRGBA c;
  for (int i =0; i < MAP_CELLS; i++) {
    if (map[i] != -1 && map[i] != DBL_MAX) {
      Point convert_pt(i);
      p.x = convert_pt.x * MAP_RESOLUTION;
      p.y = convert_pt.y * MAP_RESOLUTION;
      p.z = 0;
      
      // high the value, the bluer the color
      c.b = (map[i] - min)/range;;  // should be between 0 and 1
      c.r = 1 - c.b;
      c.g = 0;
      c.a = 1.0;
      
      marker.points.push_back(p);
      marker.colors.push_back(c);
    }
  }
  marker_pub.publish(marker);
}

void publishMap(double* map) {
    assn2::NavGrid ng;
    ng.width = (uint32_t) MAP_SIZE_X;
    ng.height = (uint32_t) MAP_SIZE_Y;
    ng.resolution = MAP_RESOLUTION;

    //convert the map to a vector
    std::vector<double> costs(ng.width*ng.height);
    for(uint32_t i = 0; i < ng.width*ng.height; i++) { costs[i] = map[i]; }
    ng.costs = costs;
    ng.goalX = goal_pose.getOrigin().getX();
    ng.goalY = goal_pose.getOrigin().getY();

    //publish
    map_pub.publish(ng);
}

void printMap(double* map) {
  ROS_INFO("[printMap]\tentering");
  for (int i=0; i<MAP_CELLS;i++) {
    if (i == MAP_SIZE_X) {printf("\n");}
    Point pt(i);
    printf("(%d,%d):%f\t",pt.x,pt.y,getVal(map,pt.x,pt.y));
  }
}

// takes global_map and creates a boolean map of known obstacles (values greater than 0)
void getBoundaryMap(bool* boundary_map) {
  
  //double vis_map[MAP_CELLS];
  ROS_DEBUG("[initBoundaryMap]\tentering");
  for (int i=0; i < MAP_CELLS; i++) {
    signed char map_cost = global_map.data[i];
    //vis_map[i] = 0.0;
    if (map_cost <= 0) {
      boundary_map[i] = false; // unkown, or known and free; don't consider part of initial set
    } else {
      boundary_map[i] = true;
    }
  }
}

int getRingNbrs(Point center, Point* nbrs) {
  Point nbr_offset = Point(0,1); // start at 12 o'clock
  int num_nbrs = 0;
  Point nbr;
  while (num_nbrs < 8) {
    Point nbr_pt = nbr_offset + center;

    if (nbr_pt.isValid()) {
      nbrs[num_nbrs] = nbr_pt;
      num_nbrs++;
    }

    if (nbr_offset.x == -1 && nbr_offset.y == 1) {
      break;
    }

    if (nbr_offset.x == 0 && nbr_offset.y == 1) {
      nbr_offset.x = 1;
      nbr_offset.y = 1;
    } else if (nbr_offset.x == 1 && nbr_offset.y == 1) {
      nbr_offset.x = 1;
      nbr_offset.y = 0;
    } else if (nbr_offset.x == 1 && nbr_offset.y == 0) {
      nbr_offset.x = 1;
      nbr_offset.y = -1;
    } else if (nbr_offset.x == 1 && nbr_offset.y == -1) {
      nbr_offset.x = 0;
      nbr_offset.y = -1;
    } else if (nbr_offset.x == 0 && nbr_offset.y == -1) {
      nbr_offset.x = -1;
      nbr_offset.y = -1;
    } else if (nbr_offset.x == -1 && nbr_offset.y == -1) {
      nbr_offset.x = -1;
      nbr_offset.y = 0;
    } else if (nbr_offset.x == -1 && nbr_offset.y == 0) {
      nbr_offset.x = -1;
      nbr_offset.y = 1;
    } 
  }
  //ROS_INFO("[getRingNbrs]\treturning %d nbrs)",num_nbrs);
  return num_nbrs;
}

int get4Nbrs(const Point &pt, Point* nbrs) {
  int num_nbrs = 0;
  Point new_pt;
  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      //ROS_INFO("[get4Nbrs]\ti=%d, j=%d",i,j);
      if (i*j == 0 && i!=j) {
	new_pt.x = pt.x + i;
	new_pt.y = pt.y + j;
	if (new_pt.isValid()) {
	  //ROS_INFO("[get4Nbrs]\tadding nbr: (%d,%d)",new_x,new_y);
	  nbrs[num_nbrs] = new_pt;
	  num_nbrs++;
	}
      }
    }
  }
  return num_nbrs;
}

void updateMapDijkstra(double* map, bool* init_set_map,Point goal,double* edge_cost_map,int max_dist) {
  ROS_DEBUG("[updateMapDijkstra]\tentering");
  bool visited[MAP_CELLS];
  for (int i=0; i < MAP_CELLS; i++) {visited[i] = false;}
  
  priority_queue<pair<Point,double>, vector<pair<Point,double> >, PairComparator > pq;
  
  double init_cost = 0.0;
  for (int i=0; i < MAP_CELLS; i++) {
    if (init_set_map[i]) {
      pair<Point,double> init_pair = make_pair(Point(i),init_cost);
      pq.push(init_pair);
      //ROS_INFO("[updateMapDijkstra]\tpushing new pair");
    }
  }

  pair<Point,double> curr_pq_elem;
  Point curr_pt;
  double curr_cost;
  double edge_cost;
  Point nbrs[4];
  int num_nbrs;
  while (!pq.empty()) {
    curr_pq_elem = pq.top();
    pq.pop();

    curr_pt = curr_pq_elem.first;
    curr_cost = curr_pq_elem.second;
    //ROS_INFO("[updateMapDijkstra]\tpopped Point(%d,%d) with cost: %f", curr_pt.x,curr_pt.y,curr_cost);
    if (getVal(visited,curr_pt.x,curr_pt.y)) {continue;} // don't update value of points which have been visited
    if (curr_cost < 0) {ROS_WARN("found cost less that 0: %f",curr_cost);}

    // set info for curr_pt
    setVal(visited,curr_pt.x,curr_pt.y,true);
    setVal(map,curr_pt.x,curr_pt.y, curr_cost);
    
    // this stops enqueuing edges when we get sufficiently far away
    if (curr_cost >= max_dist) {continue;}

    // enqueue neighbors
    edge_cost = getVal(edge_cost_map,curr_pt.x,curr_pt.y);
    num_nbrs = get4Nbrs(curr_pt,nbrs);
    for (int i = 0; i < num_nbrs; i++) {
      if (!getVal(visited,nbrs[i].x,nbrs[i].y)) {
	double new_cost = curr_cost + edge_cost;
	pq.push(make_pair(nbrs[i],new_cost));
      }
    }
  }
}

void computeDecay(double* obstacle_dist_map, double* cost_map) {
  double obstacle_dist;
  for (int i=0; i < MAP_CELLS; i++) {
    obstacle_dist = obstacle_dist_map[i];
    if (obstacle_dist == -1) { // we are not within MAX_DIST of obstacle
      // need to check whether we are in free space or unknown space
      //if (global_map.data[i] == 0) { // we are in free space
      if (global_map.data[i] == 0) { // we are in free space
	cost_map[i] = FREE_SPACE_COST;
      } else { // we are in unknown territory
	cost_map[i] = UNKNOWN_COST;
      }
    } else if (obstacle_dist >= 0) { // we are within MAX_DIST of obstacle
      if (global_map.data[i] == -1) { // we are in unknown
	cost_map[i] = UNKNOWN_COST;
      }
      else if (obstacle_dist <= DECAY_FLAT_DIST) { // we are on an obstacle
	cost_map[i] = OBSTACLE_COST;
      } else { // we are within 1 and MAX_DIST of obstacle so compute decay
	cost_map[i] = OBSTACLE_COST*(1.0/(DECAY_RATE*obstacle_dist));
	//ROS_INFO("[getCostMap]\tobstacle_dist=%f, cost=%f",obstacle_dist,cost_map[i]);
      }
    } else { // something weird happened, so just go with unknown
      cost_map[i] = UNKNOWN_COST;
      ROS_WARN("[getCostMap]\tfound unexpected obstacle_dist: %f", obstacle_dist);
    }
  }
}

void getCostFn(double* cost_map) {
  bool boundary_map[MAP_CELLS];
  getBoundaryMap(boundary_map);
  double obstacle_dist_map[MAP_CELLS]; // doesn't need to be double but easier for dijkstra interface
  double uniform_edge_cost_map[MAP_CELLS];
  for (int i = 0; i < MAP_CELLS; i++) {
    // set points which dijkstra's never touch to -1 so we know not to give them cost
    obstacle_dist_map[i] = -1;
    // set uniform cost of moving between points (this is just a way of representing distance)
    uniform_edge_cost_map[i] = 1;
  }
  Point empty_goal(-1,-1); // this point shouldn't exist so we will never converge because we reached it
  int MAX_OBSTACLE_DECAY_DIST = 6;
  updateMapDijkstra(obstacle_dist_map,boundary_map,empty_goal,uniform_edge_cost_map,MAX_OBSTACLE_DECAY_DIST);
  //visualizeHeatMap(obstacle_dist_map,"obstacle_dist");
  computeDecay(obstacle_dist_map,cost_map);
  for (int i = 0; i < MAP_CELLS; i++) {
    if (cost_map[i] <= 0) {
      ROS_WARN("[getCostFn]\tfound node with invalid cost: cost_map[%d]=%f",i,cost_map[i]);
    }
  }
  ROS_DEBUG("[getCostFn]\tcomputed cost map");
}


double simpleApprox(Point sample_pt, double* nav_fn, double intrinsic_cost) {
  double new_pt_nav_fn = DBL_MAX;

  Point nbrs[4];
  int num_nbrs = get4Nbrs(sample_pt,nbrs);
  for (int i = 0; i < num_nbrs; i++) {
    Point pt1 = nbrs[i];
    Point pt2 = nbrs[(i+1) % num_nbrs]; // this lets us loop around
    double pt1_nav_fn = getVal(nav_fn,pt1.x,pt1.y);
    double pt2_nav_fn = getVal(nav_fn,pt2.x,pt2.y);
    double cand = min(pt1_nav_fn,pt2_nav_fn) + intrinsic_cost;
    if (cand < new_pt_nav_fn) {
      new_pt_nav_fn = cand;
    }
  }
  return new_pt_nav_fn;

}


double planeWaveApprox4NbrNew(Point sample_pt, double* nav_fn, double intrinsic_cost) {
  //ROS_INFO("[planeWaveApprox]\tentering with sample_pt: (%d,%d)",sample_pt.x,sample_pt.y);
  // iterate over different pt1 offsets

  double new_pt_nav_fn = DBL_MAX;
  Point nbrs[4];
  int num_nbrs = get4Nbrs(sample_pt,nbrs);
  for (int i = 0; i < num_nbrs; i++) {
    Point pt1 = nbrs[i];
    Point pt2 = nbrs[(i+1) % num_nbrs]; // this lets us loop around

    // let pt1 be the point through which we assume the isopotential runs
    if (getVal(nav_fn,pt1.x,pt1.y) > getVal(nav_fn,pt2.x,pt2.y)) {
      Point pt1_tmp = pt1;
      pt1 = pt2;
      pt2 = pt1_tmp;
    }    
    double pt1_nav_fn = getVal(nav_fn,pt1.x,pt1.y);
    double pt2_nav_fn = getVal(nav_fn,pt2.x,pt2.y);

    double a = -1;
    double b = -1;
    double c = -1;
    double disc = -1;
    double disc_rt = -1;
    double new_pt_nav_fn_opt1 = -1;
    double new_pt_nav_fn_opt2 = -1;
    double new_pt_nav_fn_cand= -1;

    // setup quadratic coefficients
    a = 2;
    b = -2 * (pt1_nav_fn + pt2_nav_fn);
    c = pow(pt1_nav_fn,2) + pow(pt2_nav_fn,2) - pow(intrinsic_cost,2);
    disc = pow(b,2) - 4*a*c;
    // if determinant is negative, we have no real solution so do linear interpolation
    if (disc < 0) {
      new_pt_nav_fn_cand = pt1_nav_fn + intrinsic_cost;
      ROS_DEBUG("[planeWaveApprox]\tfound case where discriminant is neg: disc = %f",disc);
    } else {
      disc_rt = pow(disc, 0.5);
      new_pt_nav_fn_opt1 = (-b - disc_rt) / (2*a);
      new_pt_nav_fn_opt2 = (-b + disc_rt) / (2*a);
      new_pt_nav_fn_cand = new_pt_nav_fn_opt1;
      if (new_pt_nav_fn_opt1 < 0 || new_pt_nav_fn_opt1 < pt1_nav_fn) {
	new_pt_nav_fn_cand = new_pt_nav_fn_opt2;
      }
    }

    if (new_pt_nav_fn_cand < new_pt_nav_fn) {
      new_pt_nav_fn = new_pt_nav_fn_cand;
    }
     
    // checks
    bool isnan_check = isnan(new_pt_nav_fn_cand);
    bool isinf_check = isinf(new_pt_nav_fn_cand);
    bool new_pt_neg_check = new_pt_nav_fn_cand < 0;
    double pt1_bound = pt1_nav_fn + intrinsic_cost;
    double pt2_bound = pt2_nav_fn + intrinsic_cost;
    double EPS = 0.0000001;
    bool new_pt_too_large_pt1_check = new_pt_nav_fn_cand > pt1_bound + EPS;
    bool new_pt_too_large_pt2_check = new_pt_nav_fn_cand > pt2_bound + EPS;
    bool new_pt_lt_pt1_check = new_pt_nav_fn_cand < pt1_nav_fn;
    bool new_pt_lt_pt2_check = new_pt_nav_fn_cand < pt2_nav_fn;

    // always print this
    ROS_DEBUG("[planeWaveApprox]\t\tnew_pt_nav_fn_cand=%f", new_pt_nav_fn_cand);
    // debug info
    if (isnan_check || 
	isinf_check || 
	new_pt_neg_check ||
	new_pt_too_large_pt1_check ||
	new_pt_too_large_pt2_check ||
	new_pt_lt_pt1_check ||
	new_pt_lt_pt1_check)
      {
	ROS_INFO("[planeWaveApprox]\tfound special case, nav_fn(%d,%d)=%f",sample_pt.x,sample_pt.y,new_pt_nav_fn_cand);
	ROS_INFO("[planeWaveApprox]\tisnan_check: \t%d",isnan_check);
	ROS_INFO("[planeWaveApprox]\tisinf_check: \t%d",isinf_check);
	ROS_INFO("[planeWaveApprox]\tnew_pt_neg_check: \t%d",new_pt_neg_check);
	ROS_INFO("[planeWaveApprox]\tpt1_bound=%f",pt1_bound);
	ROS_INFO("[planeWaveApprox]\tpt2_bound=%f",pt2_bound);
	ROS_INFO("[planeWaveApprox]\tnew_pt_too_large_pt1_check: \t%d",new_pt_too_large_pt1_check);
	ROS_INFO("[planeWaveApprox]\tnew_pt_too_large_pt2_check: \t%d",new_pt_too_large_pt2_check);
	ROS_INFO("[planeWaveApprox]\tnew_pt_lt_pt1_check: \t%d",new_pt_lt_pt1_check);
	ROS_INFO("[planeWaveApprox]\tnew_pt_lt_pt2_check: \t%d",new_pt_lt_pt2_check);
	
	ROS_INFO("[planeWaveApprox]\tintrinsic_cost=%f",intrinsic_cost);
	ROS_INFO("[planeWaveApprox]\tpt1_nav_fn=%f",pt1_nav_fn);
	ROS_INFO("[planeWaveApprox]\tpt2_nav_fn=%f",pt2_nav_fn);
	
	ROS_INFO("[planeWaveApprox]\tta=%f, b=%f, c=%f, disc=%f",a,b,c,disc);
	ROS_INFO("[planeWaveApprox]\topt1=%f, opt2=%f",new_pt_nav_fn_opt1, new_pt_nav_fn_opt2);	
	
	// summary
	ROS_INFO("[planeWaveApprox]\tpt1:\tnav_fn(%d,%d)\t=%f",pt1.x,pt1.y,pt1_nav_fn);
	ROS_INFO("[planeWaveApprox]\tpt2:\tnav_fn(%d,%d)\t=%f",pt2.x,pt2.y,pt2_nav_fn);
	ROS_INFO("[planeWaveApprox]\tnew_val:\tcandidate nav_fn(%d,%d)=%f",sample_pt.x,sample_pt.y,new_pt_nav_fn_cand);
	ROS_INFO("[planeWaveApprox]\tnew_val:\tnav_fn(%d,%d)=%f",sample_pt.x,sample_pt.y,new_pt_nav_fn);
      }
  }
  return new_pt_nav_fn;
}

double planeWaveApprox8NbrNew(Point sample_pt, double* nav_fn, double intrinsic_cost) {
  //ROS_INFO("[planeWaveApprox]\tentering with sample_pt: (%d,%d)",sample_pt.x,sample_pt.y);
  // iterate over different pt1 offsets

  double new_pt_nav_fn = DBL_MAX;
  Point nbrs[8];
  int num_nbrs = getRingNbrs(sample_pt,nbrs);
  for (int i = 0; i < num_nbrs; i++) {
    Point pt1 = nbrs[i];
    Point pt2 = nbrs[(i+1) % num_nbrs]; // this lets us loop around

    // let pt1 be the corner point (c in diagrams)
    if (sample_pt.dist(pt2) < sample_pt.dist(pt1)) {
      Point pt1_tmp = pt1;
      pt1 = pt2;
      pt2 = pt1_tmp;
    }
    double pt1_nav_fn = getVal(nav_fn,pt1.x,pt1.y);
    double pt2_nav_fn = getVal(nav_fn,pt2.x,pt2.y);

    double a = -1;
    double b = -1;
    double c = -1;
    double disc = -1;
    double disc_rt = -1;
    double new_pt_nav_fn_opt1 = -1;
    double new_pt_nav_fn_opt2 = -1;
    double new_pt_nav_fn_cand= -1;

    // setup quadratic coefficients
    a = 1;
    b = -2*pt1_nav_fn;
    c = pow(pt2_nav_fn - pt1_nav_fn, 2) + pow(pt1_nav_fn,2) - pow(intrinsic_cost,2);
    disc = pow(b,2) - 4*a*c;
    // if determinant is negative, we have no real solution so do linear interpolation
    if (disc < 0) {
      double sqrt2 = pow(2,0.5);
      new_pt_nav_fn_cand = min(pt1_nav_fn + intrinsic_cost,pt2_nav_fn + sqrt2*intrinsic_cost);
      //ROS_INFO("[planeWaveApprox]\tfound case where discriminant is neg: disc = %f",disc);
      //ROS_INFO("[planeWaveApprox]\tmin(%f,%f) = %f",pt1_nav_fn + intrinsic_cost,pt2_nav_fn + sqrt2*intrinsic_cost,new_pt_nav_fn_cand);
    } else {
      disc_rt = pow(disc, 0.5);
      new_pt_nav_fn_opt1 = (-b - disc_rt) / (2*a);
      new_pt_nav_fn_opt2 = (-b + disc_rt) / (2*a);
      new_pt_nav_fn_cand = new_pt_nav_fn_opt1;
      if (new_pt_nav_fn_opt1 < 0 || new_pt_nav_fn_opt1 < pt1_nav_fn) {
	new_pt_nav_fn_cand = new_pt_nav_fn_opt2;
      }
    }

    if (new_pt_nav_fn_cand < new_pt_nav_fn) {
      new_pt_nav_fn = new_pt_nav_fn_cand;
    }
     
    // checks
    bool isnan_check = isnan(new_pt_nav_fn_cand);
    bool isinf_check = isinf(new_pt_nav_fn_cand);
    bool new_pt_neg_check = new_pt_nav_fn_cand < 0;
    double pt1_bound = pt1_nav_fn + intrinsic_cost;
    double pt2_bound = pt2_nav_fn + pow(2,0.5)*intrinsic_cost;
    double EPS = 0.0000001;
    bool new_pt_too_large_pt1_check = new_pt_nav_fn_cand > pt1_bound + EPS;
    bool new_pt_too_large_pt2_check = new_pt_nav_fn_cand > pt2_bound + EPS;
    bool new_pt_lt_pt1_check = new_pt_nav_fn_cand < pt1_nav_fn;
    bool new_pt_lt_pt2_check = new_pt_nav_fn_cand < pt2_nav_fn;
    bool too_small_check = new_pt_lt_pt1_check && new_pt_lt_pt2_check;

    // always print this
    ROS_DEBUG("[planeWaveApprox]\t\tnew_pt_nav_fn_cand=%f", new_pt_nav_fn_cand);
    // debug info
    if (true || isnan_check || 
	isinf_check || 
	new_pt_neg_check ||
	new_pt_too_large_pt1_check ||
	new_pt_too_large_pt2_check ||
	too_small_check)
      {
	ROS_INFO("\n\n[planeWaveApprox]\tfound special case, nav_fn(%d,%d)=%f",sample_pt.x,sample_pt.y,new_pt_nav_fn_cand);
	ROS_INFO("[planeWaveApprox]\tisnan_check: \t%d",isnan_check);
	ROS_INFO("[planeWaveApprox]\tisinf_check: \t%d",isinf_check);
	ROS_INFO("[planeWaveApprox]\tnew_pt_neg_check: \t%d",new_pt_neg_check);
	ROS_INFO("[planeWaveApprox]\tpt1_bound=%f",pt1_bound);
	ROS_INFO("[planeWaveApprox]\tpt2_bound=%f",pt2_bound);
	ROS_INFO("[planeWaveApprox]\tnew_pt_too_large_pt1_check: \t%d",new_pt_too_large_pt1_check);
	ROS_INFO("[planeWaveApprox]\tnew_pt_too_large_pt2_check: \t%d",new_pt_too_large_pt2_check);
	ROS_INFO("[planeWaveApprox]\ttoo_small_check: \t%d",too_small_check);
	ROS_INFO("[planeWaveApprox]\tnew_pt_lt_pt1_check: \t%d",new_pt_lt_pt1_check);
	ROS_INFO("[planeWaveApprox]\tnew_pt_lt_pt2_check: \t%d",new_pt_lt_pt2_check);
	
	ROS_INFO("[planeWaveApprox]\tintrinsic_cost=%f",intrinsic_cost);
	ROS_INFO("[planeWaveApprox]\tpt1_nav_fn=%f",pt1_nav_fn);
	ROS_INFO("[planeWaveApprox]\tpt2_nav_fn=%f",pt2_nav_fn);
	
	ROS_INFO("[planeWaveApprox]\tta=%f, b=%f, c=%f, disc=%f",a,b,c,disc);
	ROS_INFO("[planeWaveApprox]\topt1=%f, opt2=%f",new_pt_nav_fn_opt1, new_pt_nav_fn_opt2);	
	
	// summary
	ROS_INFO("[planeWaveApprox]\tpt1:\tnav_fn(%d,%d)\t=%f",pt1.x,pt1.y,pt1_nav_fn);
	ROS_INFO("[planeWaveApprox]\tpt2:\tnav_fn(%d,%d)\t=%f",pt2.x,pt2.y,pt2_nav_fn);
	ROS_INFO("[planeWaveApprox]\tnew_val:\tcandidate nav_fn(%d,%d)=%f",sample_pt.x,sample_pt.y,new_pt_nav_fn_cand);
	ROS_INFO("[planeWaveApprox]\tnew_val:\tnav_fn(%d,%d)=%f",sample_pt.x,sample_pt.y,new_pt_nav_fn);
      }
  }
  return new_pt_nav_fn;
}


double singleNbrApprox(Point sample_pt,double intrinsic_cost, double* nav_fn) {
  ROS_DEBUG("[planeWaveApprox]\tusing single neighbor interpolation for nav_fn(%d,%d)",sample_pt.x,sample_pt.y);
  double new_pt_nav_fn = DBL_MAX;

  // we are within 1 of goal, so we find an adjacent cell and use manhattan style
  for (int i = -1; i < 2; i++) {
    for (int j = -1; j < 2; j++) {
      if (i*j == 0 && i != j) {
	Point nbr_pt(sample_pt.x + i, sample_pt.y + j);
	double nbr_nav_fn = getVal(nav_fn,nbr_pt.x,nbr_pt.y);
	double candidate_nbr_nav_fn = nbr_nav_fn + intrinsic_cost;
	if (candidate_nbr_nav_fn < new_pt_nav_fn) {
	  new_pt_nav_fn = candidate_nbr_nav_fn;
	}
      }
    }
  }
  return new_pt_nav_fn;
}

void insertAndPurgeDups(priority_queue<pair<Point,double>, vector<pair<Point,double> >, PairComparator > &pq,pair<Point,double> insert_pr) {
  uint init_size = pq.size();
  pair<Point,double> pr;
  Point p;
  //double cost;
  priority_queue<pair<Point,double>, vector<pair<Point,double> >, PairComparator > pq_temp;
  while(!pq.empty()) {
    pr = pq.top();
    p = pr.first;
    //cost = pr.second;
    pq.pop(); // empty out pq
    if ((p.x != insert_pr.first.x) || (p.y != insert_pr.first.y)) {
      pq_temp.push(pr); // conditionally add to pq_temp
    } else {
      //ROS_INFO("[insertAndPurgeDups]\tfound dup for Point(%d,%d)",p.x,p.y);
    }
  }

  // copy from pq_temp to pq
  while(!pq_temp.empty()) {
    pq.push(pq_temp.top());
    pq_temp.pop();
  }

  // add new item to pq
  pq.push(insert_pr);
  if (init_size > pq.size()) {
    std::cout<< "[insertAndPurgeDups]\tentered with  "<< init_size << ", exiting with " << pq.size();
  }
}

void visualizeActiveSet(priority_queue<pair<Point,double>, vector<pair<Point,double> >, PairComparator > pq, double* nav_fn) {
  double map[MAP_CELLS];
  for (int i=0; i < MAP_CELLS; i++) {
    map[i] = -1;
  }
  pair<Point,double> pr;
  Point p;
  double cost;

  while(!pq.empty()) {
    pr = pq.top();
    p = pr.first;
    cost = pr.second;
    pq.pop();
    if (cost > getVal(nav_fn,p.x,p.y)) {continue;}
    setVal(map,p.x,p.y, cost);
  }
  visualizeHeatMap(map,"active_set");
}

void LPN2(double* nav_fn, double* intrinsic_costs, Point goal, Point start) {
  ROS_INFO("[LPN]\tentering");
  for (int i=0; i < MAP_CELLS; i++) {
    nav_fn[i] = INIT_LPN_COST;
  }

  priority_queue<pair<Point,double>, vector<pair<Point,double> >, PairComparator > pq;

  pq.push(make_pair(goal,0.0));
  setVal(nav_fn,goal.x,goal.y,0.0);

  pair<Point,double> curr_pq_entry;
  Point curr_pt;
  double curr_cost;
  double curr_intrinsic;
  Point nbrs[4];
  int num_nbrs;
  Point nbr;

  int counter = 0;
  ros::Rate rate(50);
  while(!pq.empty()) {
    curr_pq_entry = pq.top();
    pq.pop();

    curr_pt = curr_pq_entry.first;
    curr_cost = curr_pq_entry.second;

    // if nav_fn is less than the pq cost, it means this node was already
    // enqueued by some other neighbor which gave it lower value
    if (curr_cost > getVal(nav_fn,curr_pt.x,curr_pt.y)) {continue;}
    //ROS_INFO("[LPN]\tpopping Point(%d,%d)",curr_pt.x,curr_pt.y);
    
    // deal with neighbors
    num_nbrs = get4Nbrs(curr_pt,nbrs);
    curr_intrinsic = getVal(intrinsic_costs, curr_pt.x, curr_pt.y);
    //ROS_INFO("[updatedMapDijkstra]\tnum_nbrs=%d",num_nbrs);
    for (int j = 0; j < num_nbrs; j++) {
      nbr = nbrs[j];
      // compute plane wave approx from the neighbors of nbr which have already been visited
      double new_cost = -1;
      if (nbr.dist(goal) == 1) {
	new_cost = curr_intrinsic;
      } else {
	new_cost = planeWaveApprox4NbrNew(nbr,nav_fn,curr_intrinsic);
      }
      
      //if (new_cost < getVal(nav_fn,nbr.x,nbr.y)) {
      if (new_cost / getVal(nav_fn,nbr.x,nbr.y) < 1 - 0.000001) {
	//ROS_INFO("[LPN]\tupdating nbr nav_fn[%d,%d] from %f to %f",nbr.x,nbr.y,getVal(nav_fn,nbr.x,nbr.y),new_cost);
	setVal(nav_fn,nbr.x,nbr.y,new_cost);
	//ROS_INFO("[LPN]\tenqueuing Point(%d,%d) with value: %f",nbr.x,nbr.y,new_cost);
	//insertAndPurgeDups(pq,make_pair(nbr,new_cost));
	pq.push(make_pair(nbr,new_cost));
      }
    }
    if (counter % 10000 == 0) {
      //ROS_INFO("[LPN]\tpq.size() = %d", pq.size());
      //ROS_INFO("[LPN]\tcurr_active_set.size()=%d",curr_active_set.size());
      //visualizeActiveSet(pq,nav_fn);
      //visualizeHeatMap(nav_fn,"nav_fn_partial");
      //rate.sleep();
    }
    counter++;
  }

  // for (int i = 0; i < MAP_CELLS;i++) {
  //   if (nav_fn[i] < 0) {
  //     ROS_INFO("[LPN]\tfound weird val: nav_fn[%d]= %f",i,nav_fn[i]);
  //   }
  // }
}

// compute nav fn (using wavefront assumption)
void getNavFn(double* cost_map, tf::StampedTransform &robot_to_map_transform, double* nav_fn) {
  Point robot_pt(robot_to_map_transform.getOrigin().getX() / MAP_RESOLUTION,robot_to_map_transform.getOrigin().getY() / MAP_RESOLUTION);
  Point goal_pt(goal_pose.getOrigin().getX() / MAP_RESOLUTION, goal_pose.getOrigin().getY() / MAP_RESOLUTION);
  ROS_INFO("[getNavFn]\trobot_pt: (%d,%d), goal_pt(%d,%d)",robot_pt.x,robot_pt.y,goal_pt.x,goal_pt.y);

  // setup init map
  bool init_set_map[MAP_CELLS];
  for (int i=0; i<MAP_CELLS;i++) {
    init_set_map[i] = false;
  }
  setVal(init_set_map,goal_pt.x,goal_pt.y,true);

  //updateMapDijkstra(nav_fn, init_set_map, goal_pt, cost_map, INT_MAX);
  LPN2(nav_fn, cost_map, goal_pt, robot_pt);
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr msg) {
  // populate map global along with dimensions
  ROS_INFO("[map_callback]\tmsg.info.width=%d, msg.info.width=%d, resolution=%f", msg->info.width,msg->info.height,msg->info.resolution);
  MAP_SIZE_X = msg->info.width;
  MAP_SIZE_Y = msg->info.height;
  MAP_CELLS = MAP_SIZE_X * MAP_SIZE_Y;
  MAP_RESOLUTION = msg->info.resolution;
  global_map = *msg;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  // populate goal_pose global
  ROS_INFO("[goal_callback]\tmsg.header.frame_id: %s", msg->header.frame_id.c_str());
  tf::poseStampedMsgToTF(*msg, goal_pose);
}

void testDijkstras() {
  bool init_set_map[MAP_CELLS];
  double obstacle_dist_map[MAP_CELLS];
  double edge_cost_map[MAP_CELLS];
  for (int i=0; i < MAP_CELLS; i++) {
    init_set_map[i]=false;
    obstacle_dist_map[i] = -1;
    edge_cost_map[i] = 1;
  }
  init_set_map[0] = true; // keep things simple; one obstacle at (0,0)

  for (int i=0; i < MAP_CELLS; i++) {
    if (init_set_map[i]) {
      ROS_INFO("[testDijkstra]\tfound true value in init_set_map");
    }
  }

  Point goal(MAP_SIZE_X - 1, MAP_SIZE_Y - 1);
  int MAX_DIST = 10;
  updateMapDijkstra(obstacle_dist_map, init_set_map, goal,edge_cost_map, MAX_DIST);

  ROS_INFO("[testDijkstras]\tnew iteration");
  for (int i=0; i < MAP_CELLS; i++) {
    if (obstacle_dist_map[i] != -1) {
      Point conversion(i);
      ROS_INFO("[testDijkstras]\tPoint(%d,%d) has dist: %f",conversion.x,conversion.y,obstacle_dist_map[i]);
    }
  }

  visualizeHeatMap(obstacle_dist_map,"testDijkstra");
}

void testPQ() {
  priority_queue<pair<Point,double>, vector<pair<Point,double> >, PairComparator > pq;  
  
  pq.push(make_pair(Point(0),6));
  pq.push(make_pair(Point(0),4));
  pq.push(make_pair(Point(0),10));
  pq.push(make_pair(Point(0),1));
  pq.push(make_pair(Point(0),3));

  pair<Point,double> curr_pq_elem;
  while (!pq.empty()) {
    curr_pq_elem = pq.top();
    ROS_INFO("[testPQ]\tpopped sort val: %f", curr_pq_elem.second);
    pq.pop();
  }
}

int main(int argc, char** argv) {
  // init node
  ros::init(argc, argv, "planner_simple");    
  ros::NodeHandle nh;
  
  // set up subscribers, publishers and listeners
  ros::Subscriber goal_sub = nh.subscribe("/goal", 1000, goalCallback);
  ros::Subscriber map_sub = nh.subscribe("/map", 1000, mapCallback);
  //ros::Subscriber obstacle_map_sub = nh.subscribe("/obstacle_map", 1000, mapCallback);
  marker_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  map_pub = nh.advertise<assn2::NavGrid>( "nav_grid", 0 );
  tf::TransformListener listener;  
  listener.waitForTransform("/map", "/base_link", ros::Time(), ros::Duration(1.5));
  
  // set initial goal to be the robot's pose
  try {
    tf::StampedTransform map_base_link_transform; //lookup transform only works for this type
    listener.lookupTransform("/map", "/base_link", ros::Time(0), map_base_link_transform);
    goal_pose.setOrigin(map_base_link_transform.getOrigin());
    goal_pose.setRotation(map_base_link_transform.getRotation());
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
  }
  
  ros::Rate rate(10);
  while (ros::ok()) {
    // populates the goal_pose and global_map if a new message has been published
    ros::spinOnce(); 

    double cost_map[MAP_CELLS];
    getCostFn(cost_map);
    visualizeHeatMap(cost_map,"intrinsic_cost_map");

    double nav_fn[MAP_CELLS];
    tf::StampedTransform robot_to_map_transform; // the pose of the robot in the map frame
    tf_util::getTransform(listener,"/base_link","/map",robot_to_map_transform);
    getNavFn(cost_map,robot_to_map_transform, nav_fn);
    visualizeHeatMap(nav_fn,"nav_fn");

    publishMap(nav_fn);
    rate.sleep();
  }
}
