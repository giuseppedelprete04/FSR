#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"
#include <vector>
#include <math.h>
#include "boost/thread.hpp"
#include <tf/tf.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "std_msgs/Int32.h"
#include "../include/LowPassFilter.hpp"
#include "../include/spline_planner.h"
#define dx 0.05
#define dy 0.05
#define dt 0.01
#define mul 1
#define x_offset -12.2
#define y_offset -15.4
#define influence 7
#define resolution 0.05
//#define unit mul/100
using namespace std;

struct Gradient {
       vector <double> x_gradient;
       vector <double> y_gradient;
};

struct cell {
   int num;
   int x;
   int y;
};

class Planner {
       private:
	  ros::NodeHandle n;
          ros::Publisher attract_pub;
          ros::Publisher rep_pub;
          ros::Publisher total_pub;
          ros::Publisher path_pub;
          ros::Publisher grad_pub;
          ros::Publisher vel_pub;
          ros::Publisher pos_pub;
          ros::Publisher map_pub;
          ros::Subscriber map_sub;
          nav_msgs::Path path;    
          vector<geometry_msgs::TwistStamped> velocities;
          double ** newdata;
          double ** repulsive_mat;
          double ** attractive_mat;
          double ** total_mat;
          double ** min_potential;
          int ** passato;
          bool find;
          int width;
          int height;
          std_msgs::Header header;
          nav_msgs::MapMetaData info;
          Gradient gradient;
          int init_mat[2];
          int goal_mat[2];
       public:
          Planner ();
          ~Planner();
          void plan(double x, double y);
          void set_goal (double x_pose, double y_pose);
          void set_init (double x_pose, double y_pose);
          void map_cb(const nav_msgs::OccupancyGridConstPtr msg);
          double dist (double x1, double x2, double y1, double y2);
          double evaluate_rep_potential (double r_influence, double gamma, double gain, double dist);
          void rep_potential (int x, int y);
          double evaluate_attract_potential (double gain, double dist);
          void compute_attract_potential();
          bool trovato;
          void compute_rep_potential();
          void compute_total_potential();
          void compute_scale_mat(double min);
          void min_path();
          void set_gradient(int x, int y);
          void compute_gradient();
          void integrate_gradient();
          vector <geometry_msgs::PoseStamped> poses_traj;
          vector <geometry_msgs::TwistStamped> vel_traj;
          void new_mat();
          vector <cell> nav_fun(int i, int j);

};	       







  
