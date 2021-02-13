#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
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
#include "std_msgs/Int32.h"

using namespace std;

  ros::Publisher pub;
  ros::Subscriber sub;
  bool trovato=false;

void map_cb (const nav_msgs::OccupancyGridConstPtr msg) {
    std_msgs::Header header=msg->header;
    nav_msgs::MapMetaData info=msg->info;
    int width=info.width;
    int height=info.height;
    vector <int8_t> proof;
    for (unsigned int x = 0; x < height; x++) {
        for (unsigned int y = 0; y < width; y++) {
             if(msg->data[y+width*x]==0 || msg->data[y+width*x] ==100){
                proof.push_back(msg->data[y+width*x]);
             }
             else 
                proof.push_back(100);
        }
    }
    nav_msgs::OccupancyGrid grid;
    grid.header.stamp=ros::Time::now();
    grid.header.frame_id=header.frame_id;
    grid.info=info;
    grid.data = proof;
    pub.publish(grid);
    ROS_INFO("pubblicato");
    trovato=true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "convert");
  ros::NodeHandle n;
  pub = n.advertise <nav_msgs::OccupancyGrid>("map_out",0);
  sub = n.subscribe ("map",0, map_cb);
  while(!trovato) {
    ros::spinOnce();
  }
}
