#include "../include/planner2.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#define b 0.1
#define k1 5
#define k2 5
#define max_ang 1.5
#define wheel_radius (0.179/2.0)
#define wheel_dist 0.335


class Controller {
      private:
             ros::NodeHandle nh;
             ros::Subscriber pos_sub;
             ros::Subscriber joint_sub;
             ros::Publisher  vel_pub;
             ros::Publisher planned_vel;
             ros::Publisher planned_pos;
             ros::Publisher lin_vel;
             ros::Publisher ang_vel;
             ros::Publisher tracking_error;
             ros::Publisher yaw_error;
             ros::Publisher real_dist;
             vector <double> des_vel_x;
             vector <double> des_vel_y;
             vector <double> des_pos_x;
             vector <double> des_pos_y;
             double yaw;
             double yawdot;
             double position[2];
             double odom_pos[3];
             double orientation[3];
             double init_config[4];
             double left_wheel;
             double right_wheel;
             int end;
      public:
             Controller ();
             bool trovato;
             void odom_cb (const nav_msgs::OdometryConstPtr msg);
             void joint_cb(const std_msgs::Float32MultiArrayConstPtr msg);
             void des_pos_cb (const geometry_msgs::PoseStampedConstPtr msg);
             void des_vel_cb (const geometry_msgs::TwistStampedConstPtr msg);
             double get_x() {return position[0];}
             double get_y() {return position[1];}
             double get_yaw() {return orientation[2];}
             void set_path(vector<geometry_msgs::PoseStamped> pos, vector <geometry_msgs::TwistStamped> vel);
             void control (int i);
             void estimate_yaw ();
             void run ();
             void thread();

};

Controller::Controller() {
      trovato=false;
      end=0;
      vel_pub = nh.advertise <geometry_msgs::Twist>("cmd_vel",0);
      pos_sub = nh.subscribe("odom",0,&Controller::odom_cb,this);
      joint_sub = nh.subscribe("diff_wheels/vel",0,&Controller::joint_cb,this);
      planned_vel = nh.advertise <std_msgs::Float64>("velocity",0);
      yaw_error = nh.advertise <std_msgs::Float64>("yaw_err",0);
      real_dist = nh.advertise <std_msgs::Float64>("dist",0);
      planned_pos = nh.advertise <geometry_msgs::PointStamped>("position",0);
      lin_vel = nh.advertise <std_msgs::Float64>("cmd_lin",0);
      ang_vel = nh.advertise <std_msgs::Float64>("cmd_ang",0);
      tracking_error = nh.advertise <std_msgs::Float64MultiArray> ("track_error",0);
}

void Controller:: odom_cb (const nav_msgs::OdometryConstPtr msg) {
     	   position[0]=msg->pose.pose.position.x;
     	   position[1]=msg->pose.pose.position.y;
     	   tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,  msg->pose.pose.orientation.w);
     	   tf::Matrix3x3 Rot_mat(q);
     	   Rot_mat.getRPY(orientation[0],orientation[1],orientation[2]);
     	   yaw=orientation[2];
           if(!trovato) { 
              odom_pos[0]=position[0];
              odom_pos[1]=position[1];
              odom_pos[2]=yaw;
           }
           yawdot=-msg->twist.twist.angular.z;
           trovato=true;
}

void Controller::joint_cb(const std_msgs::Float32MultiArrayConstPtr msg) {
           left_wheel=msg->data[0];
           right_wheel=msg->data[1];

}

void Controller::estimate_yaw () {
     ros::Rate r(1000);
     while(ros::ok()) {
     if(end<10) {
     double w_k=-(wheel_radius/wheel_dist)*(right_wheel-left_wheel);
     odom_pos[2]=odom_pos[2]+(w_k*0.001);
     if(odom_pos[2]<-M_PI) odom_pos[2]+=(2*M_PI);
     if (odom_pos[2]>M_PI) odom_pos[2]-=(2*M_PI);
      end++;
      r.sleep();
     }
     }
}

void Controller:: thread () {
     boost::thread estimate_yaw_t( &Controller::estimate_yaw, this);
}

void Controller:: control(int i) {
     double real_distance=0.0;
     std_msgs::Float64 msg;
     std_msgs::Float64 distance;
     std_msgs::Float64 yaw_err;
     std_msgs:: Float64MultiArray err;
     double local_yaw=odom_pos[2];
     double v_k=(wheel_radius/2)*(right_wheel+left_wheel);
     double w_k=-(wheel_radius/wheel_dist)*(right_wheel-left_wheel);
     odom_pos[0]=odom_pos[0]+(v_k*dt*cos(local_yaw+((w_k*dt)/2)));
     odom_pos[1]=odom_pos[1]+(v_k*dt*sin(local_yaw+((w_k*dt)/2)));
     cout << "Yaw : " << yaw << " " << odom_pos[2] << endl;
     /*cout << "X : " << position[0] << " " << odom_pos[0] << endl;
     cout << "Y : " << position[1] << " " << odom_pos[1] << endl;*/
     yaw_err.data=abs(yaw)-abs(odom_pos[2]);
     double x=odom_pos [0];
     double y=odom_pos [1];
     double linear_vel=0.0;
     double angular_vel=0.0;
     double pseudo_vel[2];
     double y1=x+b*cos(local_yaw);
     double y2=y+b*sin(local_yaw);
     double T_inv [2][2];
     double y1d=des_pos_x[i]+b*cos(local_yaw);
     double y2d=des_pos_y[i]+b*sin(local_yaw);
     double y1ddot=des_vel_x[i]+w_k*b*sin(local_yaw);
     double y2ddot=des_vel_y[i]-w_k*b*cos(local_yaw);
     T_inv[0][0]=cos(local_yaw);
     T_inv[0][1]=sin(local_yaw);
     T_inv[1][0]=-(sin(local_yaw)/b);
     T_inv[1][1]=cos(local_yaw)/b;
     pseudo_vel[0]=/*y1ddot+*/k1*(y1d-y1);
     pseudo_vel[1]=/*y2ddot+*/k2*(y2d-y2);
     //cout << y1d-y1 << " " << y2d-y2 << endl;
     linear_vel=T_inv[0][0]*pseudo_vel[0]+T_inv[0][1]*pseudo_vel[1];
     angular_vel=T_inv[1][0]*pseudo_vel[0]+T_inv[1][1]*pseudo_vel[1];
     err.data.push_back(y1d-y1);
     err.data.push_back(y2d-y2);
     tracking_error.publish(err);
     msg.data=linear_vel;
     lin_vel.publish(msg);
     msg.data=angular_vel;
     ang_vel.publish(msg);
     geometry_msgs::Twist velocities;
     velocities.linear.x=linear_vel;
     velocities.angular.z=angular_vel;
     real_distance=(wheel_radius/yawdot)*(right_wheel-left_wheel);
     vel_pub.publish(velocities);
     yaw_error.publish(yaw_err);
     distance.data=real_distance;
     real_dist.publish(distance);
}

void Controller::set_path(vector<geometry_msgs::PoseStamped> pos, vector <geometry_msgs::TwistStamped> vel) {
     for (int i=0; i<vel.size(); i++) {
          des_pos_x.push_back(pos[i].pose.position.x);
          des_pos_y.push_back(pos[i].pose.position.y);
          des_vel_x.push_back(vel[i].twist.linear.x);
          des_vel_y.push_back(vel[i].twist.linear.y);
          if(i==vel.size()-1) {
             des_vel_x[i]=0;
             des_vel_y[i]=0;
          } 
     }
}
void Controller:: run() {
      geometry_msgs::PointStamped mess;
      std_msgs::Float64 msg;
      ros::Rate r(100);
      int i=0;
      while (ros::ok()) {
          if(end==10) {
             control(i);
             i++;
             msg.data=des_vel_x[i];
             planned_vel.publish(msg);
             mess.point.x=des_pos_x[i];
             mess.point.y=des_pos_y[i];
             mess.header.stamp=ros::Time::now();
             planned_pos.publish(mess);
             if (i>=des_vel_x.size()) {
                i=des_vel_x.size()-1;
             } 
             end=0;
             r.sleep();
           }
      }



      /*for (int i=0;i<des_vel_x.size();i++) {
          control (i);
          r.sleep();
      }
      for (int i=0;i<10;i++) {
          geometry_msgs::Twist velocities;
          velocities.linear.x=0;
          velocities.angular.z=0;
          vel_pub.publish(velocities);
          r.sleep();
      }*/
}



int main(int argc, char **argv){
  ros::init(argc, argv, "controller");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  Planner p;
  Controller c;
  while (p.trovato!=true || c.trovato!=true) {
       usleep(1000);

  }
  p.set_init(c.get_y()-y_offset,c.get_x()-x_offset);
  p.plan(12.5,10);
  c.set_path(p.poses_traj,p.vel_traj);
  c.thread();
  c.run();
  return 0;
}
