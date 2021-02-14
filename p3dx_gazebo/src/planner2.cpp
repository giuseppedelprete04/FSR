#include "../include/planner2.h"



Planner::Planner()  {
   trovato =false;
   rep_pub = n.advertise<nav_msgs::OccupancyGrid>("rep_map",0);
   attract_pub = n.advertise<nav_msgs::OccupancyGrid>("attract_map",0);
   total_pub = n.advertise<nav_msgs::OccupancyGrid>("total_map",0);
   path_pub = n.advertise<nav_msgs::OccupancyGrid>("path_map",0);
   grad_pub = n.advertise <nav_msgs::Path>("pose_map",0);
   vel_pub = n.advertise <geometry_msgs::TwistStamped>("vel",0);
   pos_pub = n.advertise <geometry_msgs::PoseStamped>("pos",0);
   map_pub = n.advertise <nav_msgs::OccupancyGrid>("maps",0);
   map_sub = n.subscribe("map",0,&Planner::map_cb,this);
}





void Planner::map_cb(const nav_msgs::OccupancyGridConstPtr msg){
  header = msg->header;
  info = msg->info;
  width=info.width;
  height=info.height;
  newdata=new double*[height];
  repulsive_mat=new double*[height];
  attractive_mat=new double*[height];
  total_mat=new double*[height];
  min_potential=new double*[height];
  passato=new int*[height];
  for (int i=0; i<height;i++) {
      newdata[i]=new double [width];
      repulsive_mat[i]=new double[width];
      attractive_mat[i]=new double[width];
      total_mat[i]=new double[width];
      min_potential[i]=new double[width];
      passato[i]=new int[width];
  }

  for (unsigned int x = 0; x < height; x++) {
    for (unsigned int y = 0; y < width; y++) {
	newdata[x][y]=msg->data[y+width*x];
        repulsive_mat[x][y]=msg->data[y+width*x];
        attractive_mat[x][y]=msg->data[y+width*x];
        passato[x][y]=0;
    }
  }
 
    ROS_INFO("Got map %d %d", width, height); 

  trovato = true;
}


double Planner::dist (double x1, double x2, double y1, double y2) {
	return resolution*sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

double Planner::evaluate_rep_potential (double r_influence, double gamma, double gain, double dist) {
       return (gain/gamma)*pow(((1/dist)-(1/r_influence)),gamma);
}

double Planner::evaluate_attract_potential (double gain, double dist) {
       if (dist>1) {
         return gain*dist;
      }
      else { 
         return 0.5*gain*pow(dist,2);

      }
}



void Planner::rep_potential (int x, int y) {
       int i=0;
       int j=0;
       int count=1;
       double distance =0.0;
       std::vector<double> punto;
       punto.resize(3);
       std::vector<std::vector<double>> punti_pieni;
       while(count<influence) {
           i=x-count;
           j=y+count;
           for(; i<=(x+count); i++) {
               if (repulsive_mat[i][j] == 100){
                    distance = dist (i,x,j,y);
                    if (distance<influence*resolution){
                     punto[0] = i; punto[1]=j; punto[2]=distance;
                     punti_pieni.push_back(punto);
                    }
               }
           }
           for(; j>=y-count; j--) {
               if (repulsive_mat[i][j] == 100) {
                    distance = dist (i,x,j,y);
                   if (distance<influence*resolution){
                     punto[0] = i; punto[1]=j; punto[2]=distance;
                     punti_pieni.push_back(punto);
                    }
               }
           }
           for(; i>=x-count; i--) {
               if (repulsive_mat[i][j] == 100) {
                    distance = dist (i,x,j,y);
                   if (distance<influence*resolution){
                     punto[0] = i; punto[1]=j; punto[2]=distance;
                     punti_pieni.push_back(punto);
                    }
               }
           }
           for(; j<=y+count; j++) {
               if (repulsive_mat[i][j] == 100) {
                    distance = dist (i,x,j,y);
                    if (distance<influence*resolution){
                     punto[0] = i; punto[1]=j; punto[2]=distance;
                     punti_pieni.push_back(punto);
                    }
               }
           }
           
           count++;
       }
       if (punti_pieni.size ()<=0) return;
       double min=1000;
       int min_index=-1;
       for(int i=0; i<punti_pieni.size(); i++) {
           if(punti_pieni[i][2]<min) {
               min_index = i;
           }
       }
       int imin = punti_pieni[min_index][0];
       int jmin = punti_pieni[min_index][1];
       double distancemin = punti_pieni[min_index][2];
       repulsive_mat[x][y]=evaluate_rep_potential(influence,2,4,distancemin);
}

void Planner::compute_rep_potential() {
 int count =0;
 vector <int8_t> proof2;
 for (unsigned int x = 0; x < height; x++) {
    for (unsigned int y = 0; y < width; y++) {
        if (repulsive_mat[x][y]==0) {
           rep_potential(x,y);
        }
        proof2.push_back(round(repulsive_mat[x][y]));
    }
  }
  cout << count << " " << width*height << endl;
  nav_msgs::OccupancyGrid grid;
  grid.header.stamp=ros::Time::now();
  grid.header.frame_id=header.frame_id;
  grid.info=info;
  grid.data = proof2;
  rep_pub.publish(grid);
  ROS_INFO("potenziali repulsivi pubblicati");
  }


void Planner::compute_attract_potential () {
     vector <int8_t> proof;
     double distance=0.0;
     for (int x=0;x<height;x++) {
         for (int y=0; y<width;y++) {
            if (attractive_mat[x][y]==0) {
                distance=dist(goal_mat[0],x,goal_mat[1],y);
                attractive_mat[x][y]=evaluate_attract_potential(5,distance);
            }
         }
     }
     for (unsigned int x = 0; x < height; x++) {
         for (unsigned int y = 0; y < width; y++) {
            proof.push_back(round(attractive_mat[x][y]));
         }
     }
     nav_msgs::OccupancyGrid grid;
     grid.header.stamp=ros::Time::now();
     grid.header.frame_id=header.frame_id;
     grid.info=info;
     grid.data = proof;
     attract_pub.publish(grid);
     ROS_INFO("potenziali attrattattivi pubblicati");
}

void Planner::compute_scale_mat (double min) {
     double max=100;
     double scale_factor=0.0;
     for (int x=0;x<height;x++) {
         for (int y=0; y<width;y++) {
             total_mat[x][y]=total_mat[x][y]-min;
             if(total_mat[x][y] > max) max=total_mat[x][y];
         }
     }
     scale_factor=(double)100/max;
     for (int x=0;x<height;x++) {
         for (int y=0; y<width;y++) {
             total_mat[x][y]=(total_mat[x][y]*scale_factor);
             min_potential[x][y]=total_mat[x][y];
         }
     }
} 

void Planner::compute_total_potential () {
     vector <int8_t> proof;
     double min =0.0;
     for (int x=0;x<height;x++) {
         for (int y=0; y<width;y++) {
             if(attractive_mat[x][y]!=100 && repulsive_mat[x][y]!=100) {
               total_mat[x][y]= repulsive_mat[x][y]+attractive_mat[x][y];
               total_mat[x][y]=total_mat[x][y];
               if (total_mat[x][y] < min) min=total_mat[x][y];
               }
             else {
               total_mat[x][y]=100;
             }
         }
     }	
     compute_scale_mat(min);
     for (unsigned int x = 0; x < height; x++) {
         for (unsigned int y = 0; y < width; y++) {
            proof.push_back(round(total_mat[x][y]));
         }
     }

     nav_msgs::OccupancyGrid grid;
     grid.header.stamp=ros::Time::now();
     grid.header.frame_id=header.frame_id;
     grid.info=info;
     grid.data = proof;
     total_pub.publish(grid);
     ROS_INFO("potenziali totali pubblicati"); 
}

void Planner::min_path() {
     int index_equal=1;
     int count =1;
     int x=init_mat[0];
     int y=init_mat[1];
     int imin=0;
     int jmin=0;
     int x_index=0;
     int y_index=0;
     std::vector<double> punto;
     punto.resize(2);
     double min=total_mat[init_mat[0]][init_mat[1]];
     vector <int8_t> proof;
     while ((x!=goal_mat[0] || y!=goal_mat[1]) && (index_equal==1)) {
           for (int i=x-count;i<=x+count;i++) {
               for (int j=y+count;j>=y-count;j--) {
                   if(min>total_mat[i][j]) {
                     min=total_mat[i][j];
                     punto[1]=i;
                     punto[2]=j;
                   }
               }
           imin=(int)punto[1];
           jmin=(int)punto[2];
           min_potential[imin][jmin]=200;
           }
           if(x_index !=imin || y_index !=jmin) {
              x_index=imin;
              y_index=jmin;
           }
           else {
                index_equal=2;
           }
           x=imin;
           y=jmin;
     }
     for (unsigned int x = 0; x < height; x++) {
         for (unsigned int y = 0; y < width; y++) {
            proof.push_back(round(min_potential[x][y]));
         }
     }
     nav_msgs::OccupancyGrid grid;
     grid.header.stamp=ros::Time::now();
     grid.header.frame_id=header.frame_id;
     grid.info=info;
     grid.data = proof;
     path_pub.publish(grid);
     ROS_INFO("path minimo pubblicato"); 
               
}

Planner::~Planner() {
	for (int i=0;i<height;i++) {
            delete[] newdata[i];
            delete[] repulsive_mat[i];
            delete[] attractive_mat[i];
            delete[] total_mat[i];
            delete[] min_potential[i];
        }
        delete[] newdata;
        delete[] repulsive_mat;
        delete[] attractive_mat;
        delete[] total_mat;
        delete[] min_potential;
}

void Planner::set_gradient (int x, int y) {
      double norm=0.0;
      double x_grad=0.0;
      double y_grad=0.0;
      double scale=0.0;
      if (total_mat[x][y]!=100) {
         x_grad=((total_mat[x+1][y-1]+2*total_mat[x+1][y]+total_mat[x+1][y+1])-(total_mat[x-1][y-1]+2*total_mat[x-1][y]+total_mat[x-1][y+1]))/dx;
         y_grad=((total_mat[x-1][y+1]+2*total_mat[x][y+1]+total_mat[x+1][y+1])-(total_mat[x-1][y-1]+2*total_mat[x][y-1]+total_mat[x+1][y-1]))/dy;
         norm=sqrt(pow(x_grad,2)+pow(y_grad,2));
         scale=abs(0.15/x_grad);
         if (scale>abs(0.15/y_grad)) scale =abs(0.15/y_grad);
         if(scale<1) {
           x_grad*=scale;
           y_grad*=scale;         
         }
            gradient.x_gradient.push_back(x_grad);
            gradient.y_gradient.push_back(y_grad);
      }
      else {
         gradient.x_gradient.push_back (0);
         gradient.y_gradient.push_back (0);
         }
}


void Planner::compute_gradient () {
    for (unsigned int x = 0; x < height; x++) {
       for (unsigned int y = 0; y < width; y++) {
           set_gradient(x,y);
       }
     }
}

void Planner::integrate_gradient () {
     ros::Rate r(100);
     vector <int8_t> proof;
     double eps =0.01, veleps=0.25;
     geometry_msgs::TwistStamped vel;
     geometry_msgs::PoseStamped pos;
     path.header.frame_id=header.frame_id;
     vel.header.frame_id="map";
     pos.header.frame_id="map";
     float x1=init_mat[1]*resolution;
     float y1=init_mat[0]*resolution;
     float x2=goal_mat[1]*resolution;
     float y2=goal_mat[0]*resolution;
     int x_mat=init_mat[0];
     int y_mat=init_mat[1];
     int count =1, durationCount=0;
     double min =0.0;
     double x_ind=x1;
     double y_ind=y1;
     pos.pose.position.x=x1+x_offset;
     pos.pose.position.y=y1+y_offset;
     path.poses.push_back(pos);
     double freq=5;
     LowPassFilter lpfx((freq/(2*M_PI)),dt);
     LowPassFilter lpfy((freq/(2*M_PI)),dt);
     bool stuck=false;
     while ((abs(x1-x2)>eps || abs(y1-y2)>eps) && count<100000) {
           lpfx.update(gradient.y_gradient[y_mat+width*x_mat]);
           lpfy.update(gradient.x_gradient[y_mat+width*x_mat]);
           x1=(x1-lpfx.getOutput()*dt);
           y1=(y1-lpfy.getOutput()*dt);
           pos.pose.position.x=x1+x_offset;
           pos.pose.position.y=y1+y_offset;
           pos.pose.position.z=0;
           vel.twist.linear.x=lpfx.getOutput();
           vel.twist.linear.y=lpfy.getOutput();
           if( (round(y1/resolution)!=x_mat) || (round(x1/resolution)!=y_mat) )
               passato[x_mat][y_mat]++;
           x_mat=round(y1/resolution);
           y_mat=round(x1/resolution); 
           count ++;
           durationCount++; 
           double x_nav=0.0;
           double y_nav=0.0;
           if (passato[x_mat][y_mat]>=50) { //attivo navfun
               ROS_WARN("STUCK");
               if(!stuck) { //Se non vengo già da una navfun cancello
                   cout << path.poses.size() << endl;
                   path.poses.erase(path.poses.end()-300,path.poses.end());
                   velocities.erase(velocities.end()-300,velocities.end());
                   cout << path.poses.size() << endl;
                   x1 = path.poses.back().pose.position.x-x_offset;
                   y1 = path.poses.back().pose.position.y-y_offset;
                   x_nav=x_mat;
                   y_nav=y_mat;
                   x_mat=round(y1/resolution);
                   y_mat=round(x1/resolution);
               } else { //altrimenti non cancello e mi prendo l'ultimo punto buono
                   x1 = path.poses.back().pose.position.x-x_offset;
                   y1 = path.poses.back().pose.position.y-y_offset;
                   x_mat=round(y1/resolution);
                   y_mat=round(x1/resolution);
               }
               stuck=true;
           } else { //non ho bisogno di navfun
               stuck=false;
               path.poses.push_back(pos);
               velocities.push_back(vel);
           }
          if (stuck) {
               cout<<"Bloccato"<<endl;
               std:: vector <cell> percorso;
               percorso=nav_fun(x_nav,y_nav); 
               SPLINE_PLANNER xplan(100),yplan(100);
               vector<double> x_points, y_points;
               vector<double> times;
               int tf=5; //10secondi
               double t=0;
               times.push_back(0);
               times.push_back(tf);
               x_points.push_back(x1);
               y_points.push_back(y1);
               x_points.push_back((percorso[0].y)*resolution);
               y_points.push_back((percorso[0].x)*resolution);
               xplan.set_waypoints(x_points,times);
               yplan.set_waypoints(y_points,times);
               xplan.compute_traj();
               yplan.compute_traj();
               x_mat = percorso[0].x;
               y_mat = percorso[0].y;
               x1=y_mat*resolution;
               y1=x_mat*resolution;
               bool nonfinito;
               do {
                   double xt,xtd,xtdd,yt,ytd,ytdd;
                   nonfinito = xplan.getNext(xt,xtd,xtdd) && yplan.getNext(yt,ytd,ytdd);
                   if(nonfinito) {
                       if( (round(yt/resolution)!=x_mat) || (round(xt/resolution)!=y_mat) )
                           passato[x_mat][y_mat]++;
                       x_mat=round(yt/resolution);
                       y_mat=round(xt/resolution);
                       pos.pose.position.x=xt+x_offset;
                       pos.pose.position.y=yt+y_offset;
                       pos.pose.position.z=0;
                       path.poses.push_back(pos);
                       vel.twist.linear.x=xtd;
                       vel.twist.linear.y=ytd;
                       velocities.push_back(vel);
                       lpfx.update(xtd);
                       lpfy.update(ytd);
                       count++;
                   }
               }
               while(nonfinito);
     
              
              cout<<"Bloccato"<<endl;
       
     }
   }
     cout << "Tempo necessario al calcolo : " << count*dt << endl;
     for (unsigned int x = 0; x < velocities.size(); x++) {
             vel.header.stamp=ros::Time::now();
             pos.header.stamp=ros::Time::now();
             //vel_pub.publish(velocities[x]);
             //pos_pub.publish(path.poses[x]);
             poses_traj.push_back(path.poses[x]);
             vel_traj.push_back(velocities[x]);
         }
     
     path.header.stamp=ros::Time::now();
     grad_pub.publish(path);
     ROS_INFO("path estratto");
}

vector <cell> Planner::nav_fun (int g, int j) {
     int max_index=0;
     std::vector <int8_t> proof;
     std::vector <int> indici;
     int threshold=2;
     cell c;
     int count =0;
     double init_pot=total_mat[g][j];
     std:: vector <cell> gridmap;
     std:: vector <cell> map;
     int x_index;
     int y_index;
     c.x=g;
     c.y=j;
     c.num=0;
     gridmap.push_back(c);
     int x_start=c.x;
     int y_start=c.y;
     int x_in=c.x;
     int y_in=c.y;
     int iteration=0;
     while (iteration<2000) {
      if(newdata[x_start][y_start]!=100) {
        if(repulsive_mat[x_start][y_start-1]<=threshold) {
          c.num=gridmap[0].num+1;
          newdata[x_start][y_start-1]=gridmap[0].num+5;
          c.x=x_start;
          c.y=y_start-1;
          gridmap.push_back(c);
          map.push_back(c);
        }
         if(repulsive_mat[x_start-1][y_start]<=threshold) {
          c.num=gridmap[0].num+1;
          newdata[x_start-1][y_start]=gridmap[0].num+5;
          c.x=x_start-1;
          c.y=y_start;
          gridmap.push_back(c);
          map.push_back(c);
        }
         if(repulsive_mat[x_start][y_start+1]<=threshold) {
          c.num=gridmap[0].num+1;
          newdata[x_start][y_start+1]=gridmap[0].num+5;
          c.x=x_start;
          c.y=y_start+1;
          gridmap.push_back(c);
          map.push_back(c);
        }
         if(repulsive_mat[x_start+1][y_start]<=threshold) {
          c.num=gridmap[0].num+1;
          newdata[x_start+1][y_start]=gridmap[0].num+5;
          c.x=x_start+1;
          c.y=y_start;
          gridmap.push_back(c);
          map.push_back(c);
        }
      } 
      if(max_index<map[map.size()-1].num) max_index=map[map.size()-1].num;
      gridmap.erase(gridmap.begin());
      x_start=gridmap[0].x;
      y_start=gridmap[0].y;
      iteration++;
     }
      double min_pot=init_pot;
      double min_index=0;
      for(int ii=0; ii<map.size(); ii++) {
          if(total_mat[map[ii].x][map[ii].y]<min_pot) {
              min_index = ii;
              min_pot = total_mat[map[ii].x][map[ii].y];
          }
          
      }
     
      indici.push_back(map[min_index].x);
      indici.push_back(map[min_index].y);
      std:: vector <cell> percorso;
      percorso.push_back(map[min_index]);
      max_index = map[min_index].num;
      for(int i=(map.size()-1); i>=0; i--) {
           if(map[i].num==(max_index-1) && ( (abs(map[i].x - percorso.back().x) <= 1) && (abs(map[i].y - percorso.back().y) <= 1) ) ) {
               percorso.push_back(map[i]);
               max_index--;
           }
      }
     
         int x_end=c.x;
         int y_end=c.y;
         newdata[g][j]=200;
     for (unsigned int x = 0; x < height; x++) {
         for (unsigned int y = 0; y < width; y++) {
            proof.push_back(round(newdata[x][y]));
         }
     }
     
     nav_msgs::OccupancyGrid grid;
     grid.header.stamp=ros::Time::now();
     grid.header.frame_id=header.frame_id;
     grid.info=info;
     grid.data = proof;
     map_pub.publish(grid);
     ROS_INFO("nav_fun eseguita"); 
     return percorso;
}


void Planner::plan(double x, double y) {
     set_goal(x,y);
     compute_rep_potential();
     compute_attract_potential();
     compute_total_potential();
     min_path();
     compute_gradient();
     integrate_gradient();
}

void Planner::set_goal (double x_pose, double y_pose) {
     goal_mat[0] = round((x_pose/resolution));
     goal_mat[1] = round((y_pose/resolution));
}

void Planner::set_init (double x_pose, double y_pose) {
     init_mat[0] = round((x_pose/resolution));
     init_mat[1] = round((y_pose/resolution));
}


    
	
