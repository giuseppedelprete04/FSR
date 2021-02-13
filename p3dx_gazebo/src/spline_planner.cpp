#include "../include/spline_planner.h"

//UTILS

bool T_RPY(Eigen::Matrix3d &R, double phi, double theta, double psi) {
  R << 0, -sin(phi), cos(phi)*cos(theta),
       0, cos(phi), sin(phi)*cos(theta),
       1, 0, -sin(theta);

  if(abs(R.determinant())>0.001) return true;
  else return false;
}

bool Tdot_RPY(Eigen::Matrix3d &R, double phi, double theta, double psi) {
  R << 0, -cos(phi), -sin(phi)*cos(theta)-cos(phi)*sin(theta) ,
       0, -sin(phi), cos(phi)*cos(theta)-sin(phi)*sin(theta),
       0, 0, -cos(theta);

  if(abs(R.determinant())>0.001) return true;
  else return false;
}

Matrix3d Skew(Vector3d v) {
  Matrix3d S;
  S << 0, -v(2), v(1),
       v(2), 0, -v(0),
       -v(1), v(0), 0;

  return S;
}

Vector3d Vee(Matrix3d S) {
  Vector3d v;
  v<<S(2,1),S(0,2),S(1,0);
  //cout<<S<<endl;
  return v;
}

void twist2Vector(const geometry_msgs::TwistStamped twist, VectorXd& vel) {
  vel.resize(6);
  vel(0) = twist.twist.linear.x;
	vel(1) = twist.twist.linear.y;
	vel(2) = twist.twist.linear.z;
	vel(3) = twist.twist.angular.x;
	vel(4) = twist.twist.angular.y;
	vel(5) = twist.twist.angular.z;
}

void accel2Vector(const geometry_msgs::AccelStamped acc, VectorXd& a) {
  a.resize(6);
  a(0) = acc.accel.linear.x;
	a(1) = acc.accel.linear.y;
	a(2) = acc.accel.linear.z;
	a(3) = acc.accel.angular.x;
	a(4) = acc.accel.angular.y;
	a(5) = acc.accel.angular.z;
}

void wrench2Vector(const geometry_msgs::WrenchStamped wrench, VectorXd& w) {
  w.resize(6);
  w(0) = wrench.wrench.force.x;
	w(1) = wrench.wrench.force.y;
	w(2) = wrench.wrench.force.z;
	w(3) = wrench.wrench.torque.x;
	w(4) = wrench.wrench.torque.y;
	w(5) = wrench.wrench.torque.z;
}

//END UTILS

SPLINE_PLANNER::SPLINE_PLANNER(double freq) {
  _freq = freq;
  _ready=false;
  _counter=0;
}

void SPLINE_PLANNER::set_waypoints(std::vector<double> points, std::vector<double> times, double xdi, double xdf, double xddi, double xddf) {
  _ready=false;
  _counter=0;
  _points.clear();_points.resize(0);
  _times.clear();_times.resize(0);
  _N = 0;
  _t.clear();_t.resize(0);
  _x.clear();_x.resize(0);
  _xd.clear();_xd.resize(0);
  _xdd.clear();_xdd.resize(0);

  _points = points;
  _times = times;
  _N = points.size();

  _xdi=xdi; _xdf=xdf;
  _xddi=xddi; _xddf=xddf;

}

void SPLINE_PLANNER::compute_traj() {

  if (_N == 2) {
    double dt = _times[1] - _times[0];
    _times.insert(_times.begin()+1,_times.front()+dt/3.0);
    _times.insert(_times.end()-1,_times.back()-dt/3.0);
  }
  else {
    _times.insert(_times.begin()+1,_times[0]+(_times[1]-_times[0])/2.0);
    _times.insert(_times.end()-1,_times[_times.size()-2]+(_times[_times.size()-1]-_times[_times.size()-2])/2.0);
  }

  _points.insert(_points.begin()+1,0.0); //dummy virtual point
  _points.insert(_points.end()-1,0.0); //dummy virtual point

  std::vector<double> dt;
  for (int i=0; i<=_times.size()-2; i++)
    dt.push_back(_times[i+1]-_times[i]);

  MatrixXd A = MatrixXd::Zero(_N,_N);
  VectorXd b(_N);

  //Diagonale
  A(0,0) = dt[0]/2.0 + dt[1]/3.0 + dt[0]*dt[0]/(6.0*dt[1]);
  A(_N-1,_N-1) = dt[_N-1]/3.0 + dt[_N]/2.0 + dt[_N]*dt[_N]/(6.0*dt[_N-1]);
  for (int i=1; i<(_N-1) ; i++)
    A(i,i) = (dt[i]+dt[i+1])/3.0;

  //Diagonale bassa
  A(1,0) = dt[1]/6.0 - dt[0]*dt[0]/(6.0*dt[1]);
  for (int i=2; i<=(_N-1) ; i++)
    A(i,i-1) = dt[i]/6.0;

  //Diagonale alta
  A(_N-2,_N-1) = dt[_N-1]/6.0 - dt[_N]*dt[_N]/(6.0*dt[_N-1]);
  for (int i=0; i<=(_N-3) ; i++)
    A(i,i+1) = dt[i+1]/6.0;

  if (_N>4) {
    b(0) = (_points[2]-_points[0])/dt[1] - ((1/dt[1])+(1/dt[0]))*(_xdi*dt[0] + _xddi*dt[0]*dt[0]/3.0) - _xddi*dt[0]/6.0;
    b(1) = (_points[0] + _xdi*dt[0] + _xddi*dt[0]*dt[0]/3.0)/dt[1] - ( (1/dt[2])+(1/dt[1]) )*_points[2] + _points[3]/dt[2];
    b(_N-2) = _points[_N-2]/dt[_N-2] - ( (1/dt[_N-1])+(1/dt[_N-2]) )*_points[_N-1] + (_points[_N+1] - _xdf*dt[_N] + _xddf*dt[_N]*dt[_N]/3.0)/dt[_N-1];
    b(_N-1) = (_points[_N-1] - _points[_N+1])/dt[_N-1] - ((1/dt[_N])+(1/dt[_N-1]))*( -_xdf*dt[_N] + _xddf*dt[_N]*dt[_N]/3.0) - _xddf*dt[_N]/6.0;
    for (int i=2; i<(_N-2) ; i++)
      b(i) = _points[i]/dt[i] - ( (1/dt[i+1])+(1/dt[i]) )*_points[i+1] + _points[i+2]/dt[i+1];
  }
  else if (_N==4) {
    b(0) = (_points[2]-_points[0])/dt[1] - ((1/dt[1])+(1/dt[0]))*(_xdi*dt[0] + _xddi*dt[0]*dt[0]/3.0) - _xddi*dt[0]/6.0;
    b(1) = (_points[0] + _xdi*dt[0] + _xddi*dt[0]*dt[0]/3.0)/dt[1] - ( (1/dt[2])+(1/dt[1]) )*_points[2] + _points[3]/dt[2];
    b(_N-2) = _points[_N-2]/dt[_N-2] - ( (1/dt[_N-1])+(1/dt[_N-2]) )*_points[_N-1] + (_points[_N+1] - _xdf*dt[_N] + _xddf*dt[_N]*dt[_N]/3.0)/dt[_N-1];
    b(_N-1) = (_points[_N-1] - _points[_N+1])/dt[_N-1] - ((1/dt[_N])+(1/dt[_N-1]))*( -_xdf*dt[_N] + _xddf*dt[_N]*dt[_N]/3.0) - _xddf*dt[_N]/6.0;
  }
  else if (_N==3) {
    b(0) = (_points[2]-_points[0])/dt[1] - ((1/dt[1])+(1/dt[0]))*(_xdi*dt[0] + _xddi*dt[0]*dt[0]/3.0) - _xddi*dt[0]/6.0;
    b(_N-1) = (_points[_N-1] - _points[_N+1])/dt[_N-1] - ((1/dt[_N])+(1/dt[_N-1]))*( -_xdf*dt[_N] + _xddf*dt[_N]*dt[_N]/3.0) - _xddf*dt[_N]/6.0;
    b(1) = (_points[0] + _xdi*dt[0] + _xddi*dt[0]*dt[0]/3.0)/dt[1] - ( (1/dt[2])+(1/dt[1]) )*_points[2] + (_points[_N+1] - _xdf*dt[3] + _xddf*dt[3]*dt[3]/3.0)/dt[2];
  }
  else {
    b(0) = (_points[_N+1]-_points[0])/dt[1] - ((1/dt[1])+(1/dt[0]))*(_xdi*dt[0] + _xddi*dt[0]*dt[0]/3.0) - _xddi*dt[0]/6.0;
    b(_N-1) = (_points[0] - _points[_N+1])/dt[_N-1] - ((1/dt[_N])+(1/dt[_N-1]))*( -_xdf*dt[_N] + _xddf*dt[_N]*dt[_N]/3.0) - _xddf*dt[_N]/6.0;
  }

  //cout<<A<<endl;
  //cout<<b<<endl;

  VectorXd temp(_N);
  temp= A.inverse() * b;
  VectorXd accel(_N+2);
  accel << 0,temp,0;
  //cout<<A<<endl<<b<<endl;
  //cout<<accel;
  _points[1] = _points[0] + _xdi*dt[0] + _xddi*dt[0]*dt[0]/3.0 + accel[1]*dt[0]*dt[0]/6.0;
  _points[_N] = _points[_N+1] - _xdf*dt[_N] + _xddf*dt[_N]*dt[_N]/3.0 + accel[_N]*dt[_N]*dt[_N]/6.0;

/*
  for(int i=0; i<=(_N+1); i++){
    cout<<"Punto "<<i<<" :"<<_points[i]<<endl;
    cout<<"Tempo "<<i<<" :"<<_times[i]<<endl;
    cout<<"Accel "<<i<<" :"<<accel[i]<<endl;
  }
*/

  for(double t=_times.front(), k=0; t<=_times.back(); t+=(1.0/_freq)) { //CHECK TEMPO INIZIALE t=0
    if (t>_times[k+1])
      k++;

    double acc = accel(k)/dt[k]*(_times[k+1]-t) + accel(k+1)/dt[k]*(t-_times[k]);
    double vel = -accel(k)/(2.0*dt[k])*(_times[k+1]-t)*(_times[k+1]-t) + accel(k+1)/(2.0*dt[k])*(t-_times[k])*(t-_times[k]) + (_points[k+1]-_points[k])/dt[k] + (dt[k]/6.0)*(accel(k)-accel(k+1));
    //double vel = ( (accel(k)/dt[k]*_times[k+1]) - (accel(k+1)/dt[k]*_times[k]) )*t + (t*t/2.0)*( (accel(k+1)/dt[k]) - (accel(k)/dt[k]) );
    //double vel = accel(k)/dt[k]*((_times[k+1]*t)-(t*t/2.0)) + accel(k+1)/dt[k]*((t*t/2.0)-(_times[k]*t));
    double pos = accel(k)/(6.0*dt[k])*(_times[k+1]-t)*(_times[k+1]-t)*(_times[k+1]-t) + accel(k+1)/(6.0*dt[k])*(t-_times[k])*(t-_times[k])*(t-_times[k]) + ( (_points[k+1]/dt[k]) - (dt[k]*accel(k+1)/6.0) )*(t-_times[k]) + ( (_points[k]/dt[k]) - (dt[k]*accel(k)/6.0) )*(_times[k+1]-t);
    //cout<<k<<"k - "<<t<<"/"<<_times.back()<<"s - "<<pos<<" - "<<vel<<" - "<<acc<<endl;
/*
    if(_points[k] == _points[k+1]) {
      //cout<<"Uguale prima:"<<pos<<"dopo :";
      pos = _points[k];
      vel = 0;
      acc = 0;
    //  cout<<pos<<endl;
  } */

    _t.push_back(t);
    _x.push_back(pos);
    _xd.push_back(vel);
    _xdd.push_back(acc);
  }
/*
  _t.push_back(_times.back());
  _x.push_back(_points.back());
  _xd.push_back(0.0);
  _xdd.push_back(0.0);
*/
  _ready=true;

}

bool SPLINE_PLANNER::getNext(double &x, double &xd, double &xdd) {
  x = _x[_counter];
  xd = _xd[_counter];
  xdd = _xdd[_counter];

  if(!_ready) return false;
  if(_counter>=(_x.size()-1)) {
    _ready = false;
    return false;
  }

  _counter++;
}


