#include "ros/ros.h"
#include "boost/thread.hpp"
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include <geometry_msgs/WrenchStamped.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;
using namespace Eigen;

bool T_RPY(Eigen::Matrix3d &R, double phi, double theta, double psi);
bool Tdot_RPY(Eigen::Matrix3d &R, double phi, double theta, double psi);
Matrix3d Skew(Vector3d v);
Vector3d Vee(Matrix3d S);
void twist2Vector(const geometry_msgs::TwistStamped twist, VectorXd& vel);
void accel2Vector(const geometry_msgs::AccelStamped acc, VectorXd& a);
void wrench2Vector(const geometry_msgs::WrenchStamped wrench, VectorXd& w);

class SPLINE_PLANNER {
	public:
		SPLINE_PLANNER(double freq);
    void compute_traj();
    void set_waypoints(std::vector<double> points, std::vector<double> times,double xdi=0,double xdf=0, double xddi=0, double xddf=0);
		bool isReady() {return _ready;};
		bool getNext(double &x, double &xd, double &xdd);

    std::vector<double> _x;
    std::vector<double> _xd;
    std::vector<double> _xdd;
    std::vector<double> _t;

	private:
    std::vector<double> _points;
    std::vector<double> _times;
    double _freq;
    int _N;
		bool _ready;
		int _counter;
		double _xdi,_xdf,_xddi,_xddf;

};


