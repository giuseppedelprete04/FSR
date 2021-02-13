#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

using namespace std;
	
namespace gazebo
{
  class WheelsVelPlugin : public ModelPlugin
  {

	private: ros::NodeHandle* _node_handle;
	private: physics::ModelPtr model;
	private: ros::Publisher _w_v_pub;
	private: event::ConnectionPtr updateConnection;
	private: physics::JointPtr _front_left_wheel_joint;
	private: physics::JointPtr _front_right_wheel_joint;
	private: std_msgs::Float32MultiArray _w_vel;
	public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {	
                ROS_WARN("Plugin Avviato");
		_node_handle = new ros::NodeHandle();	
		model = _parent;

		_front_left_wheel_joint = this->model->GetJoint("base_left_wheel_joint");
		_front_right_wheel_joint = this->model->GetJoint("base_right_wheel_joint");
		_w_v_pub = _node_handle->advertise< std_msgs::Float32MultiArray >("/diff_wheels/vel", 0);
		_w_vel.data.resize(2);
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&WheelsVelPlugin::OnUpdate, this));
    
	}

    // Called by the world update start event
    public: void OnUpdate()  {
			_w_vel.data[0] = _front_left_wheel_joint->GetVelocity(0);
			_w_vel.data[1] = _front_right_wheel_joint->GetVelocity(0);
			_w_v_pub.publish( _w_vel );
		}
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(WheelsVelPlugin)
}



