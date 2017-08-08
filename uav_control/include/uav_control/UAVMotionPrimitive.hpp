#ifndef UAVMOTIONPRIMITIVE_HPP_
#define UAVMOTIONPRIMITIVE_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>
#include <string>

using namespace std;

class UAVMotionPrimitive {  
private:
	ros::NodeHandle m_nh;
	ros::NodeHandle m_priv_nh;

	// Subscriber
	ros::Subscriber m_state_sub;
	ros::Subscriber m_local_pos_sub;
	ros::Subscriber m_init_local_pos_sub;
	ros::Subscriber m_motion_primitive_sub;

	//Publisher
	ros::Publisher m_local_pos_pub;

	// ros::ServiceClient m_arming_client;
	// ros::ServiceClient m_set_mode_client;

	bool m_verbal_flag;

	mavros_msgs::State m_current_state;
	geometry_msgs::PoseStamped m_current_pose;
	geometry_msgs::PoseStamped m_waypoint_pose;
	bool m_motion_primitive_check;
	bool m_init_local_pose_check;

	// Motion primitives
	int m_num_motion_primitive;
	vector<string> m_names;
	vector<double> m_x_pos;
	vector<double> m_y_pos;
	vector<double> m_z_pos;

public:
	UAVMotionPrimitive();

	void state_cb(const mavros_msgs::State::ConstPtr& msg);
	void cur_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void init_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void motion_primitive_cb(const std_msgs::String::ConstPtr& msg);
	void get_motion_primitive();
	void publish_motion_primitive();
};

#endif