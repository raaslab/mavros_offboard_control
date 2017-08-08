#ifndef UAVWAYPOINTCONTROL_HPP_
#define UAVWAYPOINTCONTROL_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>

using namespace std;

class UAVWaypointControl {  
private:
	ros::NodeHandle m_nh;
	ros::NodeHandle m_priv_nh;

	// Subscriber
	ros::Subscriber m_state_sub;
	ros::Subscriber m_local_pos_sub;
	ros::Subscriber m_init_local_pos_sub;

	//Publisher
	ros::Publisher m_local_pos_pub;

	// ros::ServiceClient m_arming_client;
	// ros::ServiceClient m_set_mode_client;

	bool m_verbal_flag;

	mavros_msgs::State m_current_state;
	geometry_msgs::PoseStamped m_current_pose;
	vector<geometry_msgs::PoseStamped> m_waypoint_pose;
	bool m_init_local_pose_check;
	int m_waypoint_count;

	// Waypoint points
	int m_num_waypoint;
	vector<double> m_x_pos;
	vector<double> m_y_pos;
	vector<double> m_z_pos;

public:
	UAVWaypointControl();

	void state_cb(const mavros_msgs::State::ConstPtr& msg);
	void cur_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void init_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void get_waypoint();
	void publish_waypoint();
};

#endif