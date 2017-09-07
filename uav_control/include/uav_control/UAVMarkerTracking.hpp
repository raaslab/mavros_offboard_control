#ifndef UAVMARKERTRACKING_HPP_
#define UAVMARKERTRACKING_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <uav_control/choice.h>
#include <uav_control/marker_detection.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>
#include <set>

using namespace std;

class UAVMarkerTracking {  
private:
	ros::NodeHandle m_nh;
	ros::NodeHandle m_priv_nh;

	// Subscriber
	ros::Subscriber m_state_sub;
	ros::Subscriber m_local_pos_sub;
	ros::Subscriber m_init_local_pos_sub;
	ros::Subscriber m_choice_sub;
	ros::Subscriber m_marker_detection_sub;

	//Publisher
	ros::Publisher m_local_pos_pub;
	ros::Publisher m_choice_pub;

	// ros::ServiceClient m_arming_client;
	// ros::ServiceClient m_set_mode_client;

	int m_uav_id;
	int m_total_num_robots;
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
	UAVMarkerTracking();

	void state_cb(const mavros_msgs::State::ConstPtr& msg);
	void cur_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void init_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void marker_tracking_cb(const uav_control::marker_detection::ConstPtr& msg);
	void apply_motion_primitive(int selected_motion_primitive_id);
	void get_motion_primitive();
	void publish_motion_primitive();
};

#endif