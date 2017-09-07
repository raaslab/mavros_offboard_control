#ifndef MARKERPOOLING_HPP_
#define MARKERPOOLING_HPP_

#include <ros/ros.h>
#include <apriltags/AprilTagDetections.h>
#include <apriltags/AprilTagDetection.h>
#include <uav_control/marker_detection.h>
#include <vector>
#include <string>
#include <set>

using namespace std;

class MarkerPooling {  
private:
	ros::NodeHandle m_nh;
	ros::NodeHandle m_priv_nh;

	// Subscriber
	ros::Subscriber m_pool_sub;

	//Publisher
	ros::Publisher m_marker_pub;

	int m_total_num_robots;
	int m_total_num_markers;
	int m_count_id;

	vector<int> m_marker_id;
	vector<double> m_x_pos;
	vector<double> m_y_pos;
	vector<double> m_z_pos;

public:
	MarkerPooling();

	void marker_pooling(const apriltags::AprilTagDetections::ConstPtr& msg);
};

#endif