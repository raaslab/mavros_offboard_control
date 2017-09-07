/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include "uav_control/MarkerPooling.hpp"

MarkerPooling::MarkerPooling() :
m_total_num_robots(0), m_total_num_markers(0), m_priv_nh("~")
{
    m_priv_nh.getParam("total_num_robots", m_total_num_robots);
    m_priv_nh.getParam("total_num_markers", m_total_num_markers);

    // Subscriber
    m_pool_sub = m_nh.subscribe<apriltags::AprilTagDetections>
            ("apriltags/aprilTagDetections", 10, &MarkerPooling::marker_pooling, this);

    // Publisher
    m_marker_pub = m_nh.advertise<uav_control::marker_detection>
            ("uav_control/marker_detection", 10);

    m_count_id = 0;
}

void MarkerPooling::marker_pooling(const apriltags::AprilTagDetections::ConstPtr& msg) {
    for (vector<AprilTagDetection>::iterator it = msg->detections.begin(); it != msg->detections.end(); ++it) {
        if (m_count_id == it->id) {
            m_marker_id.push_back(it->id);
            m_x_pos.push_back(it->pose.position.x);
            m_y_pos.push_back(it->pose.position.y);
            m_z_pos.push_back(it->pose.position.z);
            m_count_id += 1;
        }
    }

    if (m_count_id == m_total_num_markers+1) {
        uav_control::marker_detection pub_msg;
        pub_msg.total_num = m_total_num_markers;
        pub_msg.id = m_marker_id;
        pub_msg.x_pos = m_x_pos;
        pub_msg.y_pos = m_y_pos;
        pub_msg.z_pos = m_z_pos;

        m_marker_pub.publish(pub_msg);

        m_count_id = 0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    MarkerPooling mp;

    ros::spin();

    return 0;
}