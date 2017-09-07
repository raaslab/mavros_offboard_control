/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include "uav_control/MarkerPooling.hpp"

MarkerPooling::MarkerPooling() :
m_verbal_flag(false), m_init_local_pose_check(true), m_waypoint_count(0), m_priv_nh("~")
{
    m_priv_nh.getParam("verbal_flag", m_verbal_flag);

    // Subscriber
    m_state_sub = m_nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &MarkerPooling::state_cb, this);
    m_local_pos_sub = m_nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &MarkerPooling::cur_pose_cb, this);
    m_init_local_pos_sub = m_nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &MarkerPooling::init_pose_cb, this);

    // Publisher
    m_local_pos_pub = m_nh.advertise<geometry_msgs::PoseSta mped>
            ("mavros/setpoint_position/local", 10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    MarkerPooling mp;

    ros::spin();

    return 0;
}