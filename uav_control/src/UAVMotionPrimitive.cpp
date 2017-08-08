/**
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \08/07/2017
 * Copyright 2017. All Rights Reserved.
 */

#include "uav_control/UAVMotionPrimitive.hpp"

UAVMotionPrimitive::UAVMotionPrimitive() :
m_verbal_flag(false), m_motion_primitive_check(false), m_init_local_pose_check(true), m_priv_nh("~")
{
    m_priv_nh.getParam("verbal_flag", m_verbal_flag);

    // Subscriber
    m_state_sub = m_nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &UAVMotionPrimitive::state_cb, this);
    m_local_pos_sub = m_nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &UAVMotionPrimitive::cur_pose_cb, this);
    m_init_local_pos_sub = m_nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &UAVMotionPrimitive::init_pose_cb, this);
    m_motion_primitive_sub = m_nh.subscribe<std_msgs::String>
            ("mavros/motion_primitive", 10, &UAVMotionPrimitive::motion_primitive_cb, this);

    // Publisher
    m_local_pos_pub = m_nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // m_arming_client = m_nh.serviceClient<mavros_msgs::CommandBool>
    //         ("mavros/cmd/arming");
    // m_set_mode_client = m_nh.serviceClient<mavros_msgs::SetMode>
    //         ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    // ros::Rate rate(20.0);

    //send a few setpoints before starting
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     geometry_msgs::PoseStamped pose;
    //     pose.pose.position.x = 0;
    //     pose.pose.position.y = 0;
    //     pose.pose.position.z = 5;
    //     m_local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // wait for FCU connection
    // while(ros::ok() && m_current_state.connected){
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";

    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;

    // ros::Time last_request = ros::Time::now();

    // if( m_current_state.mode != "OFFBOARD" &&
    //     (ros::Time::now() - last_request > ros::Duration(5.0))){
    //     if( m_set_mode_client.call(offb_set_mode) &&
    //         offb_set_mode.response.success){
    //         ROS_INFO("Offboard enabled");
    //     }
    //     last_request = ros::Time::now();
    // } else {
    //     if( !m_current_state.armed &&
    //         (ros::Time::now() - last_request > ros::Duration(5.0))){
    //         if( m_arming_client.call(arm_cmd) &&
    //             arm_cmd.response.success){
    //             ROS_INFO("Vehicle armed");
    //         }
    //         last_request = ros::Time::now();
    //     }
    // }

    get_motion_primitive();
}

void UAVMotionPrimitive::state_cb(const mavros_msgs::State::ConstPtr& msg){
    m_current_state = *msg;
}

void UAVMotionPrimitive::cur_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    m_current_pose = *msg;
}

void UAVMotionPrimitive::init_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (m_init_local_pose_check) {
        m_waypoint_pose = *msg;

        m_init_local_pose_check = false;
    }

    publish_motion_primitive();

    ros::Rate rate(20.0);
    rate.sleep();
}

void UAVMotionPrimitive::motion_primitive_cb(const std_msgs::String::ConstPtr& msg) {
    for (int i = 0; i < m_num_motion_primitive; i++) {
        if(strcmp(msg->data.c_str(), m_names[i].c_str()) == 0) {
            m_waypoint_pose.pose.position.x = m_current_pose.pose.position.x + m_x_pos[i];
            m_waypoint_pose.pose.position.y = m_current_pose.pose.position.y + m_y_pos[i];
            m_waypoint_pose.pose.position.z = m_current_pose.pose.position.z + m_z_pos[i];
        }
    }

    m_motion_primitive_check = true;
}

void UAVMotionPrimitive::get_motion_primitive() {
    if (ros::param::get("offb_node/num_motion_primitive", m_num_motion_primitive)) {}
    else {
        ROS_WARN("Didn't find num_motion_primitive");
    }
    if (ros::param::get("offb_node/names", m_names)) {}
    else {
        ROS_WARN("Didn't find names");
    }
    if (ros::param::get("offb_node/x_pos", m_x_pos)) {}
    else {
        ROS_WARN("Didn't find x_pos");
    }
    if (ros::param::get("offb_node/y_pos", m_y_pos)) {}
    else {
        ROS_WARN("Didn't find y_pos");
    }
    if (ros::param::get("offb_node/z_pos", m_z_pos)) {}
    else {
        ROS_WARN("Didn't find z_pos");
    }
}

void UAVMotionPrimitive::publish_motion_primitive() {
    if (m_motion_primitive_check) { // Motion primitive is applied.
        m_local_pos_pub.publish(m_waypoint_pose);
        double dist = sqrt(
            (m_current_pose.pose.position.x-m_waypoint_pose.pose.position.x)* 
            (m_current_pose.pose.position.x-m_waypoint_pose.pose.position.x) + 
            (m_current_pose.pose.position.y-m_waypoint_pose.pose.position.y)* 
            (m_current_pose.pose.position.y-m_waypoint_pose.pose.position.y) + 
            (m_current_pose.pose.position.z-m_waypoint_pose.pose.position.z)* 
            (m_current_pose.pose.position.z-m_waypoint_pose.pose.position.z)); 
        ROS_INFO("distance: %.2f", dist);
        
        if (abs(m_current_pose.pose.position.x - m_waypoint_pose.pose.position.x) < 0.5 && 
            abs(m_current_pose.pose.position.y - m_waypoint_pose.pose.position.y) < 0.5 &&
            abs(m_current_pose.pose.position.z - m_waypoint_pose.pose.position.z) < 0.5) {
            ROS_INFO("Now you can apply the next motion primitive.");

            if(m_verbal_flag) {
                ROS_INFO("target_pos = (%.2f, %.2f, %.2f)", m_waypoint_pose.pose.position.x,
                    m_waypoint_pose.pose.position.y, m_waypoint_pose.pose.position.z);
            }
        }
    }
    else { // Motion primitive is not obtained yet.
        if (!m_init_local_pose_check) {
            m_local_pos_pub.publish(m_waypoint_pose);

            if(m_verbal_flag) {
                ROS_INFO("target_pos = (%.2f, %.2f, %.2f)", m_waypoint_pose.pose.position.x,
                    m_waypoint_pose.pose.position.y, m_waypoint_pose.pose.position.z);
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    UAVMotionPrimitive mp;

    ros::spin();
    return 0;
}