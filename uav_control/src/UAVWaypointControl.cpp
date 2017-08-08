/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include "uav_control/UAVWaypointControl.hpp"

UAVWaypointControl::UAVWaypointControl() :
m_verbal_flag(false), m_init_local_pose_check(true), m_waypoint_count(0), m_priv_nh("~")
{
    m_priv_nh.getParam("verbal_flag", m_verbal_flag);

    // Subscriber
    m_state_sub = m_nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &UAVWaypointControl::state_cb, this);
    m_local_pos_sub = m_nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &UAVWaypointControl::cur_pose_cb, this);
    m_init_local_pos_sub = m_nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &UAVWaypointControl::init_pose_cb, this);

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

    get_waypoint();
}


void UAVWaypointControl::state_cb(const mavros_msgs::State::ConstPtr& msg){
    m_current_state = *msg;
}

void UAVWaypointControl::cur_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    m_current_pose = *msg;
}

void UAVWaypointControl::init_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (m_init_local_pose_check) {
        for(int i = 0; i < m_num_waypoint; i++){
            geometry_msgs::PoseStamped temp_target_pose;

            temp_target_pose.pose.position.x = msg->pose.position.x + m_x_pos[i];
            temp_target_pose.pose.position.y = msg->pose.position.y + m_y_pos[i];
            temp_target_pose.pose.position.z = msg->pose.position.z + m_z_pos[i];

            m_waypoint_pose.push_back(temp_target_pose);
        }

        m_init_local_pose_check = false;
    }

    publish_waypoint();

    ros::Rate rate(20.0);
    rate.sleep();
}

void UAVWaypointControl::get_waypoint() {
    if (ros::param::get("offb_node/num_waypoint", m_num_waypoint)) {}
    else {
        ROS_WARN("Didn't find num_waypoint");
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

    //Safety check in case wrong waypoints are fed.
    if (m_x_pos.size() != m_num_waypoint) {
        ROS_WARN("Wrong x_pos values.");
    }
    if (m_y_pos.size() != m_num_waypoint) {
        ROS_WARN("Wrong y_pos values.");
    }
    if (m_z_pos.size() != m_num_waypoint) {
        ROS_WARN("Wrong z_pos values.");
    }
}

void UAVWaypointControl::publish_waypoint() {
    if (!m_init_local_pose_check) {
        m_local_pos_pub.publish(m_waypoint_pose[m_waypoint_count]);
        if (m_verbal_flag) {
            double dist = sqrt(
                (m_current_pose.pose.position.x-m_waypoint_pose[m_waypoint_count].pose.position.x)* 
                (m_current_pose.pose.position.x-m_waypoint_pose[m_waypoint_count].pose.position.x) + 
                (m_current_pose.pose.position.y-m_waypoint_pose[m_waypoint_count].pose.position.y)* 
                (m_current_pose.pose.position.y-m_waypoint_pose[m_waypoint_count].pose.position.y) + 
                (m_current_pose.pose.position.z-m_waypoint_pose[m_waypoint_count].pose.position.z)* 
                (m_current_pose.pose.position.z-m_waypoint_pose[m_waypoint_count].pose.position.z)); 
            ROS_INFO("distance: %.2f", dist);
        }
        
        if (abs(m_current_pose.pose.position.x - m_waypoint_pose[m_waypoint_count].pose.position.x) < 0.5 && 
            abs(m_current_pose.pose.position.y - m_waypoint_pose[m_waypoint_count].pose.position.y) < 0.5 &&
            abs(m_current_pose.pose.position.z - m_waypoint_pose[m_waypoint_count].pose.position.z) < 0.5) {

            m_waypoint_count += 1;

            if (m_waypoint_count >= m_num_waypoint) {
                m_waypoint_count = m_waypoint_count - 1;
            }

            // ROS_INFO("m_waypoint_count = %d, cur_pos = (%.2f, %.2f, %.2f), next_pos = (%.2f, %.2f, %.2f)", m_waypoint_count, 
            //     m_current_pose.pose.position.x, m_current_pose.pose.position.y, m_current_pose.pose.position.z, 
            //     m_waypoint_pose[m_waypoint_count].pose.position.x, m_waypoint_pose[m_waypoint_count].pose.position.y, m_waypoint_pose[m_waypoint_count].pose.position.z);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    UAVWaypointControl mp;

    ros::spin();

    return 0;
}