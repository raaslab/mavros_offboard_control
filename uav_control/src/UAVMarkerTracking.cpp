/**
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \08/07/2017
 * Copyright 2017. All Rights Reserved.
 */

#include "uav_control/UAVMarkerTracking.hpp"

UAVMarkerTracking::UAVMarkerTracking() :
m_uav_id(0), m_total_num_robots(0), m_verbal_flag(false), m_motion_primitive_check(false), m_init_local_pose_check(true), m_priv_nh("~")
{
    m_priv_nh.getParam("uav_id", m_uav_id);
    m_priv_nh.getParam("total_num_robots", m_total_num_robots);
    m_priv_nh.getParam("verbal_flag", m_verbal_flag);

    // Subscriber
    m_state_sub = m_nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &UAVMarkerTracking::state_cb, this);
    m_local_pos_sub = m_nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &UAVMarkerTracking::cur_pose_cb, this);
    m_init_local_pos_sub = m_nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &UAVMarkerTracking::init_pose_cb, this);
    m_marker_detection_sub = m_nh.subscribe<uav_control::marker_detection>
            ("uav_control/marker_detection", 10, &UAVMarkerTracking::marker_tracking_cb, this);
    // m_april_tag_sub = m_nh.subscribe<apriltags::AprilTagDetections>
    //         ("   ", 10, &UAVMarkerTracking::marker_tracking_cb, this);

    // Publisher
    m_local_pos_pub = m_nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    m_choice_pub = m_nh.advertise<uav_control::choice>
            ("uav_control/choice", 10);

    // m_arming_client = m_nh.serviceClient<mavros_msgs::CommandBool>
    //         ("mavros/cmd/arming");
    // m_set_mode_client = m_nh.serviceClient<mavros_msgs::SetMode>
    //         ("mavros/set_mode");

    // //the setpoint publishing rate MUST be faster than 2Hz
    // ros::Rate rate(20.0);

    // // wait for FCU connection
    // while(ros::ok() && m_current_state.connected){
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = 5;

    // //send a few setpoints before starting
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     m_local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";

    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;

    // ros::Time last_request = ros::Time::now();

    // while(1){
    //     if( m_current_state.mode != "OFFBOARD" &&
    //         (ros::Time::now() - last_request > ros::Duration(5.0))){
    //         if( m_set_mode_client.call(offb_set_mode) &&
    //             offb_set_mode.response.success){
    //             ROS_INFO("Offboard enabled");
    //         }
    //         last_request = ros::Time::now();
    //     } else {
    //         if( !m_current_state.armed &&
    //             (ros::Time::now() - last_request > ros::Duration(5.0))){
    //             if( m_arming_client.call(arm_cmd) &&
    //                 arm_cmd.response.success){
    //                 ROS_INFO("Vehicle armed");
    //             }
    //             last_request = ros::Time::now();
    //         }
    //     }

    //     m_local_pos_pub.publish(pose);

    //     if (abs(m_current_pose.pose.position.x - pose.pose.position.x) < 0.5 && 
    //         abs(m_current_pose.pose.position.y - pose.pose.position.y) < 0.5 &&
    //         abs(m_current_pose.pose.position.z - pose.pose.position.z) < 0.5) {
    //         ROS_INFO("Take-off is complete.");
    //     }
    // }

    get_motion_primitive();
}

void UAVMarkerTracking::state_cb(const mavros_msgs::State::ConstPtr& msg){
    m_current_state = *msg;
}

void UAVMarkerTracking::cur_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    m_current_pose = *msg;
}

void UAVMarkerTracking::init_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (m_init_local_pose_check) {
        m_waypoint_pose = *msg;

        m_init_local_pose_check = false;
    }

    publish_motion_primitive();

    ros::Rate rate(20.0);
    rate.sleep();
}

void UAVMarkerTracking::marker_tracking_cb(const uav_control::marker_detection::ConstPtr& msg) {
    while(1) {
        int latest_id = 0;
        vector<int> targets_covered;

        if (m_uav_id != 1) {
            boost::shared_ptr<uav_control::choice const> temp_choice_ptr;

            do {
                temp_choice_ptr = ros::topic::waitForMessage<uav_control::choice> ("uav_control/choice");
                if (temp_choice_ptr == NULL) {
                    ROS_ERROR("No choice messages received.");
                }
                else {
                    latest_id = temp_choice_ptr->id;
                }
            } while (latest_id != m_uav_id-1);

            for (int i = 0; i < temp_choice_ptr->num_covered; i++) {
                targets_covered.push_back(temp_choice_ptr->targets_covered[i]);
            }
        }

        set<int> targets_covered_set(targets_covered.begin(), targets_covered.end());
        targets_covered.clear();
        for (set<int>::iterator it = targets_covered_set.begin(); it != targets_covered_set.end(); ++it) {
            targets_covered.push_back(*it);
        }

        int best_motion_primitive_id = 0;
        double find_min_distance = 10000;
        vector<int> new_targets_covered;
        for (int i = 0; i < m_num_motion_primitive; i++) {
            double temp_accum_distance = 0.0;
            vector<int> temp_new_targets_covered;

            for (int j = 0; j < msg->total_num; j++) {
                bool check_covered_target_id = false;
                for (vector<int>::iterator it = targets_covered.begin(); it != targets_covered.end(); ++it) {
                    if (*it == msg->id[j]) {
                        check_covered_target_id = true;
                    }
                }
                if (!check_covered_target_id) {
                    temp_accum_distance += sqrt((m_x_pos[i] - msg->x_pos[j]) * (m_x_pos[i] - msg->x_pos[j]) + 
                        (m_y_pos[i] - msg->y_pos[j]) * (m_y_pos[i] - msg->y_pos[j]));
                    temp_new_targets_covered.push_back(msg->id[j]);
                }
            }

            if (find_min_distance > temp_accum_distance) {
                best_motion_primitive_id = i;
                find_min_distance = temp_accum_distance;
                new_targets_covered.clear();
                for (vector<int>::iterator it = temp_new_targets_covered.begin(); it != temp_new_targets_covered.end(); ++it) {
                    new_targets_covered.push_back(*it);
                }
            }
        }

        for (vector<int>::iterator it = new_targets_covered.begin(); it != new_targets_covered.end(); ++it) {
            targets_covered.push_back(*it);
        }

        // Publish /choice to all other robots.
        uav_control::choice pub_msg;
        pub_msg.id = m_uav_id;
        pub_msg.num_covered = targets_covered.size();
        int ii = 0;
        for (vector<int>::iterator it = targets_covered.begin(); it != targets_covered.end(); ++it) {
            pub_msg.targets_covered[ii] = *it;
            ii += 1;
        }
        m_choice_pub.publish(pub_msg);

        if (m_uav_id != m_total_num_robots) {
            boost::shared_ptr<uav_control::choice const> temp_choice_ptr;

            do {
                temp_choice_ptr = ros::topic::waitForMessage<uav_control::choice> ("uav_control/choice");
                if (temp_choice_ptr == NULL) {
                    ROS_ERROR("No choice messages received.");
                }
                else {
                    latest_id = temp_choice_ptr->id;
                }
            } while (latest_id != m_total_num_robots);
        }

        // Send the selected motion primitive
        apply_motion_primitive(best_motion_primitive_id);
    }
}

void UAVMarkerTracking::apply_motion_primitive(int selected_motion_primitive_id) {
    m_waypoint_pose.pose.position.x = m_current_pose.pose.position.x + m_x_pos[selected_motion_primitive_id];
    m_waypoint_pose.pose.position.y = m_current_pose.pose.position.y + m_y_pos[selected_motion_primitive_id];
    m_waypoint_pose.pose.position.z = m_current_pose.pose.position.z + m_z_pos[selected_motion_primitive_id];

    m_motion_primitive_check = true;
}

void UAVMarkerTracking::get_motion_primitive() {
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

void UAVMarkerTracking::publish_motion_primitive() {
    if (m_motion_primitive_check) { // Motion primitive is applied.
        while(1) {
            m_local_pos_pub.publish(m_waypoint_pose);
            double dist = sqrt(
                (m_current_pose.pose.position.x-m_waypoint_pose.pose.position.x)* 
                (m_current_pose.pose.position.x-m_waypoint_pose.pose.position.x) + 
                (m_current_pose.pose.position.y-m_waypoint_pose.pose.position.y)* 
                (m_current_pose.pose.position.y-m_waypoint_pose.pose.position.y) + 
                (m_current_pose.pose.position.z-m_waypoint_pose.pose.position.z)* 
                (m_current_pose.pose.position.z-m_waypoint_pose.pose.position.z)); 
            ROS_INFO("distance: %.2f", dist);
            
            if (abs(m_current_pose.pose.position.x - m_waypoint_pose.pose.position.x) < 0.2 && 
                abs(m_current_pose.pose.position.y - m_waypoint_pose.pose.position.y) < 0.2 &&
                abs(m_current_pose.pose.position.z - m_waypoint_pose.pose.position.z) < 0.2) {
                ROS_INFO("Now you can apply the next motion primitive.");

                if(m_verbal_flag) {
                    ROS_INFO("target_pos = (%.2f, %.2f, %.2f)", m_waypoint_pose.pose.position.x,
                        m_waypoint_pose.pose.position.y, m_waypoint_pose.pose.position.z);
                }

                break;
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
    UAVMarkerTracking mp;

    ros::spin();
    return 0;
}