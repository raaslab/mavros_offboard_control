/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include <iostream>
#include <vector>
#include <string>
#include <cstring>

using namespace std;

#define PI 3.1415926535897

// Global variables
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped target_pose;
vector<geometry_msgs::PoseStamped> waypoint_pose;
bool target_pose_activate = false;
int waypoint_count = 0;
int sensing_range = 5;
int spiral_resol = 20;
int uav_height = 2;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void cur_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}

void target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    target_pose = *msg;
    target_pose_activate = true;
}

void msg_cb(const std_msgs::String::ConstPtr& msg) {
    if (strcmp(msg->data.c_str(), "go") == 0 && waypoint_pose.empty() && target_pose_activate) {
        waypoint_pose.push_back(target_pose);
        double deg_theta = 0.0;

        while (1) {
            deg_theta += spiral_resol;
            double rad_theta = deg_theta * PI / 180;
            double range = sensing_range * rad_theta / PI;

            double x = range * cos(rad_theta) + target_pose.pose.position.x;
            double y = range * sin(rad_theta) + target_pose.pose.position.y;

            double temp_dist = sqrt((target_pose.pose.position.x - x) * (target_pose.pose.position.x - x) 
                + (target_pose.pose.position.y - y) * (target_pose.pose.position.y - y));

            if (temp_dist > 50) {
                break;
            }

            geometry_msgs::PoseStamped temp_target_pose;
            temp_target_pose.pose.position.x = x;
            temp_target_pose.pose.position.y = y;
            temp_target_pose.pose.position.z = uav_height;

            waypoint_pose.push_back(temp_target_pose);
        }
    }
    else if (strcmp(msg->data.c_str(), "stop") == 0 && !waypoint_pose.empty()) {
        waypoint_pose.clear();
        waypoint_count = 0;
        target_pose_activate = false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber target_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("gm_phd/target_pose", 10, target_cb);
    ros::Subscriber msg_sub = nh.subscribe<std_msgs::String>
            ("gm_phd/message", 10, msg_cb);
    ros::Subscriber cur_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, cur_pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = uav_height;
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(waypoint_pose[waypoint_count]);
        if (abs(current_pose.pose.position.x - waypoint_pose[waypoint_count].pose.position.x) < 0.2 && 
            abs(current_pose.pose.position.y - waypoint_pose[waypoint_count].pose.position.y) < 0.2 &&
            abs(current_pose.pose.position.z - waypoint_pose[waypoint_count].pose.position.z) < 0.2) {
            waypoint_count += 1;

            // ROS_INFO("waypoint_count = %d, cur_pos = (%.2f, %.2f, %.2f), next_pos = (%.2f, %.2f, %.2f)", waypoint_count, 
            //     current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, 
            //     waypoint_pose[waypoint_count].pose.position.x, waypoint_pose[waypoint_count].pose.position.y, waypoint_pose[waypoint_count].pose.position.z);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}