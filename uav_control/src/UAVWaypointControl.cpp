/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>

int waypoint_count = 0;
bool init_local_pose_check = true;
std::vector<geometry_msgs::PoseStamped> waypoint_pose;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped init_pose;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void cur_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}

void init_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (init_local_pose_check) {
        init_pose = *msg;

        for(int i = 0; i < 5; i++){
            geometry_msgs::PoseStamped temp_target_pose;
            if(i == 0){
                temp_target_pose.pose.position.x = init_pose.pose.position.x + 0;
                temp_target_pose.pose.position.y = init_pose.pose.position.y + 0;
                temp_target_pose.pose.position.z = init_pose.pose.position.z + 5;
            }
            else if(i == 1){
                temp_target_pose.pose.position.x = init_pose.pose.position.x + 10;
                temp_target_pose.pose.position.y = init_pose.pose.position.y + 0;
                temp_target_pose.pose.position.z = init_pose.pose.position.z + 5;
            }
            else if(i == 2){
                temp_target_pose.pose.position.x = init_pose.pose.position.x + 10;
                temp_target_pose.pose.position.y = init_pose.pose.position.y + 10;
                temp_target_pose.pose.position.z = init_pose.pose.position.z + 5;
            }
            else if(i == 3){
                temp_target_pose.pose.position.x = init_pose.pose.position.x + 0;
                temp_target_pose.pose.position.y = init_pose.pose.position.y + 10;
                temp_target_pose.pose.position.z = init_pose.pose.position.z + 5;
            }
            else if(i == 4){
                temp_target_pose.pose.position.x = init_pose.pose.position.x + 0;
                temp_target_pose.pose.position.y = init_pose.pose.position.y + 0;
                temp_target_pose.pose.position.z = init_pose.pose.position.z + 5;
            }
            else if(i == 5){
                temp_target_pose.pose.position.x = init_pose.pose.position.x + 0;
                temp_target_pose.pose.position.y = init_pose.pose.position.y + 0;
                temp_target_pose.pose.position.z = init_pose.pose.position.z + 8;
            }
            else if(i == 6){
                temp_target_pose.pose.position.x = init_pose.pose.position.x + 10;
                temp_target_pose.pose.position.y = init_pose.pose.position.y + 0;
                temp_target_pose.pose.position.z = init_pose.pose.position.z + 8;
            }
            else if(i == 7){
                temp_target_pose.pose.position.x = init_pose.pose.position.x + 10;
                temp_target_pose.pose.position.y = init_pose.pose.position.y + 10;
                temp_target_pose.pose.position.z = init_pose.pose.position.z + 8;
            }
            else if(i == 8){
                temp_target_pose.pose.position.x = init_pose.pose.position.x + 0;
                temp_target_pose.pose.position.y = init_pose.pose.position.y + 10;
                temp_target_pose.pose.position.z = init_pose.pose.position.z + 8;
            }
            else if(i == 9){
                temp_target_pose.pose.position.x = init_pose.pose.position.x + 0;
                temp_target_pose.pose.position.y = init_pose.pose.position.y + 0;
                temp_target_pose.pose.position.z = init_pose.pose.position.z + 8;
            }

            waypoint_pose.push_back(temp_target_pose);
        }

        init_local_pose_check = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, cur_pose_cb);
    ros::Subscriber init_local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, init_pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    // ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    //         ("mavros/cmd/arming");
    // ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    //         ("mavros/set_mode");

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
        pose.pose.position.z = 5;
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";

    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        // if( current_state.mode != "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(5.0))){
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.success){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // } else {
        //     if( !current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(5.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }

        if (!init_local_pose_check) {
            local_pos_pub.publish(waypoint_pose[waypoint_count]);
            double dist = sqrt(
                (current_pose.pose.position.x-waypoint_pose[waypoint_count].pose.position.x)* 
                (current_pose.pose.position.x-waypoint_pose[waypoint_count].pose.position.x) + 
                (current_pose.pose.position.y-waypoint_pose[waypoint_count].pose.position.y)* 
                (current_pose.pose.position.y-waypoint_pose[waypoint_count].pose.position.y) + 
                (current_pose.pose.position.z-waypoint_pose[waypoint_count].pose.position.z)* 
                (current_pose.pose.position.z-waypoint_pose[waypoint_count].pose.position.z)); 
            ROS_INFO("distance: %.2f", dist);
            
            if (abs(current_pose.pose.position.x - waypoint_pose[waypoint_count].pose.position.x) < 0.5 && 
                abs(current_pose.pose.position.y - waypoint_pose[waypoint_count].pose.position.y) < 0.5 &&
                abs(current_pose.pose.position.z - waypoint_pose[waypoint_count].pose.position.z) < 0.5) {
                waypoint_count += 1;

                // ROS_INFO("waypoint_count = %d, cur_pos = (%.2f, %.2f, %.2f), next_pos = (%.2f, %.2f, %.2f)", waypoint_count, 
                //     current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, 
                //     waypoint_pose[waypoint_count].pose.position.x, waypoint_pose[waypoint_count].pose.position.y, waypoint_pose[waypoint_count].pose.position.z);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}