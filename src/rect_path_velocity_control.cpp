#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <cmath>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rect_path_velocity_control");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // rectengular path
    double setpoints [4][3] = 
    {
        {0, 0, 2},
        {2, 0, 2},
        {2, 2, 2},
        {0, 2, 2},
    };

    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 2;

    //send a few setpoints before starting
    for(int i = 20; ros::ok() && i > 0; --i){
        local_vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok() && !current_state.armed){
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
        else {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_vel_pub.publish(vel);

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("RECTENGULAR PATH FLYING BY VELOCITY START");

    int count = 0;
    double prev_position_x;
    double prev_position_y;
    double prev_position_z;

    while (ros::ok()){
        vel.linear.x =  0.8 * (setpoints[count % 4][0] - current_pose.pose.position.x) - 15 * (current_pose.pose.position.x - prev_position_x);
        vel.linear.y = 0.8 * (setpoints[count % 4][1] - current_pose.pose.position.y) - 15 * (current_pose.pose.position.y - prev_position_y);
        vel.linear.z = 0.8 * (setpoints[count % 4][2] - current_pose.pose.position.z) - 15 * (current_pose.pose.position.z - prev_position_z);

        prev_position_x = current_pose.pose.position.x;
        prev_position_y = current_pose.pose.position.y;
        prev_position_z = current_pose.pose.position.z;

        if (sqrt(pow(vel.linear.x, 2) + pow(vel.linear.y, 2) + pow(vel.linear.z, 2)) < 0.1){
            count += 1;
        }

        local_vel_pub.publish(vel);

        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}

