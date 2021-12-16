/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 * 
 * from https://docs.px4.io/master/en/ros/mavros_offboard.html
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>

#include <vector>

mavros_msgs::State current_state;
mavros_msgs::PositionTarget pose_vel;
mavros_msgs::RCIn rc_in;
mavros_msgs::Thrust thrust;
sensor_msgs::Joy joy_in;
float throttle;
nav_msgs::Odometry odom;
float x_pos;
float y_pos;
float x_vel;
float y_vel;
// tf2::Quaternion enuQuaternion;
tf2::Quaternion q1, q2, qNED;
tf2::Vector3 rpy;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void rc_cb(const mavros_msgs::RCIn::ConstPtr& msg){
    rc_in = *msg;
}

void joy_cb(const sensor_msgs::Joy::ConstPtr& msg){
    joy_in = *msg;
    throttle = (joy_in.axes[1]+1)/2.0;
}

void odom_cb(const nav_msgs::OdometryConstPtr& msg){
    odom = *msg;
    x_pos = odom.pose.pose.position.y;
    y_pos = odom.pose.pose.position.x;
    x_vel = odom.twist.twist.linear.y;
    y_vel = odom.twist.twist.linear.x;

    // float x = odom.pose.pose.orientation.x;
    // float y = odom.pose.pose.orientation.y;
    // float z = odom.pose.pose.orientation.z;
    // float w = odom.pose.pose.orientation.w;

    // tf2::Quaternion qENU(x,y,z,w);
    
    // q1.setRPY(0, 0, -3.1415926535/2);
    // q2.setRPY(-3.1415926535, 0, 0);

    // qNED = q2*(q1*qENU);

    // double roll, pitch, yaw;

    // tf2::Matrix3x3(qENU).getRPY(roll, pitch, yaw);

    // std::cout << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>
            ("mavros/rc/in", 10, rc_cb);
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>
            ("joy", 10, joy_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/local_position/odom", 10, odom_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher attitude_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude", 10);
    ros::Publisher attitude_raw_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude", 10);
    ros::Publisher thrust_pub = nh.advertise<mavros_msgs::Thrust>
            ("mavros/setpoint_attitude/thrust", 10);            
    // ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    //         ("mavros/cmd/arming");
    // ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    //         ("mavros/set_mode");
    ros::Publisher local_pos_pub_mavros = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 5);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    ROS_WARN("Connected");

    throttle = 0;

    mavros_msgs::AttitudeTarget target_attitude;
    target_attitude.type_mask = 7;
    target_attitude.orientation.x = 0;
    target_attitude.orientation.y = 0;
    target_attitude.orientation.z = 0;
    target_attitude.orientation.w = 1;

    //send a few setpoints before starting
    for(int i = 200; ros::ok() && i > 0; --i){
        target_attitude.thrust = throttle;
        attitude_raw_pub.publish(target_attitude);
        ros::spinOnce();
        rate.sleep();
    }

   
    //pose_vel.coordinate_frame = pose_vel.FRAME_LOCAL_NED;
    //pose_vel.type_mask =  pose_vel.IGNORE_AFX | pose_vel.IGNORE_AFY | pose_vel.IGNORE_AFZ | pose_vel.FORCE | pose_vel.IGNORE_YAW | pose_vel.IGNORE_PX | pose_vel.IGNORE_PY | pose_vel.IGNORE_PZ;


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    Eigen::MatrixXd K(2,4);
    Eigen::VectorXd X(4);
    Eigen::VectorXd R(2);
    Eigen::VectorXd U(2);
    tf2::Quaternion q_d;
    geometry_msgs::Quaternion q_msg;
    
    // Full State Feedback
    // K << 0.0240, 0.1125, 0.1451, 0.1312,
    //      -0.0193, 0.1263, -0.0808, 0.1045;

    // LQR
    K << 0, 1.04719755, 0, 2*0.52493111,
         1.04719755, 0, 2*0.52493111, 0;

    int count = 0;

    while(ros::ok()){
        
        X << x_pos, y_pos, x_vel, y_vel;
        R << 0, 0;

        U = -0.008*K*X;

        double u1 = U(0);
        double u2 = U(1);
        if (abs(u1) > 0.1745) {u1 = sgn(u1)*0.1745;}
        if (abs(u2) > 0.1745) {u2 = sgn(u2)*0.1745;}


        q_d.setRPY(u1,u2,3.1415926535/2);

        tf2::convert(q_d, q_msg);

        std::cout << "Phi: " << U(0) << " Theta: " << U(1) << std::endl;

        target_attitude.orientation.x = q_msg.x;
        target_attitude.orientation.y = q_msg.y;
        target_attitude.orientation.z = q_msg.z;
        target_attitude.orientation.w = q_msg.w;

        target_attitude.thrust = throttle;

        attitude_raw_pub.publish(target_attitude);

        ros::spinOnce();
        rate.sleep();
        count++;
    }

    return 0;
}
