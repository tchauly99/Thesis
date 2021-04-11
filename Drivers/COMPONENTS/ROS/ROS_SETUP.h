/*
 * ROS_SETUP.h
 *
 *  Created on: Jan 26, 2021
 *      Author: chau
 */

#ifndef COMPONENTS_ROS_ROS_SETUP_H_
#define COMPONENTS_ROS_ROS_SETUP_H_


#include "roshihi.h"
#include "SerialClass.h"


#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

float odom_pose[3];
float odom_vel[3];

char get_prefix[10];
char *get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];

extern ScaledData_Def Accel_scaled;
extern ScaledData_Def Gyro_scaled;

extern void led_toggle(void);
void ros_setup(void);
void toggle_callback(const geometry_msgs::Twist &cmd_msg);
void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg);
void reset_callback(const std_msgs::Empty &reset_msg);
void prepub_odom(void);
void update_TF(geometry_msgs::TransformStamped& odom_tf);
void publish_driveinfo(void);
void update_TFPrefix(bool isConnected);
void update_IMU(sensor_msgs::Imu& imu_msg, float* quat, ScaledData_Def* accel_scaled, ScaledData_Def* gyro_scaled);
void publish_IMU(void);

ros::NodeHandle nh;

//std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);
std_msgs::Float32 yaw_msg;
ros::Publisher yaw_pub("yaw", &yaw_msg);
ros::Subscriber <geometry_msgs::Twist> toggle_sub("toggle", toggle_callback);


/*
 * Subscribers.
 */
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_callback);
ros::Subscriber<std_msgs::Empty> reset_sub("reset", reset_callback);

/*
 * Publishers.
 */
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

geometry_msgs::Twist cmd_vel_motor_msg;
ros::Publisher cmd_vel_motor_pub("cmd_vel_motor", &cmd_vel_motor_msg);

nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

#endif /* COMPONENTS_ROS_ROS_SETUP_H_ */
