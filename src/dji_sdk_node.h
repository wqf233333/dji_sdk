#ifndef __DJI_SDK_NODE_H__
#define __DJI_SDK_NODE_H__

// ROS
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Quaternion.h"
// SDK
#include <stdio.h>
#include <stdlib.h>

#include "sdk_lib/sdk_lib.h"

#include <dji_sdk/attitude_quad.h>
#include <dji_sdk/global_position.h>
#include <dji_sdk/local_position.h>
#include <dji_sdk/velocity.h>
#include <dji_sdk/acc.h>
#include <dji_sdk/gimbal.h>
<<<<<<< Updated upstream
#include <dji_sdk/rc_channels.h>
=======
>>>>>>> Stashed changes

// MATH for_example
#include <math.h>

// parameter
#define C_EARTH (double) 6378137.0
#define C_PI	(double) 3.141592653589793

#define NO_AUTHORITY 8

using namespace ros;

static float ctrl_mode = 1;

// cmd agency ack func.
void cmd_callback_fun(uint16_t *ack);

void update_ros_vars();
// ros sub from serial

// ros pub for webserver


// ros timer
ros::Timer simple_task_timer;

// enc_key
static char *key;
// req_id for nav closed by app msg
static req_id_t nav_force_close_req_id = {0};
// std msg from uav
static sdk_std_msg_t recv_sdk_std_msgs = {0};

// ros launch param
std::string	serial_name;
int		baud_rate;

int		app_id;
int		app_sdk_level;
int		app_version;
std::string	app_bundle_id;

std::string     enc_key;
// activation
//static activation_data_t activation_msg = {14,2,1,""};
static activation_data_t activation_msg = {14,2,1,0};

#endif