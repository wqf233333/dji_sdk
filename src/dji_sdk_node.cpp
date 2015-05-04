/*
 ============================================================================
 Name        : dji_sdk_node.c
 Author      : Ying Jiahang, Wu Yuwei,Hao Xu
 Version     :
 Copyright   : Your copyright notice
 Description : 
 ============================================================================
 */
// ROS
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Quaternion.h"
// SDK 
#include <stdio.h>
#include <stdlib.h>
#include "sdk_comm.h"
#include "DJI_Pro_Hw.h"
#include "DJI_Pro_Link.h"
#include "DJI_Pro_App.h"

#include <dji_sdk/attitude_quad.h>
#include <dji_sdk/global_position.h>
#include <dji_sdk/local_position.h>
#include <dji_sdk/velocity.h>
#include <dji_sdk/acc.h>
#include <dji_sdk/gimbal.h>
#include <dji_sdk/set_local_position_ref.h>

// MATH for_example
#include <math.h>
#include <DJI_Pro_App.h>

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
ros::Subscriber cmd_data_sub,nav_open_close_sub,
        ctrl_mode_sub, ctrl_data_sub, simple_task_sub,
        activation_sub;
ros::Subscriber vel_sp_sub,acc_sp_sub,
        gimbal_sp_sub,pos_sp_sub,
        pos_sp,look_at_sp_sub;
// ros pub for webserver
ros::Publisher battery_pub, nav_ctrl_status_pub,
        flight_status_pub, activation_status_pub, test_fre_pub,acc_pub;

ros::Publisher gps_pub,att_quad_pub,
        vel_pub,local_pos_pub;


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
static activation_data_t activation_msg = {14,2,1,""};

namespace position_refs
{
    dji_sdk::local_position local_position_ref;
    dji_sdk::global_position global_position_ref;
    bool localposbase_use_height  = true;
};

namespace service_handles
{
    bool set_local_position_ref (dji_sdk::set_local_position_refRequest & req,
                                 dji_sdk::set_local_position_refResponse&  rep
    )
    {
        position_refs::localposbase_use_height = req.use_height;
        position_refs::global_position_ref = req.base_pos;
        ROS_INFO("set base:%f,%f\n",
                 req.base_pos.lon,
                 req.base_pos.lat
        );
        rep.success = true;
        return true;
    }

    ros::ServiceServer local_pos_ref;
    int init_services(ros::NodeHandle  n)
    {
        local_pos_ref = n.advertiseService(
                "set_local_position_ref",
                set_local_position_ref
        );

        ROS_INFO("Init services\n");

        return 0;
    }
};

namespace gimbal
{

    float gimbal_yaw_control_sp = 0;
    float gimbal_lookat_x,
            gimbal_lookat_y,
            gimbal_lookat_z;
    bool gimbal_lookat_enable = false;

    void send_gimbal_angle(float yaw,float roll,float pitch)
    {
        gimbal_custom_control_angle_t send_data = {0};

        send_data.yaw_angle   =  floor(yaw * 10.0f);  // unit 0.1 degree
        send_data.roll_angle  = floor(roll * 10.0f);
        send_data.pitch_angle = floor(pitch * 10.0f);
        send_data.duration = 10;
        send_data.ctrl_byte.base = 1;
        App_Send_Data(0, 0, MY_CTRL_CMD_SET,API_CTRL_GIMBAL_ANGLE, (uint8_t*)&send_data, sizeof(send_data), NULL, 0, 0);
    }

    void control(float x,float y,float z)
    {

        float dx = gimbal_lookat_x - x;
        float dy = gimbal_lookat_y - y;
        float dz = gimbal_lookat_z - z;
        float theta = atan(dy/dx) * 180.0f/M_PI;
        if (fabs(dx) < 1e-4)
        {
            if (dy>0)
                theta = 90;
            else
                theta = -90;
        }

        if(dx<0)
        {
            theta = 180 - theta;
        }

        if (theta > 180)
            theta -= 360;
        if (theta < -180)
            theta += 360;

        float fai = atan( dz / sqrt( dx*dx + dy*dy )) * 180.0f /M_PI;


        if (dx*dx+dy*dy<1e-2)
        {
           fai = 0;
        }

        gimbal::gimbal_yaw_control_sp = theta;
        send_gimbal_angle(0,0,fai);

    }
};
//----------------------------------------------------------
//table of sdk req data handler
//----------------------------------------------------------
int16_t sdk_std_msgs_handler(uint8_t cmd_id,uint8_t* pbuf,uint16_t len,req_id_t req_id);
int16_t	nav_force_close_handler(uint8_t cmd_id,uint8_t* pbuf,uint16_t len,req_id_t req_id);
// cmd id table
cmd_handler_table_t cmd_handler_tab[]={
	{0x00,sdk_std_msgs_handler			},
	{0x01,nav_force_close_handler			},
	{ERR_INDEX,NULL					}
};
// cmd set table
set_handler_table_t set_handler_tab[]={
	{0x02,cmd_handler_tab				},
	{ERR_INDEX,NULL					}
};

/*
void ros_ctrl_mode_callback(const std_msgs::Float32::ConstPtr& msg)
{
    ctrl_mode = (float) msg->data;
    printf("mode %f\n", ctrl_mode);
}
*/

void look_at_sp_cb(dji_sdk::local_position lop)
{
    gimbal::gimbal_lookat_enable = true;
    gimbal::gimbal_lookat_x = lop.x;
    gimbal::gimbal_lookat_y = lop.y;
    gimbal::gimbal_lookat_z = lop.height;
}


//----------------------------------------------------------
// sdk_req_data_callback
//----------------------------------------------------------

int16_t nav_force_close_handler(uint8_t cmd_id,uint8_t* pbuf,uint16_t len,req_id_t req_id)
{
	if(len != sizeof(uint8_t))
		return 0;
	uint8_t msg;
	memcpy(&msg, pbuf, sizeof(msg));
	// test session ack
	nav_force_close_req_id.sequence_number = req_id.sequence_number;
	nav_force_close_req_id.session_id      = req_id.session_id;
	nav_force_close_req_id.reserve	       = 1;

	printf("WARNING nav close by app %d !!!!!!!!!!!!!! \n", msg);
	return 1;

}

#define _recv_std_msgs(_flag, _enable, _data, _buf, _datalen) \
	if( (_flag & _enable))\
	{\
		memcpy((uint8_t *)&(_data),(uint8_t *)(_buf)+(_datalen), sizeof(_data));\
		_datalen += sizeof(_data);\
	}

int16_t sdk_std_msgs_handler(uint8_t cmd_id,uint8_t* pbuf,uint16_t len,req_id_t req_id)
{
	uint16_t *msg_enable_flag = (uint16_t *)pbuf;
	uint16_t data_len = MSG_ENABLE_FLAG_LEN;

	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_TIME	, recv_sdk_std_msgs.time_stamp			, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_Q		, recv_sdk_std_msgs.q				, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_A		, recv_sdk_std_msgs.a				, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_V		, recv_sdk_std_msgs.v				, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_W		, recv_sdk_std_msgs.w				, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_POS	, recv_sdk_std_msgs.pos				, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_MAG	, recv_sdk_std_msgs.mag				, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_RC		, recv_sdk_std_msgs.rc				, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_GIMBAL	, recv_sdk_std_msgs.gimbal			, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_STATUS	, recv_sdk_std_msgs.status			, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_BATTERY	, recv_sdk_std_msgs.battery_remaining_capacity	, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_DEVICE	, recv_sdk_std_msgs.ctrl_device			, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_HACC	, recv_sdk_std_msgs.hacc			, pbuf, data_len);

	// testing reciever frequence
	if( (*msg_enable_flag & ENABLE_MSG_DEVICE))
	{
		std_msgs::Float32 msg;
		msg.data = (float)recv_sdk_std_msgs.ctrl_device;
		test_fre_pub.publish(msg);
		
	}

}

// app_example
//----------------------------------------------------------
// mode_test
void test_activation_ack_cmd_callback(ProHeader *header)
{
	uint16_t ack_data;
	printf("Sdk_ack_cmd0_callback,sequence_number=%d,session_id=%d,data_len=%d\n", header->sequence_number, header->session_id, header->length - EXC_DATA_SIZE);
	memcpy((uint8_t *)&ack_data,(uint8_t *)&header->magic, (header->length - EXC_DATA_SIZE));

	if( is_sys_error(ack_data))
	{
		printf("[DEBUG] SDK_SYS_ERROR!!! \n");
		std_msgs::Float32 msg;
		msg.data = NO_AUTHORITY;
		activation_status_pub.publish(msg);
	}
	else
	{
		char result[][50]={{"ACTIVATION_SUCCESS"},{"PARAM_ERROR"},{"DATA_ENC_ERROR"},{"NEW_DEVICE_TRY_AGAIN"},{"DJI_APP_TIMEOUT"},{" DJI_APP_NO_INTERNET"},{"SERVER_REFUSED"},{"LEVEL_ERROR"}};
		printf("[ACTIVATION] Activation result: %s \n", *(result+ack_data));
		std_msgs::Float32 msg;
		msg.data = (float)ack_data;
		activation_status_pub.publish(msg);

		if(ack_data == 0)
		{
			Pro_Config_Comm_Encrypt_Key(key);
			printf("[ACTIVATION] set key %s\n",key);
		}
	}
}

void test_activation(void)
{
//	msg.app_id 		= 0;
//	msg.app_sdk_level 	= 1;
//	msg.app_ver		= 2;
//	msg.app_bundle_id[0]	= 4;
	App_Send_Data( 2, 0, MY_ACTIVATION_SET, API_USER_ACTIVATION,(uint8_t*)&activation_msg,sizeof(activation_msg), test_activation_ack_cmd_callback, 1000, 1);
	printf("[ACTIVATION] send acticition msg: %d %d %d %d \n", activation_msg.app_id, activation_msg.app_sdk_level, activation_msg.app_ver ,activation_msg.app_bundle_id[0]);
}
//----------------------------------------------------------
// ros_callback function
//----------------------------------------------------------
static uint8_t cmd_send_flag = 1;

void cmd_callback_fun(uint16_t *ack)
{
	/*
	*	#define	REQ_TIME_OUT			0x0000
		#define REQ_REFUSE			0x0001
		#define CMD_RECIEVE			0x0002
		#define STATUS_CMD_EXECUTING		0x0003
		#define STATUS_CMD_EXE_FAIL		0x0004
		#define STATUS_CMD_EXE_SUCCESS		0x0005
	*/
	uint16_t ack_data = *ack;

	if( is_sys_error(ack_data))
	{
		printf("[DEBUG] SDK_SYS_ERROR!!! \n");
		std_msgs::Float32 msg;
		msg.data = NO_AUTHORITY;
		activation_status_pub.publish(msg);
	}
	else
	{
		printf("[DEBUG] recv_ack %#x \n", ack_data);
		char result[6][50]={{"REQ_TIME_OUT"},{"REQ_REFUSE"},{"CMD_RECIEVE"},{"STATUS_CMD_EXECUTING"},{"STATUS_CMD_EXE_FAIL"},{"STATUS_CMD_EXE_SUCCESS"}};
		printf("random_test Cmd result: %s \n", *(result+ack_data));
	}
	cmd_send_flag = 1;
} 

void ros_cmd_data_callback(const std_msgs::Float32::ConstPtr& msg)
{
	uint8_t send_data = (uint8_t)msg->data;
//	printf("cmd %d\n", send_data);
	if( send_data > 21)
		return;

	if(cmd_send_flag)
	{
		App_Complex_Send_Cmd(send_data, cmd_callback_fun);
		cmd_send_flag = 0;
	}
	else
	{
//		printf("[CMD] wating! \n");
	}
}

void sdk_ack_nav_open_close_callback(ProHeader *header)
{
	uint16_t ack_data;
	printf("call %s\n",__func__);
//	printf("Recv ACK,sequence_number=%d,session_id=%d,data_len=%d\n", header->sequence_number, header->session_id, header->length - EXC_DATA_SIZE);
	memcpy((uint8_t *)&ack_data,(uint8_t *)&header->magic, (header->length - EXC_DATA_SIZE));

	std_msgs::Float32 msg;
	if( is_sys_error(ack_data))
	{
		printf("[DEBUG] SDK_SYS_ERROR!!! \n");
		std_msgs::Float32 msg;
		msg.data = NO_AUTHORITY;
		activation_status_pub.publish(msg);
	}
	else
	{
		msg.data = (float)ack_data;
		nav_ctrl_status_pub.publish(msg);
	}
}

void ros_nav_open_close_callback(const std_msgs::Float32::ConstPtr& msg)
{
	uint8_t send_data = (uint8_t)msg->data;
	printf("send open nav %d\n",send_data);
	App_Send_Data(1, 1, MY_CTRL_CMD_SET, API_OPEN_SERIAL, (uint8_t*)&send_data, sizeof(send_data), sdk_ack_nav_open_close_callback,  1000, 0);
}


void ros_activation_callback(const std_msgs::Float32::ConstPtr& msg)
{
	printf("ros_activation_callback %f \n",msg->data);
	test_activation();
}

void vel_sp_cb(const dji_sdk::velocity& msg)
{

    api_ctrl_without_sensor_data_t send_data = {0};

    send_data.ctrl_flag 	= 68; // mode 4
    send_data.roll_or_x 	= msg.velx;
    send_data.pitch_or_y 	= msg.vely;
    send_data.thr_z 	= msg.velz; //m/s
    send_data.yaw 		= gimbal::gimbal_yaw_control_sp;

    App_Send_Data(0, 0, MY_CTRL_CMD_SET, API_CTRL_REQUEST, (uint8_t*)&send_data, sizeof(send_data), NULL, 0, 0);
}


void gimbal_sp_cb(const dji_sdk::gimbal gimbal)
{

    gimbal::gimbal_yaw_control_sp = gimbal.yaw;
    gimbal::gimbal_lookat_enable = false;
    gimbal::send_gimbal_angle(
            0,gimbal.roll,gimbal.pitch
    );

}


void local_pos_sp_cb(const dji_sdk::local_position lo)
{
    api_ctrl_without_sensor_data_t send_data = {0};


    send_data.ctrl_flag 	= 0x90; // mode 4
    send_data.roll_or_x 	= lo.x - position_refs::local_position_ref.x;
    send_data.pitch_or_y 	= lo.y - position_refs::local_position_ref.y;
    send_data.thr_z 	= lo.height; //m/s
    send_data.yaw 		= gimbal::gimbal_yaw_control_sp ;

    App_Send_Data(0, 0, MY_CTRL_CMD_SET, API_CTRL_REQUEST, (uint8_t*)&send_data, sizeof(send_data), NULL, 0, 0);
}
void gps_convert_ned(float & ned_x,float & ned_y,
                        double gps_t_lon,
                        double gps_t_lat,
                        double gps_r_lon,
                        double gps_r_lat
)
{

    //TODO :
    //fix bug with ellipsoid

    double d_lon = gps_t_lon - gps_r_lon;
    double d_lat = gps_t_lat - gps_r_lat;

    ned_x = d_lat * C_EARTH;
    ned_y = d_lon * C_EARTH * cos( (gps_r_lat+gps_t_lat)/2 * M_PI / 180.0f);

    return;
}
void update_ros_vars()
{
    dji_sdk::attitude_quad attitude_quad;
    dji_sdk::global_position global_position;
    dji_sdk::local_position local_position;
    dji_sdk::velocity velocity;
    dji_sdk::acc acc;

    attitude_quad.q0 = recv_sdk_std_msgs.q.q0;
    attitude_quad.q1 = recv_sdk_std_msgs.q.q1;
    attitude_quad.q2 = recv_sdk_std_msgs.q.q2;
    attitude_quad.q3 = recv_sdk_std_msgs.q.q3;
    attitude_quad.ts = recv_sdk_std_msgs.time_stamp;

    global_position.lat = recv_sdk_std_msgs.pos.lati;
    global_position.lon = recv_sdk_std_msgs.pos.longti;
    global_position.height = recv_sdk_std_msgs.pos.height;
    global_position.alti = recv_sdk_std_msgs.pos.alti;
    global_position.ts = recv_sdk_std_msgs.time_stamp;

    static int seted = 0;
    //TODO:
    // FIX BUG about flying at lat = 0
    if (global_position.ts != 0 && seted == 0 && global_position.lat != 0)
    {
        position_refs::global_position_ref = global_position;
        seted = 1;
    }

    velocity.ts = recv_sdk_std_msgs.time_stamp;
    velocity.velx = recv_sdk_std_msgs.v.x;
    velocity.vely = recv_sdk_std_msgs.v.y;
    velocity.velz = recv_sdk_std_msgs.v.z;

    acc.ax = recv_sdk_std_msgs.a.x;
    acc.ay = recv_sdk_std_msgs.a.y;
    acc.az = recv_sdk_std_msgs.a.z;

    gps_convert_ned(
            local_position.x,
            local_position.y,
            global_position.lon,
            global_position.lat,
            position_refs::global_position_ref.lon,
            position_refs::global_position_ref.lat
    );


    local_position.height = global_position.height;
    local_position.ts = global_position.ts;
    position_refs::local_position_ref = local_position;

    if (gimbal::gimbal_lookat_enable)
    {
        gimbal::control(
               local_position.x,
               local_position.y,
               local_position.height
        );
    }

    local_pos_pub.publish(local_position);

    att_quad_pub.publish(attitude_quad);

    gps_pub.publish(global_position);

    vel_pub.publish(velocity);

    acc_pub.publish(acc);
}


//----------------------------------------------------------
// timer spin_function 50Hz
//----------------------------------------------------------
void spin_callback(const ros::TimerEvent& e)
{

    update_ros_vars();

	static unsigned int count = 0;
	count++;
	if(count % 50 == 0)
	{
		std_msgs::Float32 msg;

		msg.data = (float)recv_sdk_std_msgs.status;
		flight_status_pub.publish(msg);

		msg.data = (float)recv_sdk_std_msgs.battery_remaining_capacity;
		battery_pub.publish(msg);


		ROS_INFO("STD_MSGS:");
		printf("[STD_MSGS] time_stamp %d \n",recv_sdk_std_msgs.time_stamp);
		printf("[STD_MSGS] q %f %f %f %f \n",recv_sdk_std_msgs.q.q0,recv_sdk_std_msgs.q.q1,recv_sdk_std_msgs.q.q2,recv_sdk_std_msgs.q.q3);
		printf("[STD_MSGS] a %f %f %f\n",recv_sdk_std_msgs.a.x,recv_sdk_std_msgs.a.y,recv_sdk_std_msgs.a.z);
		printf("[STD_MSGS] v %f %f %f\n",recv_sdk_std_msgs.v.x,recv_sdk_std_msgs.v.y,recv_sdk_std_msgs.v.z);
		printf("[STD_MSGS] w %f %f %f\n",recv_sdk_std_msgs.w.x,recv_sdk_std_msgs.w.y,recv_sdk_std_msgs.w.z);
		printf("[STD_MSGS] pos %f %f %f %f \n",recv_sdk_std_msgs.pos.lati, recv_sdk_std_msgs.pos.longti, recv_sdk_std_msgs.pos.alti, recv_sdk_std_msgs.pos.height);
		printf("[STD_MSGS] mag %d %d %d \n",recv_sdk_std_msgs.mag.x,recv_sdk_std_msgs.mag.y,recv_sdk_std_msgs.mag.z);
		printf("[STD_MSGS] rc %d %d %d %d %d\n",recv_sdk_std_msgs.rc.roll, recv_sdk_std_msgs.rc.pitch, recv_sdk_std_msgs.rc.yaw, recv_sdk_std_msgs.rc.throttle,recv_sdk_std_msgs.rc.mode);
		printf("[STD_MSGS] gimbal %f %f %f\n",recv_sdk_std_msgs.gimbal.x, recv_sdk_std_msgs.gimbal.y,recv_sdk_std_msgs.gimbal.z);
		printf("[STD_MSGS] status %d\n",recv_sdk_std_msgs.status);
		printf("[STD_MSGS] battery %d\n",recv_sdk_std_msgs.battery_remaining_capacity);
		printf("[STD_MSGS] ctrl_device %d\n",recv_sdk_std_msgs.ctrl_device);
		printf("[STD_MSGS] hacc %d\n",recv_sdk_std_msgs.hacc);
	}

	// test session ack for force close
	if(nav_force_close_req_id.reserve == 1)
	{
		std_msgs::Float32 msg2;
		msg2.data = 4;
		nav_ctrl_status_pub.publish(msg2);
		nav_force_close_req_id.reserve = 0;

		uint16_t ack = 0x0001;
		printf("Ack close send %d !!!!!!!!!!! \n", ack);
		App_Send_Ack(nav_force_close_req_id, (uint8_t *)&ack, sizeof(ack));
	}

}
//----------------------------------------------------------
// main_function
//----------------------------------------------------------
int main (int argc, char** argv)
{

	printf("Test SDK Protocol demo\n");
	// initialize ros
	ros::init(argc, argv, "SDK_serial");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	nh_private.param("serial_name", serial_name, std::string("/dev/ttySAC0")); 	// /dev/ttySAC0 or /dev/ttyUSB0
	nh_private.param("baud_rate", baud_rate, 230400);

	nh_private.param("app_id", app_id, 10086);
	nh_private.param("app_sdk_level", app_sdk_level, 2);
	nh_private.param("app_version", app_version, 1);
	nh_private.param("app_bundle_id", app_bundle_id, std::string("12345678901234567890123456789012"));

	nh_private.param("enc_key", enc_key, std::string("9b7c15ee8dc3849976a779b37cdec9fe4c6308af5a03b3a570b8dc0e3c7337b8"));

	activation_msg.app_id 		= (uint32_t)app_id;
	activation_msg.app_sdk_level 	= (uint32_t)app_sdk_level;
	activation_msg.app_ver		= (uint32_t)app_version;
	memcpy(activation_msg.app_bundle_id, app_bundle_id.c_str(), 32); 
	
	key = (char*)enc_key.c_str();
	

	printf("[INIT] SET serial_port	: %s \n", serial_name.c_str());
	printf("[INIT] SET baud_rate	: %d \n", baud_rate);
	printf("[INIT] ACTIVATION INFO	: \n");
	printf("[INIT] 	  app_id     	  %d \n", activation_msg.app_id);
	printf("[INIT]    app_sdk_level	  %d \n", activation_msg.app_sdk_level);
	printf("[INIT]    app_version     %d \n", activation_msg.app_ver);
	printf("[INIT]    app_bundle_id	  %s \n", activation_msg.app_bundle_id);
	printf("[INIT]    enc_key	  %s \n", key);

	
	//printf("api_ctrl_without_sensor_data_t %d \n",sizeof(api_ctrl_without_sensor_data_t));
	//printf("api_ctrl_without_packed_data_t %d \n",sizeof(api_ctrl_without_sensor_packed_data_t));
	

	// start ros subscriber
    /*
	ctrl_data_sub		= nh.subscribe("/sdk_request_ctrl", 10, ros_ctrl_data_callback);
	simple_task_sub		= nh.subscribe("/sdk_request_simple_task", 10, ros_simple_task_callback);
     */
//    ctrl_mode_sub		= nh.subscribe("/sdk_request_ctrl_mode", 10, ros_ctrl_mode_callback);

    cmd_data_sub 		= nh.subscribe("/sdk_request_cmd", 10, ros_cmd_data_callback);
	activation_sub		= nh.subscribe("/sdk_request_activation", 10, ros_activation_callback);
    nav_open_close_sub      = nh.subscribe("/nav_open_close_request", 10, ros_nav_open_close_callback);

    vel_sp_sub = nh.subscribe("/velocity_setpoint",10,vel_sp_cb);
    gimbal_sp_sub = nh.subscribe("/gimbal_setpoint",10,gimbal_sp_cb);
    pos_sp_sub = nh.subscribe("/local_position_setpoint",10,local_pos_sp_cb);
    look_at_sp_sub = nh.subscribe(
            "/lookat_setpoint",
            10,
            look_at_sp_cb
    );

	// start ros publisher
    battery_pub 		= nh.advertise<std_msgs::Float32>("/battery_status", 10);
    nav_ctrl_status_pub 	= nh.advertise<std_msgs::Float32>("/nav_open_close_status", 10);
    flight_status_pub 	= nh.advertise<std_msgs::Float32>("/flight_status", 10);
	activation_status_pub   = nh.advertise<std_msgs::Float32>("/activation_status", 10);
	test_fre_pub		= nh.advertise<std_msgs::Float32>("/test_fre", 10);

    acc_pub = nh.advertise<dji_sdk::acc>("/acceleration",10);

    //extend pub

    att_quad_pub = nh.advertise<dji_sdk::attitude_quad>("/attitude_quad",10);
    gps_pub = nh.advertise<dji_sdk::global_position>("/global_position",10);
    local_pos_pub = nh.advertise<dji_sdk::local_position>("/local_position",10);
    vel_pub = nh.advertise<dji_sdk::velocity>("/velocity",10);

    service_handles::init_services(nh);


	// ros timer 50Hz
	simple_task_timer 	= nh.createTimer(ros::Duration(1.0/50.0), spin_callback);
	// open serial port
	Pro_Hw_Setup((char *)serial_name.c_str(),baud_rate);
	Pro_Link_Setup();
	App_Recv_Set_Hook(App_Recv_Req_Data);
	App_Set_Table(set_handler_tab, cmd_handler_tab);

	CmdStartThread();

	Pro_Config_Comm_Encrypt_Key(key);
	// ros spin for timer
	ros::spin();

	return 0;
}
