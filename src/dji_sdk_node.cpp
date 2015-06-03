#include "dji_sdk_node.h"
#include <dji_sdk/dji_ros_modules.h>
#include <sdk_lib/DJI_Pro_App.h>
#include <dji_sdk/mavlink_connector.h>

//----------------------------------------------------------
//table of sdk req data handler
//----------------------------------------------------------
int16_t sdk_std_msgs_handler(uint8_t cmd_id, uint8_t *pbuf, uint16_t len, req_id_t req_id);

int16_t nav_force_close_handler(uint8_t cmd_id, uint8_t *pbuf, uint16_t len, req_id_t req_id);

// cmd id table
cmd_handler_table_t cmd_handler_tab[] = {
        {0x00, sdk_std_msgs_handler},
        {0x01, nav_force_close_handler},
        {ERR_INDEX, NULL}
};
// cmd set table
set_handler_table_t set_handler_tab[] = {
        {0x02, cmd_handler_tab},
        {ERR_INDEX, NULL}
};




//----------------------------------------------------------
// sdk_req_data_callback
//----------------------------------------------------------

int16_t nav_force_close_handler(uint8_t cmd_id, uint8_t *pbuf, uint16_t len, req_id_t req_id)
{
    if (len != sizeof(uint8_t))
        return 0;
    uint8_t msg;
    memcpy(&msg, pbuf, sizeof(msg));
    // test session ack
    nav_force_close_req_id.sequence_number = req_id.sequence_number;
    nav_force_close_req_id.session_id = req_id.session_id;
    nav_force_close_req_id.reserve = 1;

    printf("WARNING nav close by app %d !!!!!!!!!!!!!! \n", msg);
    return 1;

}

#define _recv_std_msgs(_flag, _enable, _data, _buf, _datalen) \
    if( (_flag & _enable))\
    {\
        memcpy((uint8_t *)&(_data),(uint8_t *)(_buf)+(_datalen), sizeof(_data));\
        _datalen += sizeof(_data);\
    }

int16_t sdk_std_msgs_handler(uint8_t cmd_id, uint8_t *pbuf, uint16_t len, req_id_t req_id)
{
    uint16_t *msg_enable_flag = (uint16_t *) pbuf;
    uint16_t data_len = MSG_ENABLE_FLAG_LEN;

    _recv_std_msgs(*msg_enable_flag, ENABLE_MSG_TIME, recv_sdk_std_msgs.time_stamp, pbuf, data_len);
    _recv_std_msgs(*msg_enable_flag, ENABLE_MSG_Q, recv_sdk_std_msgs.q, pbuf, data_len);
    _recv_std_msgs(*msg_enable_flag, ENABLE_MSG_A, recv_sdk_std_msgs.a, pbuf, data_len);
    _recv_std_msgs(*msg_enable_flag, ENABLE_MSG_V, recv_sdk_std_msgs.v, pbuf, data_len);
    _recv_std_msgs(*msg_enable_flag, ENABLE_MSG_W, recv_sdk_std_msgs.w, pbuf, data_len);
    _recv_std_msgs(*msg_enable_flag, ENABLE_MSG_POS, recv_sdk_std_msgs.pos, pbuf, data_len);
    _recv_std_msgs(*msg_enable_flag, ENABLE_MSG_MAG, recv_sdk_std_msgs.mag, pbuf, data_len);
    _recv_std_msgs(*msg_enable_flag, ENABLE_MSG_RC, recv_sdk_std_msgs.rc, pbuf, data_len);
    _recv_std_msgs(*msg_enable_flag, ENABLE_MSG_GIMBAL, recv_sdk_std_msgs.gimbal, pbuf, data_len);
    _recv_std_msgs(*msg_enable_flag, ENABLE_MSG_STATUS, recv_sdk_std_msgs.status, pbuf, data_len);
    _recv_std_msgs(*msg_enable_flag, ENABLE_MSG_BATTERY, recv_sdk_std_msgs.battery_remaining_capacity, pbuf, data_len);
    _recv_std_msgs(*msg_enable_flag, ENABLE_MSG_DEVICE, recv_sdk_std_msgs.ctrl_device, pbuf, data_len);
    _recv_std_msgs(*msg_enable_flag, ENABLE_MSG_HACC, recv_sdk_std_msgs.hacc, pbuf, data_len);

    // testing reciever frequence
    if ((*msg_enable_flag & ENABLE_MSG_DEVICE)) {
        std_msgs::Float32 msg;
        msg.data = (float) recv_sdk_std_msgs.ctrl_device;
        publishers::test_fre_pub.publish(msg);

    }
    return 0;

}

// app_example
//----------------------------------------------------------
// mode_test
void test_activation_ack_cmd_callback(ProHeader *header)
{
    uint16_t ack_data;
    printf("Sdk_ack_cmd0_callback,sequence_number=%d,session_id=%d,data_len=%d\n", header->sequence_number,
           header->session_id, header->length - EXC_DATA_SIZE);
    memcpy((uint8_t *) &ack_data, (uint8_t *) &header->magic, (header->length - EXC_DATA_SIZE));

    if (is_sys_error(ack_data)) {
        printf("[DEBUG] SDK_SYS_ERROR!!! \n");
        std_msgs::Float32 msg;
        msg.data = NO_AUTHORITY;
        publishers::activation_status_pub.publish(msg);
    }
    else {
        char result[][50] = {{"ACTIVATION_SUCCESS"},
                             {"PARAM_ERROR"},
                             {"DATA_ENC_ERROR"},
                             {"NEW_DEVICE_TRY_AGAIN"},
                             {"DJI_APP_TIMEOUT"},
                             {" DJI_APP_NO_INTERNET"},
                             {"SERVER_REFUSED"},
                             {"LEVEL_ERROR"}};
        printf("[ACTIVATION] Activation result: %s \n", *(result + ack_data));
        std_msgs::Float32 msg;
        msg.data = (float) ack_data;
        publishers::activation_status_pub.publish(msg);

        if (ack_data == 0) {
            Pro_Config_Comm_Encrypt_Key(key);
            printf("[ACTIVATION] set key %s\n", key);
        }
    }
}

void test_activation(void)
{
//	msg.app_id 		= 0;
//	msg.app_sdk_level 	= 1;
//	msg.app_ver		= 2;
//	msg.app_bundle_id[0]	= 4;
    App_Send_Data(2, 0, MY_ACTIVATION_SET, API_USER_ACTIVATION, (uint8_t *) &activation_msg, sizeof(activation_msg),
                  test_activation_ack_cmd_callback, 1000, 1);
    printf("[ACTIVATION] send acticition msg: %d %d %d %d \n", activation_msg.app_id, activation_msg.app_sdk_level,
           activation_msg.app_ver, activation_msg.app_bundle_id[0]);
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

    if (is_sys_error(ack_data)) {
        printf("[DEBUG] SDK_SYS_ERROR!!! \n");
        std_msgs::Float32 msg;
        msg.data = NO_AUTHORITY;
        publishers::activation_status_pub.publish(msg);
    }
    else {
        printf("[DEBUG] recv_ack %#x \n", ack_data);
        char result[6][50] = {{"REQ_TIME_OUT"},
                              {"REQ_REFUSE"},
                              {"CMD_RECIEVE"},
                              {"STATUS_CMD_EXECUTING"},
                              {"STATUS_CMD_EXE_FAIL"},
                              {"STATUS_CMD_EXE_SUCCESS"}};
        printf("random_test Cmd result: %s \n", *(result + ack_data));
    }
    cmd_send_flag = 1;
}

void ros_cmd_data_callback(const std_msgs::Float32::ConstPtr &msg)
{
    uint8_t send_data = (uint8_t) msg->data;
//	printf("cmd %d\n", send_data);
    if (send_data > 21)
        return;

    if (cmd_send_flag) {
        App_Complex_Send_Cmd(send_data, cmd_callback_fun);
        cmd_send_flag = 0;
    }
    else {
//		printf("[CMD] wating! \n");
    }
}

void sdk_ack_nav_open_close_callback(ProHeader *header)
{
    uint16_t ack_data;
    printf("call %s\n", __func__);
//	printf("Recv ACK,sequence_number=%d,session_id=%d,data_len=%d\n", header->sequence_number, header->session_id, header->length - EXC_DATA_SIZE);
    memcpy((uint8_t *) &ack_data, (uint8_t *) &header->magic, (header->length - EXC_DATA_SIZE));

    std_msgs::Float32 msg;
    if (is_sys_error(ack_data)) {
        printf("[DEBUG] SDK_SYS_ERROR!!! \n");
        msg.data = NO_AUTHORITY;
        publishers::activation_status_pub.publish(msg);
    }
    else {
        msg.data = (float) ack_data;
        publishers::nav_ctrl_status_pub.publish(msg);
    }
}

void ros_nav_open_close_callback(const std_msgs::Float32::ConstPtr &msg)
{
    uint8_t send_data = (uint8_t) msg->data;
    printf("send open nav %d\n", send_data);
    App_Send_Data(1, 1, MY_CTRL_CMD_SET, API_OPEN_SERIAL, (uint8_t *) &send_data, sizeof(send_data),
                  sdk_ack_nav_open_close_callback, 1000, 0);
}


void ros_activation_callback(const std_msgs::Float32::ConstPtr &msg)
{
    printf("ros_activation_callback %f \n", msg->data);
    test_activation();
}

void gps_convert_ned(float &ned_x, float &ned_y,
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
    ned_y = d_lon * C_EARTH * cos((gps_r_lat + gps_t_lat) / 2 * M_PI / 180.0f);

    return;
}
using namespace dji_variable;
void update_ros_vars()
{

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

    global_position_degree = global_position;

    global_position_degree.lat = global_position.lat * 180.0f /M_PI;
    global_position_degree.lon = global_position.lon * 180.0f /M_PI;

    static int seted = 0;
    //TODO:
    // FIX BUG about flying at lat = 0
    if (global_position.ts != 0 && seted == 0 && global_position.lat != 0) {
        dji_variable::global_position_ref = global_position;
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
            dji_variable::global_position_ref.lon,
            dji_variable::global_position_ref.lat
    );


    local_position.height = global_position.height;
    local_position.ts = global_position.ts;
    dji_variable::local_position_ref = local_position;

    if (gimbal::gimbal_lookat_enable) {
        gimbal::control(
                local_position.x,
                local_position.y,
                local_position.height
        );
    }

    rc_channels.pitch = recv_sdk_std_msgs.rc.pitch;
    rc_channels.roll = recv_sdk_std_msgs.rc.roll;
    rc_channels.mode = recv_sdk_std_msgs.rc.mode;
    rc_channels.gear_up = recv_sdk_std_msgs.rc.gear_up;
    rc_channels.throttle = recv_sdk_std_msgs.rc.throttle;
    rc_channels.yaw = recv_sdk_std_msgs.rc.yaw;

    battery = recv_sdk_std_msgs.battery_remaining_capacity;
//    recv_sdk_std_msgs.status

    publishers::local_pos_pub.publish(local_position);

    publishers::att_quad_pub.publish(attitude_quad);

    publishers::gps_pub.publish(global_position);

    publishers::vel_pub.publish(velocity);

    publishers::acc_pub.publish(acc);

    publishers::rc_channels_pub.publish(rc_channels);
}


//----------------------------------------------------------
// timer spin_function 50Hz
//----------------------------------------------------------
void spin_callback(const ros::TimerEvent &e)
{

    update_ros_vars();

    mavlink_adapter::loop_callback(recv_sdk_std_msgs.time_stamp);

    static unsigned int count = 0;
    count++;
    if (count % 50 == 0) {
        std_msgs::Float32 msg;

        msg.data = (float) recv_sdk_std_msgs.status;
        publishers::flight_status_pub.publish(msg);

        msg.data = (float) recv_sdk_std_msgs.battery_remaining_capacity;
        publishers::battery_pub.publish(msg);


        ROS_INFO("STD_MSGS:");
        printf("[STD_MSGS] time_stamp %d \n", recv_sdk_std_msgs.time_stamp);
        printf("[STD_MSGS] q %f %f %f %f \n", recv_sdk_std_msgs.q.q0, recv_sdk_std_msgs.q.q1, recv_sdk_std_msgs.q.q2,
               recv_sdk_std_msgs.q.q3);
        printf("[STD_MSGS] a %f %f %f\n", recv_sdk_std_msgs.a.x, recv_sdk_std_msgs.a.y, recv_sdk_std_msgs.a.z);
        printf("[STD_MSGS] v %f %f %f\n", recv_sdk_std_msgs.v.x, recv_sdk_std_msgs.v.y, recv_sdk_std_msgs.v.z);
        printf("[STD_MSGS] w %f %f %f\n", recv_sdk_std_msgs.w.x, recv_sdk_std_msgs.w.y, recv_sdk_std_msgs.w.z);
        printf("[STD_MSGS] pos %f %f %f %f \n", recv_sdk_std_msgs.pos.lati, recv_sdk_std_msgs.pos.longti,
               recv_sdk_std_msgs.pos.alti, recv_sdk_std_msgs.pos.height);
        printf("[STD_MSGS] mag %d %d %d \n", recv_sdk_std_msgs.mag.x, recv_sdk_std_msgs.mag.y, recv_sdk_std_msgs.mag.z);
        printf("[STD_MSGS] rc %d %d %d %d %d %d\n", recv_sdk_std_msgs.rc.roll, recv_sdk_std_msgs.rc.pitch,
               recv_sdk_std_msgs.rc.yaw, recv_sdk_std_msgs.rc.throttle, recv_sdk_std_msgs.rc.mode,
               recv_sdk_std_msgs.rc.gear_up
        );
        printf("[STD_MSGS] gimbal %f %f %f\n", recv_sdk_std_msgs.gimbal.x, recv_sdk_std_msgs.gimbal.y,
               recv_sdk_std_msgs.gimbal.z);
        printf("[STD_MSGS] status %d\n", recv_sdk_std_msgs.status);
        printf("[STD_MSGS] battery %d\n", recv_sdk_std_msgs.battery_remaining_capacity);
        printf("[STD_MSGS] ctrl_device %d\n", recv_sdk_std_msgs.ctrl_device);
        printf("[STD_MSGS] hacc %d\n", recv_sdk_std_msgs.hacc);
    }

    // test session ack for force close
    if (nav_force_close_req_id.reserve == 1) {
        std_msgs::Float32 msg2;
        msg2.data = 4;
        publishers::nav_ctrl_status_pub.publish(msg2);
        nav_force_close_req_id.reserve = 0;

        uint16_t ack = 0x0001;
        printf("Ack close send %d !!!!!!!!!!! \n", ack);
        App_Send_Ack(nav_force_close_req_id, (uint8_t *) &ack, sizeof(ack));
    }

}

 ros::Subscriber cmd_data_sub, nav_open_close_sub,
            activation_sub;


    int init_subscibers(ros::NodeHandle &nh)
    {
        cmd_data_sub = nh.subscribe("/sdk_request_cmd", 10, ros_cmd_data_callback);
        activation_sub = nh.subscribe("/sdk_request_activation", 10, ros_activation_callback);
        nav_open_close_sub = nh.subscribe("/nav_open_close_request", 10, ros_nav_open_close_callback);

        return 0;
    }
//----------------------------------------------------------
// main_function
//----------------------------------------------------------
int main(int argc, char **argv)
{

    printf("Test SDK Protocol demo\n");
    // initialize ros
    ros::init(argc, argv, "SDK_serial");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.param("serial_name", serial_name, std::string("/dev/ttySAC0"));    // /dev/ttySAC0 or /dev/ttyUSB0
    nh_private.param("baud_rate", baud_rate, 230400);

    nh_private.param("app_id", app_id, 10086);
    nh_private.param("app_sdk_level", app_sdk_level, 2);
    nh_private.param("app_version", app_version, 1);
    nh_private.param("app_bundle_id", app_bundle_id, std::string("12345678901234567890123456789012"));

    nh_private.param("enc_key", enc_key,
                     std::string("9b7c15ee8dc3849976a779b37cdec9fe4c6308af5a03b3a570b8dc0e3c7337b8"));

    activation_msg.app_id = (uint32_t) app_id;
    activation_msg.app_sdk_level = (uint32_t) app_sdk_level;
    activation_msg.app_ver = (uint32_t) app_version;
    memcpy(activation_msg.app_bundle_id, app_bundle_id.c_str(), 32);

    key = (char *) enc_key.c_str();


    printf("[INIT] SET serial_port	: %s \n", serial_name.c_str());
    printf("[INIT] SET baud_rate	: %d \n", baud_rate);
    printf("[INIT] ACTIVATION INFO	: \n");
    printf("[INIT] 	  app_id     	  %d \n", activation_msg.app_id);
    printf("[INIT]    app_sdk_level	  %d \n", activation_msg.app_sdk_level);
    printf("[INIT]    app_version     %d \n", activation_msg.app_ver);
    printf("[INIT]    app_bundle_id	  %s \n", activation_msg.app_bundle_id);
    printf("[INIT]    enc_key	  %s \n", key);


    publishers::init_publishers(nh);
    service_handles::init_services(nh);
    init_subscibers(nh);



    // ros timer 50Hz
    simple_task_timer = nh.createTimer(ros::Duration(1.0 / 50.0), (const TimerCallback &) spin_callback);
    // open serial port
    Pro_Hw_Setup((char *) serial_name.c_str(), baud_rate);
    Pro_Link_Setup();
    App_Recv_Set_Hook(App_Recv_Req_Data);
    App_Set_Table(set_handler_tab, cmd_handler_tab);

    CmdStartThread();

    Pro_Config_Comm_Encrypt_Key(key);
    // ros spin for timer
    ros::spin();

    return 0;
}
