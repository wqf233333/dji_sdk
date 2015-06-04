#include <dji_sdk_node.h>
#include "dji_sdk/dji_ros_modules.h"

extern static uint8_t cmd_send_flag = 1;

namespace dji_commands
{
    void cmd_takeoff_send_cb(uint16_t *ack)
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

    void set_takeoff()
    {
        uint8_t send_data = 4;
        printf("cmd %d\n", send_data);
        if (send_data > 21)
            return;

        if (cmd_send_flag) {
            App_Complex_Send_Cmd(send_data, cmd_callback_fun);
            cmd_send_flag = 0;
        }
        else {
		printf("[CMD] wating! \n");
        }
    }

    void set_land()
    {
        uint8_t send_data = 6;
        printf("cmd %d\n", send_data);
        if (send_data > 21)
            return;

        if (cmd_send_flag) {
            App_Complex_Send_Cmd(send_data, cmd_callback_fun);
            cmd_send_flag = 0;
        }
        else {
		printf("[CMD] wating! \n");
        }
    }

    void set_loiter()
    {
        uint8_t send_data = 4;
        printf("cmd %d\n", send_data);
        if (send_data > 21)
            return;

        if (cmd_send_flag) {
            App_Complex_Send_Cmd(send_data, cmd_callback_fun);
            cmd_send_flag = 0;
        }
        else {
		printf("[CMD] wating! \n");
        }
    }

    void set_return2home()
    {
        uint8_t send_data = 1;
        printf("cmd %d\n", send_data);
        if (send_data > 21)
            return;

        if (cmd_send_flag) {
            App_Complex_Send_Cmd(send_data, cmd_callback_fun);
            cmd_send_flag = 0;
        }
        else {
            printf("[CMD] wating! \n");
        }
    }
}