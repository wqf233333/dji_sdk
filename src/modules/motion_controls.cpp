//
// Created by Hao Xu on 15/5/5.
//

#include <dji_sdk/motion_controls.h>
#include <sdk_lib/sdk_lib.h>
#include <dji_sdk/dji_variable.h>
#include <dji_sdk/dji_gimbal.h>

namespace motion_controls
{
    void fly_to_localpos(dji_sdk::local_position los,
                         bool use_height
    )
    {

        api_ctrl_without_sensor_data_t send_data = {0};


        send_data.ctrl_flag = 0x90; // mode 4
        send_data.roll_or_x = los.x - dji_variable::local_position_ref.x;
        send_data.pitch_or_y = los.y - dji_variable::local_position_ref.y;
        send_data.thr_z = los.height; //m/s
        send_data.yaw = gimbal::gimbal_yaw_control_sp;

//        printf("%f %f \n",
//               los.x - dji_variable::local_position_ref.x,
//               los.x - dji_variable::local_position_ref.x
//        );

        App_Send_Data(0, 0, MY_CTRL_CMD_SET, API_CTRL_REQUEST, (uint8_t *) &send_data, sizeof(send_data), NULL, 0, 0);
    }

    void set_velocity(dji_sdk::velocity msg)
    {

        api_ctrl_without_sensor_data_t send_data = {0};

        send_data.ctrl_flag = 68; // mode 4
        send_data.roll_or_x = msg.velx;
        send_data.pitch_or_y = msg.vely;
        send_data.thr_z = msg.velz; //m/s
        send_data.yaw = gimbal::gimbal_yaw_control_sp;

        App_Send_Data(0, 0, MY_CTRL_CMD_SET, API_CTRL_REQUEST, (uint8_t *) &send_data, sizeof(send_data), NULL, 0, 0);
    }

};
