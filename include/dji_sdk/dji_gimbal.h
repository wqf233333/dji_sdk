#ifndef __DJI_SDK_GIMBAL_H__
#define __DJI_SDK_GIMBAL_H__

#include <sdk_lib/sdk_lib.h>
#include <dji_sdk/local_position.h>

namespace gimbal
{

    extern float gimbal_yaw_control_sp;
    extern float gimbal_lookat_x,
            gimbal_lookat_y,
            gimbal_lookat_z;

    extern bool gimbal_lookat_enable;

    void send_gimbal_angle(float yaw,float roll,float pitch);

    void control(float x,float y,float z);

    void look_at(dji_sdk::local_position lop);
};

#endif