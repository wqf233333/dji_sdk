#ifndef __DJI_SDK_LOCALS_H__
#define __DJI_SDK_LOCALS_H__

#include <dji_sdk/global_position.h>
#include <dji_sdk/local_position.h>
#include <dji_sdk/attitude_quad.h>
#include <dji_sdk/global_position.h>
#include <dji_sdk/local_position.h>
#include <dji_sdk/velocity.h>
#include <dji_sdk/acc.h>
#include <dji_sdk/gimbal.h>
#include <dji_sdk/rc_channels.h>

namespace dji_variable
{
    extern dji_sdk::local_position local_position_ref;
    extern dji_sdk::global_position global_position_ref;
    extern bool localposbase_use_height;
    extern dji_sdk::attitude_quad attitude_quad;
    extern dji_sdk::velocity velocity;
    extern dji_sdk::acc acc;
    extern dji_sdk::rc_channels rc_channels;
    extern dji_sdk::global_position global_position;
    extern dji_sdk::global_position global_position_degree;
    extern dji_sdk::local_position local_position;
    extern float battery;
};

#endif
