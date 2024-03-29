#include <dji_sdk/dji_variable.h>
#include <dji_sdk/attitude_quad.h>
#include <dji_sdk/velocity.h>

namespace dji_variable
{
    dji_sdk::local_position local_position_ref;
    dji_sdk::global_position global_position_ref;
    bool localposbase_use_height  = true;
    dji_sdk::attitude_quad attitude_quad;
    dji_sdk::velocity velocity;
    dji_sdk::acc acc;
    dji_sdk::rc_channels rc_channels;
    dji_sdk::global_position global_position;
    dji_sdk::global_position global_position_degree;
    dji_sdk::local_position local_position;
    nav_msgs::Odometry odem;
    uint8_t  flight_status;
    uint8_t ctrl_device;
    float battery = 0;
};
