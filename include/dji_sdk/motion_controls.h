#ifndef __DJI_SDK_MOTION_CONTROLS_H__
#define __DJI_SDK_MOTION_CONTROLS_H__

#include "motion_controls.h"
#include "../../../../devel/include/dji_sdk/velocity.h"
#include <ros/ros.h>
#include <dji_sdk/local_position.h>

namespace motion_controls
{
    void fly_to_localpos(dji_sdk::local_position los,
                         bool use_height
    );

    void set_velocity(dji_sdk::velocity msg);
};

#endif