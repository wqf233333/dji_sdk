#ifndef __DJI_SDK_LOCALS_H__
#define __DJI_SDK_LOCALS_H__

#include <dji_sdk/global_position.h>
#include <dji_sdk/local_position.h>

namespace position_refs
{
    extern dji_sdk::local_position local_position_ref;
    extern dji_sdk::global_position global_position_ref;
    extern bool localposbase_use_height;
};

#endif
