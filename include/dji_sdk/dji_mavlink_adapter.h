#include "dji_mavlink/dji_sdk_onboard/mavlink.h"

namespace mavlink_adapter
{
    extern int fd;
    void loop_callback(long timestamp);
    void set_mavlink(std::string _tty,int _port);
}
