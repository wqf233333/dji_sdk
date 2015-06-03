//
//  mavlink_unreal.cpp
//  dSim
//
//  Created by Hao Xu on 15/4/26.
//  Copyright (c) 2015年 xuhao. All rights reserved.
//

#include "dji_sdk/mavlink_connector.h"
#include <dji_mavlink/dji_sdk_onboard/mavlink.h>
#include <dji_sdk/attitude_quad.h>
#include "dji_sdk/dji_variable.h"

namespace mavlink_adapter
{

    void mavlink_connector::init_network(std::string ip, int port)
    {
        //create a UDP socket
        if ((socket_s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
            printf("Socket Create failed");
        }


        if ((socket_s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
            printf("Failed");

        memset((char *) &si_other, 0, sizeof(si_other));
        si_other.sin_family = AF_INET;
        si_other.sin_port = htons(port);
        if (inet_aton(ip.c_str(), &si_other.sin_addr) == 0) {
            fprintf(stderr, "inet_aton() failed\n");
        }

        printf("build udp 2 %s via port %d\n", ip.c_str(), port);
    }


    int mavlink_connector::write(const char *s, int len)
    {
        const char *buf = s;
        int slen = sizeof(si_other);
        if (sendto(socket_s, buf, len, 0, (sockaddr *) &si_other, slen) == -1) {
            printf("failed:len %d\n", len);
            return -1;
        }

        return 0;
    }

    void mavlink_connector::send()
    {
        mavlink_attitude_quaternion_t att;
        mavlink_local_position_ned_t pos;
        mavlink_message_t msg;

        att.q1 = dji_variable::attitude_quad.q0;
        att.q2 = dji_variable::attitude_quad.q1;
        att.q3 = dji_variable::attitude_quad.q2;
        att.q4 = dji_variable::attitude_quad.q3;

        att.pitchspeed = dji_variable::attitude_quad.wy;
        att.rollspeed = dji_variable::attitude_quad.wx;
        att.yawspeed = dji_variable::attitude_quad.wz;

        pos.x = dji_variable::local_position.x;
        pos.y = dji_variable::local_position.y;
        pos.z = dji_variable::local_position.height;

        pos.vx = dji_variable::velocity.velx;
        pos.vy = dji_variable::velocity.vely;
        pos.vz = dji_variable::velocity.velz;


        mavlink_msg_attitude_quaternion_encode(0, 200, &msg, &att);
        send_msg(&msg);
        mavlink_msg_local_position_ned_encode(0, 200, &msg, &pos);
        send_msg(&msg);

    }

    int mavlink_connector::send_msg(mavlink_message_t * msg)
    {

        int len = mavlink_msg_to_send_buffer((uint8_t *) buffer, msg);
        write(buffer, len);
        return 0;
    }

    mavlink_connector::mavlink_connector(std::string ip, int port)
    {
        init_network(ip, port);
    }
};