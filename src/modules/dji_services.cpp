//
// Created by Hao Xu on 15/5/5.
//

#include <dji_sdk/dji_services.h>
#include <dji_sdk/set_local_position_ref.h>
#include <dji_sdk/set_lookat_local.h>
#include <dji_sdk/dji_gimbal.h>
#include <dji_sdk/set_gimbal_angles.h>
#include <dji_sdk/fly_to_local.h>
#include <dji_sdk/set_velocity.h>
#include <dji_sdk/motion_controls.h>

namespace service_handles
{

    bool set_local_position_ref_cb(dji_sdk::set_local_position_refRequest &req,
                                dji_sdk::set_local_position_refResponse &rep
    )
    {
        position_refs::localposbase_use_height = req.use_height;
        position_refs::global_position_ref = req.base_pos;
        ROS_INFO("set base:%f,%f\n",
                 req.base_pos.lon,
                 req.base_pos.lat
        );
        rep.success = true;
        return true;
    }
    bool set_lookat_local_cb(dji_sdk::set_lookat_localRequest & request,
                       dji_sdk::set_lookat_localResponse & repsonse)
    {
        repsonse.sucess = true;
        gimbal::look_at(request.target);

        return true;
    }

    bool set_gimbal_angles_cb(
            dji_sdk::set_gimbal_anglesRequest & request,
            dji_sdk::set_gimbal_anglesResponse & response
    )
    {
//        printf("set lookat %f %f\n",request.gimbal_sp.pitch,
//               request.gimbal_sp.yaw) ;
        gimbal::gimbal_lookat_enable = false;
        gimbal::gimbal_yaw_control_sp = request.gimbal_sp.yaw;
        gimbal::send_gimbal_angle(
                request.gimbal_sp.yaw,
//                0,
//                request.gimbal_sp.yaw,
                request.gimbal_sp.roll,
                request.gimbal_sp.pitch
        );


        return true;
    }

    bool fly_to_local_cb(
            dji_sdk::fly_to_localRequest  & request,
            dji_sdk::fly_to_localResponse & response
    )

    {
//        printf("fly to %f %f %f",
//               request.target.x,
//               request.target.y,
//               request.target.height
//        );

        motion_controls::fly_to_localpos(request.target,true);
        response.success = true;
        return true;
    }

    bool set_velocity_cb(
            dji_sdk::set_velocityRequest & request,
            dji_sdk::set_velocityResponse & response
    )
    {
//        printf("set velocity %f %f %f\n",
//               request.vel_sp.velx,
//               request.vel_sp.vely,
//               request.vel_sp.velz
//        );

        motion_controls::set_velocity(
                request.vel_sp
        );

        response.success = true;

        return true;
    }

    ros::ServiceServer local_pos_ref,lookat_ser,gimbal_angles_ser;
    ros::ServiceServer fly_to_local,set_velocity;

    int init_services(ros::NodeHandle & n)
    {
        local_pos_ref = n.advertiseService(
                "set_local_position_ref",
                set_local_position_ref_cb
        );

        lookat_ser = n.advertiseService(
                "set_lookat_local",
                set_lookat_local_cb

        );

        gimbal_angles_ser = n.advertiseService(
                "set_gimbal_angles",
                set_gimbal_angles_cb
        );

        fly_to_local = n.advertiseService(
               "fly_to_local",
               fly_to_local_cb
        );

        set_velocity = n.advertiseService(
                "set_velocity",
                set_velocity_cb
        );


        ROS_INFO("Init services\n");

        return 0;
    }
}