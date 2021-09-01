#ifndef __HELPER_H__
#define __HELPER_H__
#include <atomic>
#include <string>

//struktura pro uchovani konfigurace
typedef struct
{
    std::string general_camera_stream_path;
    std::string general_vnc_viewer;
    std::string network_robot_ip;
    int         network_camera_port;
    std::string network_ssh_user;
    std::string network_ssh_passwd;
}config_base;
//struktura pro uchovani prikazu
typedef struct
{
    std::string basic_ros_core_launch;
    std::string basic_ros_core_shutdown;
    std::string basic_ros_launch_all;
    std::string basic_reboot;
    std::string basic_shutdown;

    std::string node_ult_sensor;
    std::string node_whiskers_sensor;
    std::string node_arduino;
    std::string node_orb_slam;
    std::string node_camera;
    std::string node_autonomous_mode;
    std::string node_gui_status;
    std::string node_gui_control;
    std::string node_image_transport;

    std::string other_uv4l_launch;
    std::string other_uv4l_shutdown;
    std::string other_vnc_run;
    std::string other_wlan0_up;
    std::string other_wlan0_down;
    std::string other_ping;
    std::string other_network_restart;
    std::string other_cmd1;
    std::string other_cmd2;
    std::string other_cmd3;
    std::string other_cmd4;
    std::string other_cmd5;
    std::string other_cmd6;
    std::string other_cmd7;
}command_base;

// Tracking states
enum TrackingState
{
    SYSTEM_NOT_READY    =-1,
    NO_IMAGES_YET       =0,
    NOT_INITIALIZED     =1,
    OK                  =2,
    LOST                =3
};
enum AutonomousLevel
{
    MANUAL                = 0x00,
    COLLISION_DETECTION   = 0x01,
    WANDERER_WALK         = 0x02,
    RANDOM_WALK           = 0x03
};

#endif /* __HELPER_H__*/