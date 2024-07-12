/**
 * @file exchange.c
 * @author Reeve Ni (reeveni666@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-07-12
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "exchange.h"
#include "robot_def.h"
#include "can_comm.h"
#include "remote_control.h"
#include "video_control.h"

static CANComm_Instance *normal_can_comm; // 一般通信CAN comm，检测remote/video
static CANComm_Instance *remote_can_comm; // remote通信CAN comm
static CANComm_Instance *video_can_comm;  // video通信CAN comm

static Normal_Recv_Data_s normal_data_recv;   // 一般通信接收到的信息
static Normal_Upload_Data_s normal_data_send; // 一般通信发送的信息
static Remote_Recv_Data_s remote_data_recv;   // remote接收到的信息
static Remote_Upload_Data_s remote_data_send; // remote发送的信息
static Video_Recv_Data_s video_data_recv;     // video接收到的信息
static Video_Upload_Data_s video_data_send;   // video发送的信息

Robot_Control_Data_s robot_ctrl; // 机器人控制数据

extern RC_ctrl_t rc_ctrl[2];
extern Video_ctrl_t video_ctrl[2];

void exchange_init()
{
    CANComm_Init_Config_s normal_comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x301,
            .rx_id = 0x302,
        },
        .recv_data_len = sizeof(Normal_Recv_Data_s),
        .send_data_len = sizeof(Normal_Upload_Data_s),
    };
    normal_can_comm = CANCommInit(&normal_comm_conf); // can comm初始化

    CANComm_Init_Config_s remote_comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Remote_Recv_Data_s),
        .send_data_len = sizeof(Remote_Upload_Data_s),
    };
    remote_can_comm = CANCommInit(&remote_comm_conf); // can comm初始化

    CANComm_Init_Config_s video_comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x322,
            .rx_id = 0x323,
        },
        .recv_data_len = sizeof(Video_Recv_Data_s),
        .send_data_len = sizeof(Video_Upload_Data_s),
    };
    video_can_comm = CANCommInit(&video_comm_conf); // can comm初始化
}

void exchange_task()
{
    normal_data_recv = *(Normal_Recv_Data_s *)CANCommGet(normal_can_comm);
    remote_data_recv = *(Remote_Recv_Data_s *)CANCommGet(remote_can_comm);
    video_data_recv = *(Video_Recv_Data_s *)CANCommGet(video_can_comm);

    if (normal_data_recv.is_remote_online)
    {
        sbus_to_rc(remote_data_recv.remote_buff);
        robot_ctrl.key_ctrl = rc_ctrl[0];
    }
    else
    {
        VideoRead(video_data_recv.video_buff);
        robot_ctrl.key_ctrl.mouse.x = video_ctrl[0].key_data.mouse_x;
        robot_ctrl.key_ctrl.mouse.y = video_ctrl[0].key_data.mouse_y;
        robot_ctrl.key_ctrl.mouse.press_l = video_ctrl[0].key_data.left_button_down;
        robot_ctrl.key_ctrl.mouse.press_r = video_ctrl[0].key_data.right_button_down;
        robot_ctrl.key_ctrl.key[3] = video_ctrl[0].key[3];
        robot_ctrl.key_ctrl.key_count[3][16] = video_ctrl[0].key_count[3][16];
    }

    robot_ctrl.top_yaw = normal_data_recv.top_yaw;
    robot_ctrl.top_pitch = normal_data_recv.top_pitch;
    robot_ctrl.vision_is_tracking = normal_data_recv.vision_is_tracking;
    robot_ctrl.friction_mode = normal_data_recv.friction_mode;
    robot_ctrl.video_mode = normal_data_recv.video_mode;
    robot_ctrl.is_friction_on = normal_data_recv.is_friction_on;
}