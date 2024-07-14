// /**
//  * @file exchange.c
//  * @author Reeve Ni (reeveni666@gmail.com)
//  * @brief
//  * @version 0.1
//  * @date 2024-07-12
//  *
//  * @copyright Copyright (c) 2024
//  *
//  */

// #include "exchange.h"
// #include "can_comm.h"
// #include "robot_def.h"

// // 引用全局变量
// extern uint8_t is_remote_online;
// extern uint8_t vision_is_tracking;
// extern uint8_t friction_flag;
// extern uint8_t video_mode;
// extern uint8_t is_friction_on;

// static int16_t ins_yaw = 0; // 用于接收yaw的值
// static int16_t ins_pitch = 0;

// void exchange_init()
// {
//     CANComm_Init_Config_s normal_comm_conf = {
//         .can_config = {
//             .can_handle = &hcan1,
//             .tx_id = 0x302,
//             .rx_id = 0x301,
//         },
//         .recv_data_len = sizeof(Remote_Recv_Data_s),
//         .send_data_len = sizeof(Remote_Upload_Data_s),
//     };
//     normal_can_comm = CANCommInit(&normal_comm_conf); // can comm初始化
// }

// void exchange_task()
// {
//     normal_data_send.is_remote_online = is_remote_online;
//     normal_data_send.top_yaw = ins_yaw;
//     normal_data_send.top_pitch = ins_pitch;
//     normal_data_send.vision_is_tracking = vision_is_tracking;
//     normal_data_send.friction_mode = friction_flag;
//     normal_data_send.video_mode = video_mode;
//     normal_data_send.is_friction_on = is_friction_on;
//     CANCommSend(normal_can_comm, (void *)&normal_data_send);

//     normal_data_recv = *(Normal_Recv_Data_s *)CANCommGet(normal_can_comm);
//     // 一般通信接收到的信息处理
//     // ...
// }