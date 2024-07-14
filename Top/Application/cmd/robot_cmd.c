/**
 * @file robot_cmd.c
 * @author Reeve Ni (reeveni666@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-07-14
 *
 * @copyright Copyright (c) 2024
 *
 */

// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "video_control.h"
#include "ins_task.h"
#include "miniPC_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "can_comm.h"
// bsp
#include "bsp_dwt.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
static CANComm_Instance *cmd_can_comm;           // 双板通信
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Video_ctrl_t *video_data;        // 图传数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态
extern uint8_t is_remote_online;   // 遥控器在线状态

static int16_t chassis_speed_max;                                        // 底盘速度最大值
static int16_t chassis_speed_buff[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}; // 底盘速度缓冲区

void RobotCMDInit()
{
    Vision_Init_Config_s config = {
        .recv_config = {
            .header = VISION_RECV_HEADER,
        },
        .send_config = {
            .header = VISION_SEND_HEADER,
            .detect_color = VISION_DETECT_COLOR_BLUE,
            .reset_tracker = VISION_RESET_TRACKER_NO,
            .is_shoot = VISION_SHOOTING,
            .tail = VISION_SEND_TAIL,
        },
        .usart_config = {
            .recv_buff_size = VISION_RECV_SIZE,
            .usart_handle = &huart6,
        },
    };
    vision_recv_data = VisionInit(&config);
    rc_data = RemoteControlInit(&huart3);              // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    video_data = VideoTransmitterControlInit(&huart6); // 修改为对应串口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);

    gimbal_cmd_send.pitch = 0;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096            // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW;

    // 发射参数
    chassis_cmd_send.loader_mode = LOAD_SPEED;
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[下],停止发射
    {
        shoot_cmd_send.friction_mode = FRICTION_STOP;
        chassis_cmd_send.loader_mode = LOAD_STOP;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[中],摩擦轮启动
    {
        shoot_cmd_send.friction_mode = FRICTION_NORMAL;
        chassis_cmd_send.loader_mode = LOAD_STOP;
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[上],发射
    {
        shoot_cmd_send.friction_mode = FRICTION_NORMAL;
        chassis_cmd_send.loader_mode = LOAD_STOP;
    }

    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    chassis_cmd_send.vx = 10.0f * rc_data[TEMP].rc.rocker_r_;                                                  // _水平方向
    chassis_cmd_send.vy = 10.0f * rc_data[TEMP].rc.rocker_r1;                                                  // 1数值方向
    chassis_cmd_send.wz = 100.0f * (gimbal_fetch_data.yaw_angle - YAW_CHASSIS_ALIGN_ECD) * ECD_ANGLE_COEF_DJI; // 旋转方向

    // 云台参数
    gimbal_cmd_send.yaw += 0.005f * (float)rc_data[TEMP].rc.rocker_l_; // 系数待测
    gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
}

/**
 * @brief 遥控器链路键鼠时模式和控制量设置
 *
 */
static void RemoteMouseKeySet()
{
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 2)
    {
    case 1:
        chassis_cmd_send.supcap_mode = SUPCAP_ON;
        chassis_speed_max = chassis_speed_buff[chassis_fetch_data.robot_level - 1] + 12000;
        break;
    default:
        chassis_cmd_send.supcap_mode = SUPCAP_OFF;
        chassis_speed_max = chassis_speed_buff[chassis_fetch_data.robot_level - 1];
        break;
    }

    chassis_cmd_send.vx = (rc_data[TEMP].key[KEY_PRESS].d - rc_data[TEMP].key[KEY_PRESS].a) * chassis_speed_max; // 系数待测
    chassis_cmd_send.vy = (rc_data[TEMP].key[KEY_PRESS].w - rc_data[TEMP].key[KEY_PRESS].s) * chassis_speed_max;
    if (rc_data[TEMP].key[KEY_PRESS].shift) // 小陀螺
        chassis_cmd_send.wz = 24000;        // 待测
    else
        chassis_cmd_send.wz = 100.0f * (gimbal_fetch_data.yaw_angle - YAW_CHASSIS_ALIGN_ECD) * ECD_ANGLE_COEF_DJI; // 旋转方向

    if (rc_data[TEMP].mouse.press_r && vision_recv_data->is_tracking) // 视觉
    {
        gimbal_cmd_send.yaw = vision_recv_data->yaw;
        gimbal_cmd_send.pitch = vision_recv_data->pitch;
    }
    else
    {
        gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
        gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;
    }

    if (gimbal_cmd_send.yaw > 180)
        gimbal_cmd_send.yaw -= 360;
    else if (gimbal_cmd_send.yaw < -180)
        gimbal_cmd_send.yaw += 360;

    if (gimbal_cmd_send.pitch > PITCH_MAX) // 软件限位
        gimbal_cmd_send.pitch = PITCH_MAX;
    else if (gimbal_cmd_send.pitch < PITCH_MIN)
        gimbal_cmd_send.pitch = PITCH_MIN;

    // F键控制底盘模式
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2)
    {
    case 1:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW; // normal
        break;
    default:
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE; // stop
        break;
    }

    // Q键控制拨盘模式
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Q] % 3)
    {
    case 1:
        chassis_cmd_send.loader_mode = LOAD_SPEED;
        chassis_cmd_send.loader_ui_mode = LOAD_SPEED;
        break;
    case 2:
        chassis_cmd_send.loader_mode = LOAD_BUFF;
        chassis_cmd_send.loader_ui_mode = LOAD_BUFF;
        break;
    default:
        chassis_cmd_send.loader_mode = LOAD_SINGLE;
        chassis_cmd_send.loader_ui_mode = LOAD_SINGLE;
        break;
    }

    if (rc_data[TEMP].key[KEY_PRESS].z) // 拨盘反转
        chassis_cmd_send.loader_mode = LOAD_REVERSE;
    else if (!rc_data[TEMP].mouse.press_l)
        chassis_cmd_send.loader_mode = LOAD_STOP;

    // E键切换摩擦轮速度
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 4)
    {
    case 1:
        shoot_cmd_send.friction_mode = FRICTION_NORMAL;
        chassis_cmd_send.friction_mode = FRICTION_NORMAL;
        break;
    case 2:
        shoot_cmd_send.friction_mode = FRICTION_LOW;
        chassis_cmd_send.friction_mode = FRICTION_LOW;
        break;
    case 3:
        shoot_cmd_send.friction_mode = FRICTION_HIGH;
        chassis_cmd_send.friction_mode = FRICTION_HIGH;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_STOP;
        chassis_cmd_send.friction_mode = FRICTION_STOP;
        break;
    }

    // B键切换图传模式，normal，adaptive
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_B] % 2)
    {
    case 1:
        shoot_cmd_send.video_mode = VIDEO_ADAPTIVE;
        chassis_cmd_send.video_mode = VIDEO_ADAPTIVE;
        break;
    default:
        shoot_cmd_send.video_mode = VIDEO_NORMAL;
        chassis_cmd_send.video_mode = VIDEO_NORMAL;
        break;
    }

    // G键切换开镜模式
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_G] % 2)
    {
    case 1:
        shoot_cmd_send.lens_mode = LENS_ON;
        break;
    default:
        shoot_cmd_send.lens_mode = LENS_OFF;
        break;
    }

    if (rc_data[TEMP].key[KEY_PRESS].x) // 刷新ui
        chassis_cmd_send.ui_mode = UI_REFRESH;
    else
        chassis_cmd_send.ui_mode = UI_KEEP;
}

/**
 * @brief 图传链路键鼠时模式和控制量设置
 *
 */
static void VideoMouseKeySet()
{
    // 开关超电
    switch (video_data[TEMP].key_count[KEY_PRESS][Key_C] % 2)
    {
    case 1:
        chassis_cmd_send.supcap_mode = SUPCAP_ON;
        chassis_speed_max = chassis_speed_buff[chassis_fetch_data.robot_level - 1] + 12000;
        break;
    default:
        chassis_cmd_send.supcap_mode = SUPCAP_OFF;
        chassis_speed_max = chassis_speed_buff[chassis_fetch_data.robot_level - 1];
        break;
    }

    chassis_cmd_send.vx = (video_data[TEMP].key[KEY_PRESS].d - video_data[TEMP].key[KEY_PRESS].a) * chassis_speed_max; // 系数待测
    chassis_cmd_send.vy = (video_data[TEMP].key[KEY_PRESS].w - video_data[TEMP].key[KEY_PRESS].s) * chassis_speed_max;
    if (video_data[TEMP].key[KEY_PRESS].shift) // 小陀螺
        chassis_cmd_send.wz = 24000;           // 待测
    else
        chassis_cmd_send.wz = 100.0f * (gimbal_fetch_data.yaw_angle - YAW_CHASSIS_ALIGN_ECD) * ECD_ANGLE_COEF_DJI; // 旋转方向

    if (video_data[TEMP].key_data.right_button_down && vision_recv_data->is_tracking) // 视觉
    {
        gimbal_cmd_send.yaw = vision_recv_data->yaw;
        gimbal_cmd_send.pitch = vision_recv_data->pitch;
    }
    else
    {
        gimbal_cmd_send.yaw += (float)video_data[TEMP].key_data.mouse_x / 660 * 10; // 系数待测
        gimbal_cmd_send.pitch += (float)video_data[TEMP].key_data.mouse_y / 660 * 10;
    }

    if (gimbal_cmd_send.yaw > 180)
        gimbal_cmd_send.yaw -= 360;
    else if (gimbal_cmd_send.yaw < -180)
        gimbal_cmd_send.yaw += 360;

    if (gimbal_cmd_send.pitch > PITCH_MAX) // 软件限位
        gimbal_cmd_send.pitch = PITCH_MAX;
    else if (gimbal_cmd_send.pitch < PITCH_MIN)
        gimbal_cmd_send.pitch = PITCH_MIN;

    // F键控制底盘模式
    switch (video_data[TEMP].key_count[KEY_PRESS][Key_F] % 2)
    {
    case 1:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW; // normal
        break;
    default:
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE; // stop
        break;
    }

    // Q键控制拨盘模式
    switch (video_data[TEMP].key_count[KEY_PRESS][Key_Q] % 3)
    {
    case 1:
        chassis_cmd_send.loader_mode = LOAD_SPEED;
        chassis_cmd_send.loader_ui_mode = LOAD_SPEED;
        break;
    case 2:
        chassis_cmd_send.loader_mode = LOAD_BUFF;
        chassis_cmd_send.loader_ui_mode = LOAD_BUFF;
        break;
    default:
        chassis_cmd_send.loader_mode = LOAD_SINGLE;
        chassis_cmd_send.loader_ui_mode = LOAD_SINGLE;
        break;
    }

    if (video_data[TEMP].key[KEY_PRESS].z) // 拨盘反转
        chassis_cmd_send.loader_mode = LOAD_REVERSE;
    else if (!video_data[TEMP].key_data.left_button_down)
        chassis_cmd_send.loader_mode = LOAD_STOP;

    // E键切换摩擦轮速度
    switch (video_data[TEMP].key_count[KEY_PRESS][Key_E] % 4)
    {
    case 1:
        shoot_cmd_send.friction_mode = FRICTION_NORMAL;
        chassis_cmd_send.friction_mode = FRICTION_NORMAL;
        break;
    case 2:
        shoot_cmd_send.friction_mode = FRICTION_LOW;
        chassis_cmd_send.friction_mode = FRICTION_LOW;
        break;
    case 3:
        shoot_cmd_send.friction_mode = FRICTION_HIGH;
        chassis_cmd_send.friction_mode = FRICTION_HIGH;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_STOP;
        chassis_cmd_send.friction_mode = FRICTION_STOP;
        break;
    }

    // B键切换图传模式，normal，adaptive
    switch (video_data[TEMP].key_count[KEY_PRESS][Key_B] % 2)
    {
    case 1:
        shoot_cmd_send.video_mode = VIDEO_ADAPTIVE;
        chassis_cmd_send.video_mode = VIDEO_ADAPTIVE;
        break;
    default:
        shoot_cmd_send.video_mode = VIDEO_NORMAL;
        chassis_cmd_send.video_mode = VIDEO_NORMAL;
        break;
    }

    // G键切换开镜模式
    switch (video_data[TEMP].key_count[KEY_PRESS][Key_G] % 2)
    {
    case 1:
        shoot_cmd_send.lens_mode = LENS_ON;
        break;
    default:
        shoot_cmd_send.lens_mode = LENS_OFF;
        break;
    }

    if (video_data[TEMP].key[KEY_PRESS].x) // 刷新ui
        chassis_cmd_send.ui_mode = UI_REFRESH;
    else
        chassis_cmd_send.ui_mode = UI_KEEP;
}

/**
 * @brief 输入为视觉调试时模式和控制量设置,测试用
 *
 */
static void VisionControlSet()
{
    // 底盘
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    // 云台
    if (vision_recv_data->is_tracking)
    {
        gimbal_cmd_send.yaw = vision_recv_data->yaw;
        gimbal_cmd_send.pitch = vision_recv_data->pitch;
    }
    else
    {
        gimbal_cmd_send.yaw += 0.005f * (float)rc_data[TEMP].rc.rocker_l_;
        gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
    }

    // 发射参数
    chassis_cmd_send.loader_mode = LOAD_SPEED;
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[下],停止发射
    {
        shoot_cmd_send.friction_mode = FRICTION_STOP;
        chassis_cmd_send.loader_mode = LOAD_STOP;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[中],摩擦轮启动
    {
        shoot_cmd_send.friction_mode = FRICTION_NORMAL;
        chassis_cmd_send.loader_mode = LOAD_STOP;
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[上],发射
    {
        shoot_cmd_send.friction_mode = FRICTION_NORMAL;
        chassis_cmd_send.loader_mode = LOAD_STOP;
    }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    // if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    // {
    //     robot_state = ROBOT_STOP;
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    //     chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    //     shoot_cmd_send.shoot_mode = SHOOT_OFF;
    //     shoot_cmd_send.friction_mode = FRICTION_OFF;
    //     shoot_cmd_send.load_mode = LOAD_STOP;
    // }
    // // 遥控器右侧开关为[上],恢复正常运行
    // if (switch_is_up(rc_data[TEMP].rc.switch_right))
    // {
    //     robot_state = ROBOT_READY;
    //     shoot_cmd_send.shoot_mode = SHOOT_ON;
    //     LOGINFO("[CMD] reinstate, robot ready");
    // }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);

    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // 遥控器链路
    if (is_remote_online)
    {
        if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 遥控器右侧开关状态为[下],遥控器控制
            RemoteControlSet();
        else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 遥控器右侧开关状态为[上],键盘控制
            RemoteMouseKeySet();
        else if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 遥控器右侧开关状态为[上],视觉模式
            VisionControlSet();
    }
    else // 图传链路
    {
        VideoMouseKeySet();
    }

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置

    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);

    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    VisionSend(&vision_send_data);
}
