/**
 * @file robot_def.h
 * @author Reeve Ni (reeveni666@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-07-12
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "remote_control.h"
#include "ins_task.h"

#define CHASSIS_BOARD

#define VISION_USE_VCP // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

#define RAD_2_DEGREE 57.2957795f    // 180/pi
#define DEGREE_2_RAD 0.01745329252f // pi/180

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 2711  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD 3412      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX 0                 // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN 0                 // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 60    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 19.0f // 拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 6             // 拨盘一圈的装载量
#define SHOOT_DELAY 1000             // 发射后的延迟时间,单位ms
#define TRIGGER_SINGLE_ANGLE 1140    // 单发拨盘转动的角度,19*360/6=1140
#define LENS_THRESHOLD_CURRENT 1500  // 判断lens阈值电流
#define LENS_PREPARE_SPEED 1000      // lens准备阶段速度
#define LENS_MOVE_ANGLE 1950         // lens移动角度
#define VIDEO_MOVE_ANGLE 1400        // video移动角度
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 425                          // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 371                         // 横向轮距(左右平移方向)
#define RADIUS_WHEEL 77                         // 轮子半径
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define CENTER_GIMBAL_OFFSET_X 0                // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0                // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define REDUCTION_RATIO_WHEEL 19.0f             // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

#define GYRO2GIMBAL_DIR_YAW 1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0, // 电流零输入
    CHASSIS_ROTATE,         // 小陀螺模式
    CHASSIS_FOLLOW,         // 不跟随，允许全向平移
} chassis_mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
} gimbal_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_NORMAL = 0,
    SHOOT_BUFF, // 爆破发射
} shoot_mode_e;

typedef enum
{
    FRICTION_NORMAL = 0, // 摩擦轮1档
    FRICTION_LOW,
    FRICTION_HIGH,
    FRICTION_STOP,
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0, // 停止发射
    LOAD_REVERSE,  // 反转
    LOAD_SINGLE,   // 单发
    LOAD_BUFF,     // 爆破
    LOAD_SPEED,    // 速度模式
} loader_mode_e;

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float Supcap_power;
} Supcap_Power_Data_s;

typedef enum
{
    SUPCAP_OFF = 2,
    SUPCAP_ON,
} supcap_mode_e;

typedef enum
{
    VIDEO_NORMAL = 0,
    VIDEO_ADAPTIVE,
} video_mode_e;

typedef enum
{
    LENS_OFF = 0,
    LENS_ON,
} lens_mode_e;

typedef enum
{
    LENS_MODE_SPEED = 0,
    LENS_MODE_ANGLE,
} lens_judge_mode_e;

typedef enum
{
    UI_KEEP = 0,
    UI_REFRESH,
} ui_mode_e;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;
    supcap_mode_e supcap_mode;
    Robot_Status_e robot_status;
    // 拨盘控制
    loader_mode_e loader_mode;
    // UI部分
    ui_mode_e ui_mode;
    loader_mode_e loader_ui_mode;
    video_mode_e video_mode;
    friction_mode_e friction_mode;
    float pitch;
    uint8_t is_tracking; // 视觉是否追踪
} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    friction_mode_e friction_mode;
    lens_judge_mode_e lens_judge_mode; // prepare
    lens_mode_e lens_mode;             // lens
    video_mode_e video_mode;           // video
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
    float chassis_ins_pitch;
    uint8_t robot_level;
} Chassis_Upload_Data_s;

typedef struct
{
    attitude_t gimbal_ins;
    uint16_t yaw_angle;
} Gimbal_Upload_Data_s;

typedef struct
{
    int16_t lens_current;    // 开镜电机电流
    float lens_total_angle;  // 开镜电机角度
    int16_t video_current;   // 图传电机电流
    float video_total_angle; // 图传电机角度
} Shoot_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H