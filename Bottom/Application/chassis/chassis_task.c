/**
 * @file chassis_task.c
 * @author Reeve Ni (reeveni666@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-07-10
 * @copyright Copyright (c) 2024
 *
 */

#include "chassis_task.h"
#include "dji_motor.h"
#include "controller.h"
#include "referee_task.h"
#include "remote_control.h"
#include "video_control.h"

extern RC_ctrl_t rc_ctrl[2];        // 遥控器数据
extern Video_ctrl_t video_ctrl[2];  // 图传数据
extern uint8_t is_remote_online;    // 遥控器在线标志位
extern uint8_t supercap_flag;       // 超级电容标志位
extern referee_hero_t referee_hero; // 裁判系统英雄信息

static DJIMotor_Instance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
static referee_info_t *referee_data;                                 // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data;                           // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static int16_t chassis_speed_max = 0; // 底盘最大速度
static int16_t chassis_wz_max = 0;    // 底盘最大转动速度
const int16_t chassis_speed_max_buff[] = {
    5000,  // 1
    5500,  // 2
    6000,  // 3
    6500,  // 4
    7500,  // 5
    8500,  // 6
    9000,  // 7
    10000, // 8
    11000, // 9
    12000  // 10
};

void chassis_init()
{
    referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI

    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };

    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_motor_config);
}

void chassis_task()
{
    level_judge(); // 等级判断，获取最大速度

    if (is_remote_online) // 遥控器链路
        rc_mode_choose();

    else // 图传链路
        video_mode_choose();
}

void level_judge()
{
    if (referee_hero.robot_level != 0)
    {
        if (referee_hero.robot_level >= 1 && referee_hero.robot_level <= 10)
            chassis_speed_max = chassis_speed_max_buff[referee_hero.robot_level - 1];

        if (referee_hero.robot_level > 0 && referee_hero.robot_level < 6)
            chassis_wz_max = CHASSIS_WZ_MAX_1;
        else
            chassis_wz_max = CHASSIS_WZ_MAX_2;
    }

    else
    {
        chassis_speed_max = chassis_speed_max_buff[0];
        chassis_wz_max = CHASSIS_WZ_MAX_1;
    }

    if (supercap_flag)
        chassis_speed_max += 2000;
}