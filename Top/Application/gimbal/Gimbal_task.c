/*
*****Gimbal_task云台任务*****
* 云台电机为6020，ID = 1
* 云台电机为motor_top[6] CAN1控制
* 遥控器控制：左拨杆左右
* 键鼠控制：鼠标左右滑动
*/

#include "Gimbal_task.h"
#include "ins_task.h"
#include "dji_motor.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "stdbool.h"
#include "remote_control.h"
#include "video_control.h"

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotor_Instance *yaw_motor, *pitch_motor;

extern RC_ctrl_t rc_ctrl[2];
extern Video_ctrl_t video_ctrl[2];
extern INS_t INS;
extern bool vision_is_tracking;
extern float vision_yaw;
extern uint8_t is_remote_online; // 遥控器是否在线

static void angle_over_zero(float err); // 角度过零处理
static void detel_calc(float *angle);   // 角度范围限制
static void gimbal_mode_vision();       // 视觉控制
static void gimbal_mode_normal();       // 锁yaw

// 云台运动task
void Gimbal_task()
{
    // 遥控链路
    if (is_remote_online) {
        // 视觉控制
        if (switch_is_up(rc_ctrl[TEMP].rc.switch_right) || rc_ctrl[TEMP].mouse.press_r) // 左拨杆上 || 按住右键
        {
            gimbal_mode_vision();
        }

        // 锁yaw模式
        else // 左拨杆中或下
        {
            gimbal_mode_normal();
        }
    }

    // 图传链路
    else {
        // 视觉控制
        if (video_ctrl[TEMP].key_data.right_button_down) // 按住右键
        {
            gimbal_mode_vision();
        }

        // 锁yaw模式
        else // 左拨杆中或下
        {
            gimbal_mode_normal();
        }
    }
}

void Gimbal_init()
{
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源

    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 8, // 8
                .Ki            = 0,
                .Kd            = 0,
                .DeadBand      = 0.1,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,

                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp            = 50,  // 50
                .Ki            = 200, // 200
                .Kd            = 0,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut        = 20000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};

    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id      = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 10, // 10
                .Ki            = 0,
                .Kd            = 0,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut        = 500,
            },
            .speed_PID = {
                .Kp            = 50,  // 50
                .Ki            = 350, // 350
                .Kd            = 0,   // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut        = 20000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor   = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);
}