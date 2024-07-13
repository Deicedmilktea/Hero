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
#include "ins_task.h"
#include "robot_def.h"

extern Robot_Control_Data_s robot_ctrl;
extern uint8_t is_remote_online;    // 遥控器在线标志位
extern uint8_t supercap_flag;       // 超级电容标志位
extern referee_hero_t referee_hero; // 裁判系统英雄信息

attitude_t *chassis_ins; // 底盘IMU数据

static DJIMotor_Instance *motor_lf, *motor_rf, *motor_lb, *motor_rb;                  // left right forward back
static float chassis_vx, chassis_vy, chassis_wz;                                      // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;                                              // 底盘速度解算后的临时输出,待进行限幅
static int16_t chassis_speed_max = 0;                                                 // 底盘最大速度
static int16_t chassis_wz_max = 0;                                                    // 底盘最大转动速度
static int16_t key_x_fast, key_y_fast, key_x_slow, key_y_slow, key_Wz_acw, key_Wz_cw; // 键盘控制变量

static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

const int16_t chassis_speed_max_buff[] = {
    500,  // 1
    550,  // 2
    600,  // 3
    650,  // 4
    750,  // 5
    850,  // 6
    900,  // 7
    1000, // 8
    1100, // 9
    1200  // 10
};

static void level_judge();
static void rc_mode_choose();
static void video_mode_choose();
static void chassis_mode_follow();
static void chassis_mode_stop();
static void MecanumCalculate();
static void LimitChassisOutput(float Chassis_pidout_target_limit);
static void key_control();
static void detel_calc(float *angle);
static void read_keyboard();

void chassis_init()
{
    chassis_ins = INS_Init();                     // 底盘IMU初始化
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

static void level_judge()
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

static void rc_mode_choose()
{
    read_keyboard(); // 读取键盘数据

    // 右拨杆下，遥控操作
    if (switch_is_down(robot_ctrl.key_ctrl.rc.switch_right))
    {
        chassis_mode_follow(); // 正常模式，便于检录小陀螺展示
    }

    // 右拨杆中，键鼠操作
    else if (switch_is_mid(robot_ctrl.key_ctrl.rc.switch_right))
    {
        read_keyboard(); // 读取键盘数据
        key_control();

        if (ui_data.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
        {
            chassis_mode_follow();
            LimitChassisOutput(4 * chassis_speed_max);
        }
        else
        {
            chassis_mode_stop();
        }
    }

    // 右拨杆上，校正yaw
    else
    {
        chassis_mode_stop();
    }
}

static void video_mode_choose()
{
    read_keyboard(); // 读取键盘数据
    key_control();

    if (ui_data.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        chassis_mode_follow();
        LimitChassisOutput(4 * chassis_speed_max);
    }
    else
    {
        chassis_mode_stop();
    }
}

static void chassis_mode_follow()
{
    DJIMotorEnable(motor_lf);
    DJIMotorEnable(motor_rf);
    DJIMotorEnable(motor_lb);
    DJIMotorEnable(motor_rb);

    int16_t temp_vx = robot_ctrl.key_ctrl.rc.rocker_r_ / 660.0f * chassis_speed_max + key_x_fast - key_x_slow; // left and right
    int16_t temp_vy = robot_ctrl.key_ctrl.rc.rocker_r1 / 660.0f * chassis_speed_max + key_y_fast - key_y_slow; // front and back

    // 保证切换回这个模式的时候，头在初始方向上，速度移动最快，方便逃跑(●'◡'●)
    float relative_yaw = (robot_ctrl.top_yaw - INIT_YAW) / 8191.0f * 360;

    // 便于小陀螺操作
    if (key_Wz_acw)
    {
        chassis_wz = key_Wz_acw; // rotate
    }

    else if (robot_ctrl.key_ctrl.rc.dial)
    {
        chassis_wz = robot_ctrl.key_ctrl.rc.dial / 660.0f * chassis_wz_max; // rotate
    }

    else
    {
        // 消除静态旋转
        if (relative_yaw > -5 && relative_yaw < 5)
        {
            chassis_wz = 0;
        }

        else
        {
            detel_calc(&relative_yaw);
            chassis_wz = -relative_yaw * FOLLOW_WEIGHT;

            if (chassis_wz > chassis_wz_max)
                chassis_wz = chassis_wz_max;
            if (chassis_wz < -chassis_wz_max)
                chassis_wz = -chassis_wz_max;
        }
    }

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    float cos_theta = arm_cos_f32(relative_yaw * DEGREE_2_RAD);
    float sin_theta = arm_sin_f32(relative_yaw * DEGREE_2_RAD);
    chassis_vx = temp_vx * cos_theta - temp_vy * sin_theta;
    chassis_vy = temp_vx * sin_theta + temp_vy * cos_theta;

    MecanumCalculate();
}

static void chassis_mode_stop()
{
    DJIMotorStop(motor_lf);
    DJIMotorStop(motor_rf);
    DJIMotorStop(motor_lb);
    DJIMotorStop(motor_rb);
}

/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    vt_lf = -chassis_vx - chassis_vy - chassis_wz * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy - chassis_wz * RF_CENTER;
    vt_lb = chassis_vx - chassis_vy - chassis_wz * LB_CENTER;
    vt_rb = chassis_vx + chassis_vy - chassis_wz * RB_CENTER;
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput(float Chassis_pidout_target_limit)
{
    float Watch_Power = referee_hero.chassis_power;
    float Watch_Buffer = referee_hero.buffer_energy;              // 限制值，功率值，缓冲能量值，初始值是1，0，0
    float Scaling1 = 0, Scaling2 = 0, Scaling3 = 0, Scaling4 = 0; // 比例
    float Klimit = 1;                                             // 限制值
    float Plimit = 0;                                             // 约束比例
    float Chassis_pidout_max = 52428;                             // 16384 * 4 *0.8，取了4个3508电机最大电流的一个保守值

    // if (Watch_Power > 600)
    // {
    //     for (uint8_t i = 0; i < 4; i++)
    //     {
    //         chassis_motor[i].target_speed = Motor_Speed_limiting(chassis_motor[i].target_speed, 4096); // 限制最大速度 ;//5*4*24;先以最大电流启动，后平滑改变
    //     }
    // }
    // else
    // {
    float Chassis_pidout = (fabs(vt_lf - motor_lf->measure.speed_aps) +
                            fabs(vt_rf - motor_rf->measure.speed_aps) +
                            fabs(vt_lb - motor_lb->measure.speed_aps) +
                            fabs(vt_rb - motor_rb->measure.speed_aps)); // fabs是求绝对值，这里获取了4个轮子的差值求和

    /*期望滞后占比环，增益个体加速度*/
    if (Chassis_pidout)
    {
        Scaling1 = (vt_lf - motor_lf->measure.speed_aps) / Chassis_pidout;
        Scaling2 = (vt_rf - motor_rf->measure.speed_aps) / Chassis_pidout;
        Scaling3 = (vt_lb - motor_lb->measure.speed_aps) / Chassis_pidout;
        Scaling4 = (vt_rb - motor_rb->measure.speed_aps) / Chassis_pidout; // 求比例，4个scaling求和为1
    }
    else
    {
        Scaling1 = 0.25, Scaling2 = 0.25, Scaling3 = 0.25, Scaling4 = 0.25;
    }

    /*功率满输出占比环，车总增益加速度*/
    Klimit = Chassis_pidout / Chassis_pidout_target_limit;

    if (Klimit > 1)
        Klimit = 1;
    else if (Klimit < -1)
        Klimit = -1; // 限制绝对值不能超过1，也就是Chassis_pidout一定要小于某个速度值，不能超调

    /*缓冲能量占比环，总体约束*/
    if (!supercap_flag)
    {
        if (Watch_Buffer <= 60 && Watch_Buffer >= 40)
            Plimit = 0.95; // 近似于以一个线性来约束比例（为了保守可以调低Plimit，但会影响响应速度）
        else if (Watch_Buffer < 40 && Watch_Buffer >= 35)
            Plimit = 0.75;
        else if (Watch_Buffer < 35 && Watch_Buffer >= 30)
            Plimit = 0.5;
        else if (Watch_Buffer < 30 && Watch_Buffer >= 20)
            Plimit = 0.25;
        else if (Watch_Buffer < 20 && Watch_Buffer >= 10)
            Plimit = 0.125;
        else if (Watch_Buffer < 10 && Watch_Buffer > 0)
            Plimit = 0.05;
        else
            Plimit = 1;
    }

    else
        Plimit = 1;

    vt_lf = Scaling1 * (Chassis_pidout_max * Klimit) * Plimit; // 输出值
    vt_rf = Scaling2 * (Chassis_pidout_max * Klimit) * Plimit;
    vt_lb = Scaling3 * (Chassis_pidout_max * Klimit) * Plimit;
    vt_rb = Scaling4 * (Chassis_pidout_max * Klimit) * Plimit;

    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
    // }
}

static void key_control()
{
    if (robot_ctrl.key_ctrl.key[KEY_PRESS].d)
        key_x_fast += KEY_START_OFFSET;
    else
        key_x_fast -= KEY_STOP_OFFSET;

    if (robot_ctrl.key_ctrl.key[KEY_PRESS].a)
        key_x_slow += KEY_START_OFFSET;
    else
        key_x_slow -= KEY_STOP_OFFSET;

    if (robot_ctrl.key_ctrl.key[KEY_PRESS].w)
        key_y_fast += KEY_START_OFFSET;
    else
        key_y_fast -= KEY_STOP_OFFSET;

    if (robot_ctrl.key_ctrl.key[KEY_PRESS].s)
        key_y_slow += KEY_START_OFFSET;
    else
        key_y_slow -= KEY_STOP_OFFSET;

    // 正转
    if (robot_ctrl.key_ctrl.key[KEY_PRESS].shift)
        key_Wz_acw += KEY_START_OFFSET;
    else
        key_Wz_acw -= KEY_STOP_OFFSET;

    if (key_x_fast > chassis_speed_max)
        key_x_fast = chassis_speed_max;
    if (key_x_fast < 0)
        key_x_fast = 0;
    if (key_x_slow > chassis_speed_max)
        key_x_slow = chassis_speed_max;
    if (key_x_slow < 0)
        key_x_slow = 0;
    if (key_y_fast > chassis_speed_max)
        key_y_fast = chassis_speed_max;
    if (key_y_fast < 0)
        key_y_fast = 0;
    if (key_y_slow > chassis_speed_max)
        key_y_slow = chassis_speed_max;
    if (key_y_slow < 0)
        key_y_slow = 0;
    if (key_Wz_acw > chassis_wz_max)
        key_Wz_acw = chassis_wz_max;
    if (key_Wz_acw < 0)
        key_Wz_acw = 0;
}

static void detel_calc(float *angle)
{
    // 如果角度大于180度，则减去360度
    if (*angle > 180)
    {
        *angle -= 360;
    }

    // 如果角度小于-180度，则加上360度
    else if (*angle < -180)
    {
        *angle += 360;
    }
}

static void read_keyboard()
{
    // F键控制底盘模式
    switch (robot_ctrl.key_ctrl.key_count[KEY_PRESS][Key_F] % 2)
    {
    case 1:
        ui_data.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW; // normal
        break;
    default:
        ui_data.chassis_mode = CHASSIS_ZERO_FORCE; // stop
        break;
    }

    // Q键控制拨盘模式
    switch (robot_ctrl.key_ctrl.key_count[KEY_PRESS][Key_Q] % 2)
    {
    case 1:
        ui_data.shoot_mode = SHOOT_BUFF;
        break;
    default:
        ui_data.shoot_mode = SHOOT_NORMAL;
        break;
    }

    // // 超电开关
    // switch (SupercapRxData.state)
    // {
    // case 3:
    //     supercap_flag = 1;
    //     ui_data.supcap_mode = SUPCAP_ON;
    //     break;
    // default: // receive 0
    //     supercap_flag = 0;
    //     ui_data.supcap_mode = SUPCAP_OFF;
    //     break;
    // }

    // E键切换摩擦轮速度，012分别为low，normal，high
    switch (robot_ctrl.friction_mode)
    {
    case FRICTION_NORMAL:
        ui_data.friction_mode = FRICTION_NORMAL;
        break;
    case FRICTION_LOW:
        ui_data.friction_mode = FRICTION_LOW;
        break;
    case FRICTION_HIGH:
        ui_data.friction_mode = FRICTION_HIGH;
        break;
    case FRICTION_STOP:
        ui_data.friction_mode = FRICTION_STOP;
        break;
    }

    // B键切换图传模式，normal，adaptive
    switch (robot_ctrl.video_mode)
    {
    case 1:
        ui_data.video_mode = VIDEO_ADAPTIVE;
        break;
    default:
        ui_data.video_mode = VIDEO_NORMAL;
        break;
    }
}