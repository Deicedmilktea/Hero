/**
 * @file chassis_task.c
 * @author Reeve Ni (reeveni666@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-07-10
 * @copyright Copyright (c) 2024
 *
 */

#include "chassis.h"
#include "dji_motor.h"
#include "controller.h"
#include "referee_task.h"
#include "ins_task.h"
#include "robot_def.h"
#include "can_comm.h"
#include "supcap.h"

#define LOADER_SPEED 12000
#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

static attitude_t *chassis_ins;                     // 底盘IMU数据
static CANComm_Instance *chasiss_can_comm;          // 双板通信CAN comm
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static DJIMotor_Instance *motor_lf, *motor_rf, *motor_lb, *motor_rb, *loader; // left right forward back
static float chassis_vx, chassis_vy, chassis_wz;                              // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;                                      // 底盘速度解算后的临时输出,待进行限幅
static int16_t chassis_wz_buff[2] = {24000, 36000};                           // 旋转速度

static Supcap_Instance *cap; // 超级电容

static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;
float loadercur = 0;
float loaderang = 0;
float now_time = 0;

static void MecanumCalculate();
static void LimitChassisOutput();
static void loader_task();

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

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 5,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 100, // 10
                .Ki = 0,
                .Kd = 3,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 10000,
            },
            .speed_PID = {
                .Kp = 10,  // 10
                .Ki = 0.1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 10000,
            },
            .current_PID = {
                .Kp = 1, // 0.7
                .Ki = 0, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 10000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M3508 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    Supcap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x302, // 超级电容默认接收id
            .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = Supcap_init(&cap_conf); // 超级电容初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
}

void chassis_task()
{
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);

    ui_data.ui_mode = chassis_cmd_recv.ui_mode;
    ui_data.chassis_mode = chassis_cmd_recv.chassis_mode;
    ui_data.supcap_mode = chassis_cmd_recv.supcap_mode;
    ui_data.loader_mode = chassis_cmd_recv.loader_mode;
    ui_data.video_mode = chassis_cmd_recv.video_mode;
    ui_data.friction_mode = chassis_cmd_recv.friction_mode;
    ui_data.pitch = chassis_cmd_recv.pitch;
    ui_data.is_tracking = chassis_cmd_recv.is_tracking;
    ui_data.Supcap_last_Power_Data.Supcap_power = cap->cap_rx_data.voltage;

    // chassis
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE || chassis_cmd_recv.robot_status == ROBOT_STOP)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    if (chassis_cmd_recv.robot_status == ROBOT_STOP)
        DJIMotorStop(loader);
    else
        DJIMotorEnable(loader);

    float cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    float sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_ROTATE:
        chassis_wz = (referee_data->GameRobotState.robot_level < 6) ? chassis_wz_buff[0] : chassis_wz_buff[1];
        break;
    default:
        chassis_wz = chassis_cmd_recv.wz = -1.5f * chassis_cmd_recv.offset_angle * abs(chassis_cmd_recv.offset_angle);
        break;
    }

    MecanumCalculate();   // 计算底盘速度
    LimitChassisOutput(); // 限制底盘输出

    // loadercur = loader->measure.real_current;
    // loaderang = loader->measure.total_angle;
    // now_time = DWT_GetTimeline_ms();
    loader_task();

    if (chassis_cmd_recv.supcap_mode == SUPCAP_ON)
        SupcapSetData(referee_data->PowerHeatData.buffer_energy, referee_data->PowerHeatData.chassis_power, SUPCAP_ON); // 超级电容数据设置
    else
        SupcapSetData(referee_data->PowerHeatData.buffer_energy, referee_data->PowerHeatData.chassis_power, SUPCAP_OFF); // 超级电容数据设置
    SupcapSend();                                                                                                        // 超级电容数据发送

    chassis_feedback_data.chassis_ins_pitch = chassis_ins->Roll;
    chassis_feedback_data.robot_level = referee_data->GameRobotState.robot_level;
    CANCommSend(chasiss_can_comm, (uint8_t *)&chassis_feedback_data);
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
static void LimitChassisOutput()
{
    float Watch_Buffer = referee_data->PowerHeatData.buffer_energy; // 获取裁判系统的电量数据
    float Plimit;                                                   // 限制比例

    if (chassis_cmd_recv.supcap_mode == SUPCAP_ON)
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

    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf * Plimit);
    DJIMotorSetRef(motor_rf, vt_rf * Plimit);
    DJIMotorSetRef(motor_lb, vt_lb * Plimit);
    DJIMotorSetRef(motor_rb, vt_rb * Plimit);
}

/**
 * @brief loader任务处理
 *
 */
static void loader_task()
{
    if (hibernate_time + dead_time < DWT_GetTimeline_ms()) // 检测是否在休眠状态
    {
        // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
        switch (chassis_cmd_recv.loader_mode)
        {
        // 停止拨盘
        case LOAD_STOP:
            loader->motor_settings.close_loop_type = SPEED_LOOP | CURRENT_LOOP;
            DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
            DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
            break;
        // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
        case LOAD_SINGLE:
            loader->motor_settings.close_loop_type = ANGLE_LOOP | CURRENT_LOOP;
            DJIMotorOuterLoop(loader, ANGLE_LOOP);                                      // 切换到角度环
            DJIMotorSetRef(loader, loader->measure.total_angle - TRIGGER_SINGLE_ANGLE); // 控制量增加一发弹丸的角度
            hibernate_time = DWT_GetTimeline_ms();                                      // 记录触发指令的时间
            dead_time = 1000;                                                           // 完成1发弹丸发射的时间
            break;
        case LOAD_BUFF:
            loader->motor_settings.close_loop_type = ANGLE_LOOP | CURRENT_LOOP;
            DJIMotorOuterLoop(loader, ANGLE_LOOP); // 切换到角度环
            DJIMotorSetRef(loader, loader->measure.total_angle - TRIGGER_SINGLE_ANGLE);
            hibernate_time = DWT_GetTimeline_ms();
            dead_time = 100;
        case LOAD_SPEED:
            loader->motor_settings.close_loop_type = SPEED_LOOP | CURRENT_LOOP;
            DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
            DJIMotorSetRef(loader, -LOADER_SPEED); // 正转
            break;
        case LOAD_REVERSE:
            loader->motor_settings.close_loop_type = SPEED_LOOP | CURRENT_LOOP;
            DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
            DJIMotorSetRef(loader, LOADER_SPEED);  // 反转
            break;
        }
    }
}