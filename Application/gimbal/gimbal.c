/*
*****Gimbal_task云台任务*****
* 云台电机为6020，ID = 1
* 云台电机为motor_top[6] CAN1控制
* 遥控器控制：左拨杆左右
* 键鼠控制：鼠标左右滑动
*/

#include "gimbal.h"
#include "ins_task.h"
#include "dji_motor.h"
#include "lk_motor.h"
#include "robot_def.h"
#include "message_center.h"

static attitude_t *gimbal_ins; // 云台IMU数据
static DJIMotor_Instance *yaw_motor, *pitch_motor;
static LKMotor_Instance *lk_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static float yaw_cur;
static float yaw_angle_out;
static float yaw_speed_out;

void gimbal_init()
{
    gimbal_ins = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源

    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.4,
                .Ki = 0.415,
                .Kd = 0.016,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ChangingIntegrationRate,
                .DeadBand = 0.1,
                .CoefB = 0.6,
                .CoefA = 0.4,
                .Derivative_LPF_RC = 0.025,
                .IntegralLimit = 160,
                .MaxOut = 800,
            },
            .speed_PID = {
                .Kp = 10000,
                .Ki = 2400,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate,
                .CoefB = 0.3,
                .CoefA = 0.2,
                .IntegralLimit = 10000,
                .MaxOut = 25000,
            },
            .other_angle_feedback_ptr = &gimbal_ins->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_ins->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = GM6020};

    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 5, // 10
                .Ki = 2,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate,
                .CoefB = 0.6,
                .CoefA = 0.4,
                .IntegralLimit = 200,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 100, // 50
                .Ki = 10,  // 350
                .Kd = 0,  // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 16384,
            },
            .other_angle_feedback_ptr = &gimbal_ins->Roll,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimbal_ins->Gyro[1]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

    // // yaw
    // Motor_Init_Config_s lk_config = {
    //     .can_init_config = {
    //         .can_handle = &hcan1,
    //         .tx_id = 1,
    //     },
    //     .controller_param_init_config = {
    //         .angle_PID = {
    //             .Kp = 0.6,
    //             .Ki = 0.01,
    //             .Kd = 0.0135,
    //             .Improve = PID_Integral_Limit | PID_OutputFilter,
    //             .IntegralLimit = 10,
    //             .MaxOut = 50,
    //         },
    //         .speed_PID = {
    //             .Kp = 10,
    //             .Ki = 0,
    //             .Kd = 0,
    //             .Improve = PID_Integral_Limit | PID_OutputFilter,
    //             .IntegralLimit = 800,
    //             .MaxOut = 2000,
    //         },
    //         .current_PID = {
    //             .Kp = 280,
    //             .Ki = 200,
    //             .Kd = 0,
    //             .Improve = PID_Integral_Limit | PID_OutputFilter,
    //             .IntegralLimit = 800,
    //             .MaxOut = 20000,
    //         },
    //         .other_angle_feedback_ptr = &gimbal_ins->Roll,
    //         // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
    //         .other_speed_feedback_ptr = (&gimbal_ins->Gyro[1]),
    //     },
    //     .controller_setting_init_config = {
    //         .angle_feedback_source = MOTOR_FEED,
    //         .speed_feedback_source = MOTOR_FEED,
    //         .outer_loop_type = SPEED_LOOP,
    //         .close_loop_type = SPEED_LOOP,
    //         .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
    //         .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    //     },
    //     .motor_work_type = LK_SINGLE_MOTOR_TORQUE,
    //     .motor_type = LK9025,
    // };

    // lk_motor = LKMotorInit(&lk_config);
}

// 云台运动task
void gimbal_task()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    if (gimbal_cmd_recv.robot_status == ROBOT_STOP)
    {
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
    }
    else
    {
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
    }

    DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
    DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
    DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
    DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);

    DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
    DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);

    yaw_cur = yaw_motor->measure.real_current;
    yaw_angle_out = yaw_motor->motor_controller.angle_PID.Output;
    yaw_speed_out = yaw_motor->motor_controller.speed_PID.Output;
    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_ins = *gimbal_ins;
    gimbal_feedback_data.yaw_angle = yaw_motor->measure.ecd;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);

    // LKMotorSetRef(lk_motor, 6000);
    // LKMotorStop(lk_motor);
}