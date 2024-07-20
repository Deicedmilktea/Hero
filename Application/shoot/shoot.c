/*
**********Shoot_task射击任务**********
包含对三个摩擦轮以及开镜电机的控制
摩擦轮分别为3508, ID = 1(left) ID = 2(right) ID = 3(up) ID = 4(lens_up开关镜), ID = 5(lens_down图传上下), CAN2 控制
遥控器控制：左边拨杆，拨到中间启动摩擦轮(lr)，最上面发射(up) (从上到下分别为132)
键鼠控制：默认开启摩擦轮(lr)，左键发射(up)
*/

#include "shoot.h"
#include "message_center.h"
#include "robot_def.h"
#include "bsp_dwt.h"
#include "dji_motor.h"

static DJIMotor_Instance *friction_l, *friction_r, *friction_up, *lens, *video;

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

static int16_t lens_init_angle = 0;
static int16_t video_init_angle = 0;

void shoot_init()
{
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 5, // 20
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 1, // 0.7
                .Ki = 0, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 1; // 左摩擦轮
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 2; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 3; // 上摩擦轮
    friction_config.motor_type = M2006;
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_up = DJIMotorInit(&friction_config);

    Motor_Init_Config_s lens_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20, // 20
                .Ki = 0,  // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 2000,
                .MaxOut = 2000,
            },
            .angle_PID = {
                .Kp = 20, // 20
                .Ki = 0,  // 1
                .Kd = 1,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 8000,
            },
            .current_PID = {
                .Kp = 1, // 0.7
                .Ki = 0, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 4000,
                .MaxOut = 8000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006};

    lens_config.can_init_config.tx_id = 4; // 开镜电机
    lens = DJIMotorInit(&lens_config);

    lens_config.can_init_config.tx_id = 5; // 图传上下
    video = DJIMotorInit(&lens_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

void shoot_task()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    if (shoot_cmd_recv.robot_status == ROBOT_STOP)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(friction_up);
        DJIMotorStop(lens);
        DJIMotorStop(video);
    }
    else
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(friction_up);
        // DJIMotorEnable(lens);
        // DJIMotorEnable(video);
        // DJIMotorStop(friction_l);
        // DJIMotorStop(friction_r);
        // DJIMotorStop(friction_up);
        DJIMotorStop(lens);
        DJIMotorStop(video);
    }

    switch (shoot_cmd_recv.friction_mode)
    {
    case FRICTION_STOP:
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
        DJIMotorSetRef(friction_up, 0);
        break;
    case FRICTION_NORMAL:
        DJIMotorSetRef(friction_l, FRICTION_SPEED_NORMAL);
        DJIMotorSetRef(friction_r, FRICTION_SPEED_NORMAL);
        DJIMotorSetRef(friction_up, FRICTION_SPEED_NORMAL);
        break;
    case FRICTION_LOW:
        DJIMotorSetRef(friction_l, FRICTION_SPEED_LOW);
        DJIMotorSetRef(friction_r, FRICTION_SPEED_LOW);
        DJIMotorSetRef(friction_up, FRICTION_SPEED_LOW);
        break;
    case FRICTION_HIGH:
        DJIMotorSetRef(friction_l, FRICTION_SPEED_HIGH);
        DJIMotorSetRef(friction_r, FRICTION_SPEED_HIGH);
        DJIMotorSetRef(friction_up, FRICTION_SPEED_HIGH);
        break;
    }

    if (shoot_cmd_recv.lens_judge_mode == LENS_MODE_SPEED)
    {
        DJIMotorSetRef(lens, -LENS_PREPARE_SPEED);
        DJIMotorSetRef(video, -LENS_PREPARE_SPEED);

        lens_init_angle = lens->measure.total_angle;
        video_init_angle = video->measure.total_angle;
    }
    else
    {
        DJIMotorOuterLoop(lens, ANGLE_LOOP);
        DJIMotorOuterLoop(video, ANGLE_LOOP);
        lens->motor_settings.close_loop_type = ANGLE_LOOP | CURRENT_LOOP;
        video->motor_settings.close_loop_type = ANGLE_LOOP | CURRENT_LOOP;
        if (shoot_cmd_recv.lens_mode == LENS_ON)
            DJIMotorSetRef(lens, lens_init_angle + LENS_MOVE_ANGLE);
        else
            DJIMotorSetRef(lens, lens_init_angle);

        if (shoot_cmd_recv.video_mode == VIDEO_ADAPTIVE)
            DJIMotorSetRef(video, video_init_angle + VIDEO_MOVE_ANGLE);
        else
            DJIMotorSetRef(video, video_init_angle);
    }

    shoot_feedback_data.lens_current = lens->measure.real_current;
    shoot_feedback_data.video_current = video->measure.real_current;

    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}