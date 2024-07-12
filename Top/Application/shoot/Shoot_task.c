/*
**********Shoot_task射击任务**********
包含对三个摩擦轮以及开镜电机的控制
摩擦轮分别为3508, ID = 1(left) ID = 2(right) ID = 3(up) ID = 4(lens_up开关镜), ID = 5(lens_down图传上下), CAN2 控制
遥控器控制：左边拨杆，拨到中间启动摩擦轮(lr)，最上面发射(up) (从上到下分别为132)
键鼠控制：默认开启摩擦轮(lr)，左键发射(up)
*/

#include "Shoot_task.h"
#include "remote_control.h"
#include "video_control.h"
#include "ins_task.h"

uint8_t friction_flag = 0; // 摩擦轮转速标志位，012分别为low, normal, high, 默认为normal
uint8_t video_mode;        // 图传模式
uint8_t is_friction_on;    // 摩擦轮是否开启

static int16_t friction_speed    = 0;
static int16_t friction_up_speed = 0;
static uint8_t is_lens_ready     = 0; // 开镜是否到达机械限位

extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl[2];
extern Video_ctrl_t video_ctrl[2];
extern uint8_t is_remote_online;
extern INS_t INS;

static void read_keyboard();                                                              // 读取摩擦轮速度
static void shoot_start_lr();                                                             // 左右摩擦轮开启模式
static void shoot_start_remote();                                                         // 遥控器三摩擦轮开启模式
static void shoot_start_mouse();                                                          // 键鼠三摩擦轮开启模式
static void shoot_stop();                                                                 // 摩擦轮关闭模式
static void lens_judge();                                                                 // 判断开关镜
static void shoot_can2_cmd(uint8_t mode, int16_t v1, int16_t v2, int16_t v3, int16_t v4); // can2发送电流
static void shoot_current_give();                                                         // PID计算速度并发送电流
static void read_keyboard();                                                              // 读取键鼠是否开启摩擦轮
static void lens_prepare();                                                               // 开镜准备
static void video_adaptive();                                                             // 图传自适应调节
static void read_friction_on();                                                           // 摩擦轮是否开启

void Shoot_task(void const *argument)
{
    read_keyboard();
    read_friction_on();
    lens_prepare();
    video_adaptive();
    lens_judge();

    // 遥控器链路
    if (is_remote_online) {
        // 右拨杆中，键鼠控制
        if (switch_is_mid(rc_ctrl[TEMP].rc.switch_right)) {
            // shoot_start_mouse();
        }

        // 右拨杆下，遥控器控制
        else {
            // 遥控器左边拨到上和中，电机启动
            if (switch_is_up(rc_ctrl[TEMP].rc.switch_left) || switch_is_mid(rc_ctrl[TEMP].rc.switch_left)) {
                friction_speed    = FRICTION_SPEED_NORMAL;
                friction_up_speed = -FRICTION_SPEED_NORMAL;
                shoot_start_remote();
            } else {
                shoot_stop();
            }
        }
    }

    // 图传链路
    else {
        // shoot_start_mouse();
    }
}

/**************初始化***************/
void shoot_init()
{
}

// 读取摩擦轮速度
static void read_keyboard()
{
    // 遥控器链路
    if (is_remote_online) {
        // E键切换摩擦轮速度
        switch (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_E] % 4) {
            case 1:
                friction_speed    = FRICTION_SPEED_NORMAL;
                friction_up_speed = FRICTION_UP_SPEED;
                friction_flag     = FRICTION_NORMAL;
                break;
            case 2:
                friction_speed    = FRICTION_SPEED_LOW;
                friction_up_speed = FRICTION_UP_SPEED;
                friction_flag     = FRICTION_LOW;
                break;
            case 3:
                friction_speed    = FRICTION_SPEED_HIGH;
                friction_up_speed = FRICTION_UP_SPEED;
                friction_flag     = FRICTION_HIGH;
                break;
            default:
                friction_speed    = FRICTION_SPEED_STOP;
                friction_up_speed = FRICTION_UP_SPEED;
                friction_flag     = FRICTION_STOP;
                break;
        }
    }

    // 图传链路
    else {
        // E键切换摩擦轮速度
        switch (video_ctrl[TEMP].key_count[KEY_PRESS][Key_E] % 4) {
            case 1:
                friction_speed    = FRICTION_SPEED_NORMAL;
                friction_up_speed = FRICTION_UP_SPEED;
                friction_flag     = FRICTION_NORMAL;
                break;
            case 2:
                friction_speed    = FRICTION_SPEED_LOW;
                friction_up_speed = FRICTION_UP_SPEED;
                friction_flag     = FRICTION_LOW;
                break;
            case 3:
                friction_speed    = FRICTION_SPEED_HIGH;
                friction_up_speed = FRICTION_UP_SPEED;
                friction_flag     = FRICTION_HIGH;
                break;
            default:
                friction_speed    = FRICTION_SPEED_STOP;
                friction_up_speed = FRICTION_UP_SPEED;
                friction_flag     = FRICTION_STOP;
                break;
        }
    }
}

/***************** 判断开关镜 *******************/
static void lens_judge()
{
    // // 遥控器链路
    // if (is_remote_online) {
    //     if (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_G] % 2 == 1)
    //         lens_motor[0].target_angle = lens_motor[0].init_angle + LENS_UP_MOVE_ANGLE;
    //     else
    //         lens_motor[0].target_angle = lens_motor[0].init_angle;

    //     if (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_B] % 2 == 1)
    //         video_mode = VIDEO_ADAPTIVE;
    //     else
    //         video_mode = VIDEO_NORMAL;
    // }

    // // 图传链路
    // else {
    //     if (video_ctrl[TEMP].key_count[KEY_PRESS][Key_G] % 2 == 1)
    //         lens_motor[0].target_angle = lens_motor[0].init_angle + LENS_UP_MOVE_ANGLE;
    //     else
    //         lens_motor[0].target_angle = lens_motor[0].init_angle;

    //     if (video_ctrl[TEMP].key_count[KEY_PRESS][Key_B] % 2 == 1)
    //         video_mode = VIDEO_ADAPTIVE;
    //     else
    //         video_mode = VIDEO_NORMAL;
    // }
}

void lens_prepare()
{
    // if (!is_lens_ready) {
    //     lens_motor[0].target_speed = -1000;
    //     lens_motor[1].target_speed = -1000;

    //     if ((motor_top[3].torque_current < -1500) && (motor_top[4].torque_current < -1500)) {
    //         is_lens_ready            = 1;
    //         lens_motor[0].init_angle = motor_top[3].total_angle;
    //         lens_motor[1].init_angle = motor_top[4].total_angle;
    //     }
    // }
}

void video_adaptive()
{
    // if (is_lens_ready) {
    //     if (video_mode == VIDEO_ADAPTIVE) {
    //         // 调节角度
    //         int16_t adptive_angle      = 36 * (INS.Roll + 6);
    //         lens_motor[1].target_angle = lens_motor[1].init_angle + adptive_angle;
    //     }

    //     else {
    //         lens_motor[1].target_angle = lens_motor[1].init_angle;
    //     }
    // }
}

void read_friction_on()
{
    if (friction_speed)
        is_friction_on = 1;
    else
        is_friction_on = 0;
}