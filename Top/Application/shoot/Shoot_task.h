#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#define FRICTION_MAX_SPEED     20000
#define FRICTION_UP_SPEED      2000
#define FRICTION_SPEED_NORMAL  5200
#define FRICTION_SPEED_LOW     5500
#define FRICTION_SPEED_HIGH    5900
#define FRICTION_SPEED_STOP    0
#define FRICTION_UP_SPEED_STOP 0

#define LENS_UP_MOVE_ANGLE     1950
#define LENS_DOWN_MOVE_ANGLE   1400

typedef enum {
    FRICTION_NORMAL, // 摩擦轮1档
    FRICTION_LOW,
    FRICTION_HIGH,
    FRICTION_STOP,
} friction_mode_e;

typedef enum {
    VIDEO_NORMAL = 0,
    VIDEO_ADAPTIVE,
} video_mode_e;

void shoot_init(); // 初始化
void Shoot_task();

#endif