#ifndef SHOOT_H
#define SHOOT_H

#define FRICTION_MAX_SPEED 20000
#define FRICTION_UP_SPEED 2000
#define FRICTION_SPEED_NORMAL 5200
#define FRICTION_SPEED_LOW 5500
#define FRICTION_SPEED_HIGH 5900
#define FRICTION_SPEED_STOP 0
#define FRICTION_UP_SPEED_STOP 0

void shoot_init(); // 初始化
void shoot_task();

#endif