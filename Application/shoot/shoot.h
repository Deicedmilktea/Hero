#ifndef SHOOT_H
#define SHOOT_H

#define FRICTION_MAX_SPEED 20000
#define FRICTION_UP_SPEED 2000
#define FRICTION_SPEED_1 28800
#define FRICTION_SPEED_2 29400
#define FRICTION_SPEED_3 30000
#define FRICTION_SPEED_4 30600
#define FRICTION_SPEED_STOP 0
#define FRICTION_UP_SPEED_STOP 0

void shoot_init(); // 初始化
void shoot_task();

#endif