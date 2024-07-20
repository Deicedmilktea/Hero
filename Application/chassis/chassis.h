#ifndef CHASSIS_H
#define CHASSIS_H

#define CHASSIS_WZ_MAX_1 5500
#define CHASSIS_WZ_MAX_2 8000
#define INIT_YAW 6910
#define KEY_START_OFFSET 20
#define KEY_STOP_OFFSET 40
#define FOLLOW_WEIGHT 100

void chassis_init();
void chassis_task();

#endif // !CHASSIS_H