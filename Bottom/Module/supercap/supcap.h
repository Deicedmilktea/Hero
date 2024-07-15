#ifndef SUPCAP_H
#define SUPCAP_H

#include "bsp_can.h"

#pragma pack(1)
typedef struct
{
    uint16_t vol;     // 电压
    uint16_t current; // 电流
    uint16_t power;   // 功率
} Supcap_Msg_s;
#pragma pack()

/* 超级电容实例 */
typedef struct
{
    CANInstance *can_ins; // CAN实例
    Supcap_Msg_s cap_msg; // 超级电容信息
} Supcap_Instance;

/* 超级电容初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_config;
} Supcap_Init_Config_s;

/**
 * @brief 初始化超级电容
 *
 * @param supercap_config 超级电容初始化配置
 * @return Supcap_Instance* 超级电容实例指针
 */
Supcap_Instance *Supcap_init(Supcap_Init_Config_s *supercap_config);

/**
 * @brief 发送超级电容控制信息
 *
 * @param instance 超级电容实例
 * @param data 超级电容控制信息
 */
void SupcapSend(Supcap_Instance *instance, uint8_t *data);

#endif // !SUPCAP_H