#ifndef SUPCAP_H
#define SUPCAP_H

#include "bsp_can.h"

#pragma pack(1)
typedef struct
{
    uint16_t voltage; // 超电电压
    uint16_t power;   // 超电功率
    uint8_t state;    // 超电状态
} SupercapRxData_s;

// 超电发送数据
typedef struct
{
    uint16_t buffer; // 缓冲能量
    uint16_t power;  // 底盘功率
    uint8_t state;   // 超电状态
} SupercapTxData_s;
#pragma pack()

/* 超级电容实例 */
typedef struct
{
    CAN_Instance *can_ins;         // CAN实例
    SupercapRxData_s cap_rx_data; // 超级电容接收数据
    SupercapTxData_s cap_tx_data; // 超级电容发送数据
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

void SupcapSetData(uint16_t buffer, uint16_t power, uint8_t state);

void SupcapSend();

#endif // !SUPCAP_H