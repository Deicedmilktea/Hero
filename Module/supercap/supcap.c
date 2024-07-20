/**
 * @file supcap.c
 * @author Reeve Ni (reeveni666@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-07-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "supcap.h"
#include "stdlib.h"
#include "string.h"

static Supcap_Instance *super_cap_instance = NULL; // 可以由app保存此指针

static void SuperCapRxCallback(CAN_Instance *_instance)
{
    uint8_t *rxbuff;
    SupercapRxData_s *rx_data;
    rxbuff = _instance->rx_buff;
    rx_data = &super_cap_instance->cap_rx_data;
    rx_data->voltage = (uint16_t)(rxbuff[0] << 8 | rxbuff[1]);
    rx_data->power = (uint16_t)(rxbuff[2] << 8 | rxbuff[3]);
    rx_data->state = rxbuff[4];
}

Supcap_Instance *Supcap_init(Supcap_Init_Config_s *supercap_config)
{
    super_cap_instance = (Supcap_Instance *)malloc(sizeof(Supcap_Instance));
    memset(super_cap_instance, 0, sizeof(Supcap_Instance));

    supercap_config->can_config.can_module_callback = SuperCapRxCallback;
    super_cap_instance->can_ins = CANRegister(&supercap_config->can_config);
    return super_cap_instance;
}

/**
 * @brief
 *
 * @param buffer 缓冲能量
 * @param power 底盘功率
 * @param state 超电设定状态
 */
void SupcapSetData(uint16_t buffer, uint16_t power, uint8_t state)
{
    super_cap_instance->cap_tx_data.buffer = buffer;
    super_cap_instance->cap_tx_data.power = power;
    super_cap_instance->cap_tx_data.state = state;
}

void SupcapSend()
{
    uint8_t data[5];
    memcpy(&data[0], &super_cap_instance->cap_tx_data.buffer, 2);
    memcpy(&data[2], &super_cap_instance->cap_tx_data.power, 2);
    data[4] = super_cap_instance->cap_tx_data.state;

    memcpy(super_cap_instance->can_ins->tx_buff, data, 5);
    CANTransmit(super_cap_instance->can_ins, 1);
}

SupercapRxData_s SuperCapGet(Supcap_Instance *instance)
{
    return instance->cap_rx_data;
}