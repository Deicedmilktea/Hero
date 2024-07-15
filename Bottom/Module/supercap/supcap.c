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

static Supcap_Instance *super_cap_instance = NULL; // 可以由app保存此指针

static void SuperCapRxCallback(CANInstance *_instance)
{
    uint8_t *rxbuff;
    Supcap_Msg_s *Msg;
    rxbuff = _instance->rx_buff;
    Msg = &super_cap_instance->cap_msg;
    Msg->vol = (uint16_t)(rxbuff[0] << 8 | rxbuff[1]);
    Msg->current = (uint16_t)(rxbuff[2] << 8 | rxbuff[3]);
    Msg->power = (uint16_t)(rxbuff[4] << 8 | rxbuff[5]);
}

Supcap_Instance *Supcap_init(Supcap_Init_Config_s *supercap_config)
{
    super_cap_instance = (Supcap_Instance *)malloc(sizeof(Supcap_Instance));
    memset(super_cap_instance, 0, sizeof(Supcap_Instance));

    supercap_config->can_config.can_module_callback = SuperCapRxCallback;
    super_cap_instance->can_ins = CANRegister(&supercap_config->can_config);
    return super_cap_instance;
}

void SupcapSend(Supcap_Instance *instance, uint8_t *data)
{
    memcpy(instance->can_ins->tx_buff, data, 8);
    CANTransmit(instance->can_ins, 1);
}

Supcap_Msg_s SuperCapGet(Supcap_Instance *instance)
{
    return instance->cap_msg;
}