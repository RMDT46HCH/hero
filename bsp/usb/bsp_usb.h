#pragma once
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_if.h"

typedef struct
{
    USBCallback tx_cbk;
    USBCallback rx_cbk;
} USB_Init_Config_s;

/* @note 虚拟串口的波特率/校验位/数据位等动态可变,取决于上位机的设定 */
/* 使用时不需要关心这些设置(作为从机) */

uint8_t *USBInit(USB_Init_Config_s usb_conf); // bsp初始化时调用会重新枚举设备

void USBTransmit(uint8_t *buffer, uint16_t len); // 通过usb发送数据