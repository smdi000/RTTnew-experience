/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-06-10     30818       the first version
 */
#ifndef __IMU_APP_H__
#define __IMU_APP_H__

#include <rtthread.h>
#include "drv_imu_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_CH_NUM      4
#define IMU_BAUDRATE    115200

int imu_app_start_one(int index);
int imu_app_start_all(void);

const imu_uart_data_t *imu_app_get_latest(int index);
void imu_app_print_one(int index);
void imu_app_print_all(void);

#ifdef __cplusplus
}
#endif

#endif
