/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-06-10     30818       the first version
 */
#ifndef __AD7606_APP_H__
#define __AD7606_APP_H__

#include <rtthread.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AD7606_CH_NUM      8

typedef struct
{
    rt_uint32_t seq;
    rt_tick_t tick;

    int16_t raw[AD7606_CH_NUM];
    int mv[AD7606_CH_NUM];
} ad7606_sample_t;

int ad7606_hw_init(void);
int ad7606_sample_once(ad7606_sample_t *out);
const ad7606_sample_t *ad7606_get_latest(void);
void ad7606_print_sample(const ad7606_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif
