/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-05-15     30818       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>

static rt_device_t uart3_dev = RT_NULL;
static struct rt_semaphore uart3_rx_sem;

static rt_err_t uart3_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&uart3_rx_sem);
    return RT_EOK;
}

static void uart3_raw_thread_entry(void *parameter)
{
    rt_uint8_t ch;

    while (1)
    {
        while (rt_device_read(uart3_dev, 0, &ch, 1) == 1)
        {
            rt_kprintf("%02X ", ch);
        }

        rt_sem_take(&uart3_rx_sem, RT_WAITING_FOREVER);
    }
}

int uart3_raw_test(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_thread_t tid;

    uart3_dev = rt_device_find("uart3");
    if (uart3_dev == RT_NULL)
    {
        rt_kprintf("uart3 not found.\n");
        return -1;
    }

    config.baud_rate = 115200;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity    = PARITY_NONE;
    config.bufsz     = 512;

    rt_device_control(uart3_dev, RT_DEVICE_CTRL_CONFIG, &config);

    rt_sem_init(&uart3_rx_sem, "u3rx", 0, RT_IPC_FLAG_FIFO);

    rt_device_set_rx_indicate(uart3_dev, uart3_rx_ind);

    if (rt_device_open(uart3_dev, RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        rt_kprintf("uart3 open failed.\n");
        return -1;
    }

    tid = rt_thread_create("u3raw",
                           uart3_raw_thread_entry,
                           RT_NULL,
                           2048,
                           15,
                           10);

    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
        rt_kprintf("uart3 raw test started.\n");
    }
    else
    {
        rt_kprintf("create uart3 raw thread failed.\n");
        return -1;
    }

    return 0;
}
MSH_CMD_EXPORT(uart3_raw_test, uart3 raw receive test);
