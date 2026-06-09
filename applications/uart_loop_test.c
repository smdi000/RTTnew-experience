/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-06-09     30818       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>

static int uart_loop_once(int argc, char **argv)
{
    const char *uart_name;
    rt_device_t dev;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_uint8_t tx_buf[] = {0x55, 0xAA, 0x7E, 0x23, 0x11, 0x22, 0x33, 0x44};
    rt_uint8_t rx_buf[16];
    int i;
    rt_size_t rx_len = 0;

    if (argc < 2)
    {
        rt_kprintf("usage: uart_loop_once <uart3|uart4|uart6|uart7>\n");
        return -1;
    }

    uart_name = argv[1];

    dev = rt_device_find(uart_name);
    if (dev == RT_NULL)
    {
        rt_kprintf("%s not found.\n", uart_name);
        return -1;
    }

    config.baud_rate = 115200;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity    = PARITY_NONE;
    config.bufsz     = 512;

    rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &config);

    if (rt_device_open(dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        rt_kprintf("%s open failed.\n", uart_name);
        return -1;
    }

    while (rt_device_read(dev, 0, rx_buf, sizeof(rx_buf)) > 0)
    {
        /* clear old rx data */
    }

    rt_kprintf("%s write: ", uart_name);
    for (i = 0; i < sizeof(tx_buf); i++)
    {
        rt_kprintf("%02X ", tx_buf[i]);
    }
    rt_kprintf("\n");

    rt_device_write(dev, 0, tx_buf, sizeof(tx_buf));

    rt_thread_mdelay(50);

    rx_len = rt_device_read(dev, 0, rx_buf, sizeof(rx_buf));

    rt_kprintf("%s read len=%d: ", uart_name, rx_len);
    for (i = 0; i < rx_len; i++)
    {
        rt_kprintf("%02X ", rx_buf[i]);
    }
    rt_kprintf("\n");

    rt_device_close(dev);

    return 0;
}
MSH_CMD_EXPORT(uart_loop_once, uart tx rx loopback test);
