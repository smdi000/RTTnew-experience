#include <rtthread.h>
#include <rtdevice.h>

static int uart2_test(int argc, char **argv)
{
    rt_device_t uart2;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    const char msg[] = "uart2 hello 4M\r\n";
    rt_err_t ret;

    uart2 = rt_device_find("uart2");
    if (uart2 == RT_NULL)
    {
        rt_kprintf("uart2 not found\r\n");
        return -1;
    }

    config.baud_rate = 4000000;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity    = PARITY_NONE;

    ret = rt_device_control(uart2, RT_DEVICE_CTRL_CONFIG, &config);
    if (ret != RT_EOK)
    {
        rt_kprintf("uart2 config failed %d\r\n", ret);
        return -1;
    }

    ret = rt_device_open(uart2, RT_DEVICE_FLAG_RDWR);
    if (ret != RT_EOK && ret != -RT_EBUSY)
    {
        rt_kprintf("uart2 open failed %d\r\n", ret);
        return -1;
    }

    rt_device_write(uart2, 0, msg, sizeof(msg) - 1);

    rt_kprintf("uart2 send done, baudrate = 4000000\r\n");

    return 0;
}
MSH_CMD_EXPORT(uart2_test, uart2_test);
static int uart2_loop(int argc, char **argv)
{
    rt_device_t uart2;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_err_t ret;

    const char tx_buf[] = "UART2_4M_LOOPBACK_TEST\r\n";
    char rx_buf[64];
    int rx_len = 0;
    int i;

    uart2 = rt_device_find("uart2");
    if (uart2 == RT_NULL)
    {
        rt_kprintf("uart2 not found\r\n");
        return -1;
    }

    config.baud_rate = 4000000;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity    = PARITY_NONE;
    config.bit_order = BIT_ORDER_LSB;
    config.invert    = NRZ_NORMAL;
    config.bufsz     = 256;

    ret = rt_device_control(uart2, RT_DEVICE_CTRL_CONFIG, &config);
    if (ret != RT_EOK)
    {
        rt_kprintf("uart2 config failed %d\r\n", ret);
        return -1;
    }

    ret = rt_device_open(uart2, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    if (ret != RT_EOK)
    {
        rt_kprintf("uart2 open failed %d\r\n", ret);
        rt_kprintf("please reset board and run uart2_loop first\r\n");
        return -1;
    }

    while (rt_device_read(uart2, 0, rx_buf, sizeof(rx_buf)) > 0)
    {
        /* clear old rx data */
    }

    rt_kprintf("uart2 loopback send: %s", tx_buf);

    rt_device_write(uart2, 0, tx_buf, sizeof(tx_buf) - 1);

    rt_thread_mdelay(10);

    rx_len = rt_device_read(uart2, 0, rx_buf, sizeof(rx_buf) - 1);
    if (rx_len <= 0)
    {
        rt_kprintf("uart2 loopback rx empty\r\n");
        rt_device_close(uart2);
        return -1;
    }

    rx_buf[rx_len] = '\0';

    rt_kprintf("uart2 loopback recv: %s", rx_buf);
    rt_kprintf("rx_len = %d\r\n", rx_len);

    for (i = 0; i < sizeof(tx_buf) - 1; i++)
    {
        if (i >= rx_len || rx_buf[i] != tx_buf[i])
        {
            rt_kprintf("loopback mismatch at %d, tx=0x%02x, rx=0x%02x\r\n",
                       i, tx_buf[i], rx_buf[i]);
            rt_device_close(uart2);
            return -1;
        }
    }

    rt_kprintf("uart2 4M loopback OK\r\n");

    rt_device_close(uart2);

    return 0;
}
MSH_CMD_EXPORT(uart2_loop, uart2_loop);
