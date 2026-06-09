#include <rtthread.h>
#include <stdlib.h>
#include "drv_imu_uart.h"

#define IMU_CH_NUM      4
#define IMU_BAUDRATE    115200

static imu_uart_dev_t imu_devs[IMU_CH_NUM];

static const char *imu_uart_names[IMU_CH_NUM] =
{
    "uart3",
    "uart4",
    "uart6",
    "uart5",
};

static rt_uint8_t imu_inited[IMU_CH_NUM] = {0};

static void imu_data_callback(imu_uart_dev_t *imu,
                              rt_uint8_t func,
                              const imu_uart_data_t *data)
{
    RT_UNUSED(imu);
    RT_UNUSED(func);
    RT_UNUSED(data);

    /*
     * 这里不要 rt_kprintf。
     * IMU 数据频率高，回调里打印很容易把 msh 打死。
     */
}

static void imu_print_one(int index)
{
    const imu_uart_data_t *data;

    if (index < 0 || index >= IMU_CH_NUM)
    {
        rt_kprintf("bad imu index=%d\n", index);
        return;
    }

    if (!imu_inited[index])
    {
        rt_kprintf("IMU%d %s not initialized.\n",
                   index + 1,
                   imu_uart_names[index]);
        return;
    }

    data = imu_uart_get_latest(&imu_devs[index]);
    if (data == RT_NULL)
    {
        rt_kprintf("IMU%d %s latest data NULL.\n",
                   index + 1,
                   imu_uart_names[index]);
        return;
    }

    rt_kprintf("[IMU%d %s] ACC=%d EULER=%d ERR checksum=%d frame=%d\n",
               index + 1,
               imu_uart_names[index],
               data->acc_update_count,
               data->euler_update_count,
               data->checksum_error_count,
               data->frame_error_count);

    rt_kprintf("  ACC mg: ax=%d ay=%d az=%d\n",
               (int)(data->acc_g[0] * 1000),
               (int)(data->acc_g[1] * 1000),
               (int)(data->acc_g[2] * 1000));

    rt_kprintf("  EULER mdeg: roll=%d pitch=%d yaw=%d\n",
               (int)(data->euler_deg[0] * 1000),
               (int)(data->euler_deg[1] * 1000),
               (int)(data->euler_deg[2] * 1000));
}

static int imu_start_index(int index)
{
    int ret;

    if (index < 0 || index >= IMU_CH_NUM)
    {
        rt_kprintf("usage: imu_start_one 0~3\n");
        return -1;
    }

    if (imu_inited[index])
    {
        rt_kprintf("IMU%d %s already initialized.\n",
                   index + 1,
                   imu_uart_names[index]);
        return 0;
    }

    rt_kprintf("starting IMU%d on %s...\n",
               index + 1,
               imu_uart_names[index]);

    ret = imu_uart_init(&imu_devs[index],
                        imu_uart_names[index],
                        IMU_BAUDRATE,
                        imu_data_callback);

    if (ret != RT_EOK)
    {
        rt_kprintf("IMU%d init FAILED on %s, ret=%d\n",
                   index + 1,
                   imu_uart_names[index],
                   ret);
        return ret;
    }

    imu_inited[index] = 1;

    rt_kprintf("IMU%d init OK on %s\n",
               index + 1,
               imu_uart_names[index]);

    return 0;
}

int imu_start_one(int argc, char **argv)
{
    int index;

    if (argc < 2)
    {
        rt_kprintf("usage: imu_start_one <0|1|2|3>\n");
        rt_kprintf("  0 -> uart3\n");
        rt_kprintf("  1 -> uart4\n");
        rt_kprintf("  2 -> uart7\n");
        rt_kprintf("  3 -> uart6\n");
        return -1;
    }

    index = atoi(argv[1]);
    return imu_start_index(index);
}
MSH_CMD_EXPORT(imu_start_one, start one IMU by index);

int imu_show_one(int argc, char **argv)
{
    int index;

    if (argc < 2)
    {
        rt_kprintf("usage: imu_show_one <0|1|2|3>\n");
        return -1;
    }

    index = atoi(argv[1]);
    imu_print_one(index);

    return 0;
}
MSH_CMD_EXPORT(imu_show_one, show one IMU latest data);

int imu_show_all(void)
{
    int i;

    for (i = 0; i < IMU_CH_NUM; i++)
    {
        imu_print_one(i);
    }

    return 0;
}
MSH_CMD_EXPORT(imu_show_all, show all initialized IMUs once);
/* ================= IMU periodic print thread ================= */

#define IMU_PRINT_STACK_SIZE    4096
#define IMU_PRINT_PRIORITY      25
#define IMU_PRINT_TIMESLICE     10
#define IMU_PRINT_MIN_PERIOD    200

static volatile rt_bool_t imu_print_running = RT_FALSE;
static rt_thread_t imu_print_tid = RT_NULL;
static rt_uint32_t imu_print_period_ms = 1000;

static void imu_print_compact_one(int index)
{
    const imu_uart_data_t *data;

    if (index < 0 || index >= IMU_CH_NUM)
    {
        return;
    }

    if (!imu_inited[index])
    {
        rt_kprintf("[IMU%d %s] not started\n",
                   index + 1,
                   imu_uart_names[index]);
        return;
    }

    data = imu_uart_get_latest(&imu_devs[index]);
    if (data == RT_NULL)
    {
        rt_kprintf("[IMU%d %s] data NULL\n",
                   index + 1,
                   imu_uart_names[index]);
        return;
    }

    /*
     * 一路只打一行，避免 console 被刷爆。
     * 不用 %f，全部转成整数打印。
     */
    rt_kprintf("[IMU%d %s] "
               "cnt A=%d E=%d err C=%d F=%d | "
               "ACC mg=(%d,%d,%d) | "
               "EULER mdeg=(%d,%d,%d)\n",
               index + 1,
               imu_uart_names[index],
               data->acc_update_count,
               data->euler_update_count,
               data->checksum_error_count,
               data->frame_error_count,
               (int)(data->acc_g[0] * 1000),
               (int)(data->acc_g[1] * 1000),
               (int)(data->acc_g[2] * 1000),
               (int)(data->euler_deg[0] * 1000),
               (int)(data->euler_deg[1] * 1000),
               (int)(data->euler_deg[2] * 1000));
}

static void imu_print_thread_entry(void *parameter)
{
    int i;

    RT_UNUSED(parameter);

    rt_kprintf("imu print thread running, period=%d ms\n",
               imu_print_period_ms);

    while (imu_print_running)
    {
        for (i = 0; i < IMU_CH_NUM; i++)
        {
            imu_print_compact_one(i);
        }

        rt_kprintf("\n");

        rt_thread_mdelay(imu_print_period_ms);
    }

    rt_kprintf("imu print thread exit.\n");

    imu_print_tid = RT_NULL;
}

int imu_print_start(int argc, char **argv)
{
    rt_uint32_t period = 1000;

    if (imu_print_running)
    {
        rt_kprintf("imu print thread already running.\n");
        return 0;
    }

    if (argc >= 2)
    {
        period = (rt_uint32_t)atoi(argv[1]);
    }

    if (period < IMU_PRINT_MIN_PERIOD)
    {
        rt_kprintf("period too small, force to %d ms\n",
                   IMU_PRINT_MIN_PERIOD);
        period = IMU_PRINT_MIN_PERIOD;
    }

    imu_print_period_ms = period;
    imu_print_running = RT_TRUE;

    imu_print_tid = rt_thread_create("imuprt",
                                     imu_print_thread_entry,
                                     RT_NULL,
                                     IMU_PRINT_STACK_SIZE,
                                     IMU_PRINT_PRIORITY,
                                     IMU_PRINT_TIMESLICE);

    if (imu_print_tid == RT_NULL)
    {
        imu_print_running = RT_FALSE;
        rt_kprintf("create imu print thread failed.\n");
        return -1;
    }

    rt_thread_startup(imu_print_tid);

    rt_kprintf("imu print thread started.\n");
    return 0;
}
MSH_CMD_EXPORT(imu_print_start, start imu periodic print);

int imu_print_stop(void)
{
    if (!imu_print_running)
    {
        rt_kprintf("imu print thread not running.\n");
        return 0;
    }

    imu_print_running = RT_FALSE;
    rt_kprintf("imu print thread stopping.\n");

    return 0;
}
MSH_CMD_EXPORT(imu_print_stop, stop imu periodic print);
