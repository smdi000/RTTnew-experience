/*
 * IMU application wrapper
 *
 * Four IMU UART mapping:
 *
 * IMU1 -> uart3 -> PB10/PB11
 * IMU2 -> uart4 -> PC10/PC11
 * IMU3 -> uart6 -> PG14/PG9
 * IMU4 -> uart5 -> PB13/PB12
 *
 * This file does not auto-start anything.
 * Use msh commands manually:
 *
 *   imu_start_one 0
 *   imu_start_all
 *   imu_show_one 0
 *   imu_show_all
 *   imu_print_start 500
 *   imu_print_stop
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include "imu_app.h"
#include <string.h>

static imu_uart_dev_t imu_devs[IMU_CH_NUM];

static const char *imu_uart_names[IMU_CH_NUM] =
{
    "uart3",
    "uart4",
    "uart6",
    "uart5",
};

static rt_uint8_t imu_inited[IMU_CH_NUM] = {0};

static volatile rt_uint32_t imu_acc_cb_count[IMU_CH_NUM] = {0};
static volatile rt_uint32_t imu_euler_cb_count[IMU_CH_NUM] = {0};

/* ================= callback ================= */

static int imu_find_index_by_dev(imu_uart_dev_t *imu)
{
    int i;

    for (i = 0; i < IMU_CH_NUM; i++)
    {
        if (&imu_devs[i] == imu)
        {
            return i;
        }
    }

    return -1;
}

static void imu_data_callback(imu_uart_dev_t *imu,
                              rt_uint8_t func,
                              const imu_uart_data_t *data)
{
    int index;

    RT_UNUSED(data);

    /*
     * Do not print here.
     * Do not delay here.
     * Do not block here.
     *
     * UART RX callback must stay lightweight.
     */
    index = imu_find_index_by_dev(imu);
    if (index < 0)
    {
        return;
    }

    if (func == IMU_UART_FUNC_RAW_ACCEL)
    {
        imu_acc_cb_count[index]++;
    }
    else if (func == IMU_UART_FUNC_EULER)
    {
        imu_euler_cb_count[index]++;
    }
}

/* ================= public API ================= */

int imu_app_start_one(int index)
{
    int ret;

    if (index < 0 || index >= IMU_CH_NUM)
    {
        rt_kprintf("bad imu index=%d, valid range: 0~3\n", index);
        return -RT_ERROR;
    }

    if (imu_inited[index])
    {
        rt_kprintf("IMU%d %s already initialized.\n",
                   index + 1,
                   imu_uart_names[index]);
        return RT_EOK;
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

    return RT_EOK;
}

int imu_app_start_all(void)
{
    int i;
    int ret;
    int fail = 0;

    for (i = 0; i < IMU_CH_NUM; i++)
    {
        ret = imu_app_start_one(i);
        if (ret != RT_EOK)
        {
            fail++;
        }
    }

    if (fail == 0)
    {
        rt_kprintf("all IMUs init OK\n");
        return RT_EOK;
    }

    rt_kprintf("IMU init done, failed=%d\n", fail);
    return -RT_ERROR;
}

const imu_uart_data_t *imu_app_get_latest(int index)
{
    if (index < 0 || index >= IMU_CH_NUM)
    {
        return RT_NULL;
    }

    if (!imu_inited[index])
    {
        return RT_NULL;
    }

    return imu_uart_get_latest(&imu_devs[index]);
}

imu_uart_dev_t *imu_app_get_dev(int index)
{
    if (index < 0 || index >= IMU_CH_NUM)
    {
        return RT_NULL;
    }

    return &imu_devs[index];
}

void imu_app_print_one(int index)
{
    const imu_uart_data_t *data;

    if (index < 0 || index >= IMU_CH_NUM)
    {
        rt_kprintf("bad imu index=%d, valid range: 0~3\n", index);
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

    rt_kprintf("[IMU%d %s] ACC=%d EULER=%d cbA=%d cbE=%d ERR checksum=%d frame=%d\n",
               index + 1,
               imu_uart_names[index],
               data->acc_update_count,
               data->euler_update_count,
               imu_acc_cb_count[index],
               imu_euler_cb_count[index],
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

void imu_app_print_all(void)
{
    int i;

    for (i = 0; i < IMU_CH_NUM; i++)
    {
        imu_app_print_one(i);
    }
}

/* ================= compact print thread ================= */

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
     * One line per IMU.
     * Do not use %f in rt_kprintf.
     */
    rt_kprintf("[IMU%d %s] "
               "cnt A=%d E=%d cbA=%d cbE=%d err C=%d F=%d | "
               "ACC mg=(%d,%d,%d) | "
               "EULER mdeg=(%d,%d,%d)\n",
               index + 1,
               imu_uart_names[index],
               data->acc_update_count,
               data->euler_update_count,
               imu_acc_cb_count[index],
               imu_euler_cb_count[index],
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

/* ================= single MSH command ================= */

static void imu_cmd_usage(void)
{
    rt_kprintf("usage:\n");
    rt_kprintf("  imu start <0|1|2|3>\n");
    rt_kprintf("  imu start all\n");
    rt_kprintf("  imu show <0|1|2|3>\n");
    rt_kprintf("  imu show all\n");
    rt_kprintf("  imu print_start [period_ms]\n");
    rt_kprintf("  imu print_stop\n");
    rt_kprintf("mapping:\n");
    rt_kprintf("  0 -> uart3\n");
    rt_kprintf("  1 -> uart4\n");
    rt_kprintf("  2 -> uart6\n");
    rt_kprintf("  3 -> uart5\n");
}

int imu(int argc, char **argv)
{
    int index;

    if (argc < 2)
    {
        imu_cmd_usage();
        return -RT_ERROR;
    }

    if (strcmp(argv[1], "start") == 0)
    {
        if (argc < 3)
        {
            imu_cmd_usage();
            return -RT_ERROR;
        }

        if (strcmp(argv[2], "all") == 0)
        {
            return imu_app_start_all();
        }

        index = atoi(argv[2]);
        return imu_app_start_one(index);
    }

    if (strcmp(argv[1], "show") == 0)
    {
        if (argc < 3)
        {
            imu_cmd_usage();
            return -RT_ERROR;
        }

        if (strcmp(argv[2], "all") == 0)
        {
            imu_app_print_all();
            return RT_EOK;
        }

        index = atoi(argv[2]);
        imu_app_print_one(index);
        return RT_EOK;
    }

    if (strcmp(argv[1], "print_start") == 0)
    {
        rt_uint32_t period = 1000;

        if (imu_print_running)
        {
            rt_kprintf("imu print thread already running, period=%d ms\n",
                       imu_print_period_ms);
            return RT_EOK;
        }

        if (argc >= 3)
        {
            period = (rt_uint32_t)atoi(argv[2]);
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
            return -RT_ERROR;
        }

        rt_thread_startup(imu_print_tid);

        rt_kprintf("imu print thread started.\n");
        return RT_EOK;
    }

    if (strcmp(argv[1], "print_stop") == 0)
    {
        if (!imu_print_running)
        {
            rt_kprintf("imu print thread not running.\n");
            return RT_EOK;
        }

        imu_print_running = RT_FALSE;
        rt_kprintf("imu print thread stopping.\n");
        return RT_EOK;
    }

    imu_cmd_usage();
    return -RT_ERROR;
}
MSH_CMD_EXPORT(imu, imu command);
