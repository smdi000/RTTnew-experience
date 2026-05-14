#include <rtthread.h>
#include "drv_imu_uart.h"

static imu_uart_dev_t imu1;

static volatile rt_uint32_t imu_acc_count = 0;
static volatile rt_uint32_t imu_euler_count = 0;

static void imu_data_callback(imu_uart_dev_t *imu,
                              rt_uint8_t func,
                              const imu_uart_data_t *data)
{
    RT_UNUSED(imu);
    RT_UNUSED(data);

    /*
     * 注意：
     * 这里不要 rt_kprintf。
     * IMU 数据来得很快，如果每帧都打印，很容易把系统打爆。
     */
    if (func == IMU_UART_FUNC_RAW_ACCEL)
    {
        imu_acc_count++;
    }
    else if (func == IMU_UART_FUNC_EULER)
    {
        imu_euler_count++;
    }
}

int imu_test_init(void)
{
    return imu_uart_init(&imu1, "uart3", 115200, imu_data_callback);
}
MSH_CMD_EXPORT(imu_test_init, start imu uart parser);

int imu_show(void)
{
    const imu_uart_data_t *data;

    data = imu_uart_get_latest(&imu1);
    if (data == RT_NULL)
    {
        rt_kprintf("imu not initialized.\n");
        return -1;
    }

    rt_kprintf("ACC count=%d, EULER count=%d\n",
               data->acc_update_count,
               data->euler_update_count);

    rt_kprintf("ERR checksum=%d, frame=%d\n",
               data->checksum_error_count,
               data->frame_error_count);

    rt_kprintf("ACC milli-g: ax=%d ay=%d az=%d\n",
               (int)(data->acc_g[0] * 1000),
               (int)(data->acc_g[1] * 1000),
               (int)(data->acc_g[2] * 1000));

    rt_kprintf("EULER milli-deg: roll=%d pitch=%d yaw=%d\n",
               (int)(data->euler_deg[0] * 1000),
               (int)(data->euler_deg[1] * 1000),
               (int)(data->euler_deg[2] * 1000));

    return 0;
}
MSH_CMD_EXPORT(imu_show, show latest imu data);
