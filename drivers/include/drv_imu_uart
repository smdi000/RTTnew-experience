#ifndef __DRV_IMU_UART_H__
#define __DRV_IMU_UART_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_UART_FRAME_HEAD_0        0x7E
#define IMU_UART_FRAME_HEAD_1        0x23

#define IMU_UART_FUNC_RAW_ACCEL      0x04
#define IMU_UART_FUNC_EULER          0x26

#define IMU_UART_MAX_FRAME_LEN       128

typedef struct
{
    float acc_g[3];          /* ax, ay, az, unit: g */
    float euler_deg[3];      /* roll, pitch, yaw, unit: degree */

    rt_uint32_t acc_update_count;
    rt_uint32_t euler_update_count;
    rt_uint32_t checksum_error_count;
    rt_uint32_t frame_error_count;
} imu_uart_data_t;

struct imu_uart_dev;
typedef struct imu_uart_dev imu_uart_dev_t;

typedef void (*imu_uart_data_callback_t)(imu_uart_dev_t *imu,
                                         rt_uint8_t func,
                                         const imu_uart_data_t *data);

struct imu_uart_dev
{
    char name[RT_NAME_MAX];

    rt_device_t serial;
    struct rt_semaphore rx_sem;
    rt_thread_t rx_thread;

    rt_uint8_t frame_buf[IMU_UART_MAX_FRAME_LEN];
    rt_uint8_t parse_state;
    rt_uint8_t frame_len;
    rt_uint8_t frame_index;

    volatile rt_bool_t running;

    imu_uart_data_t latest;
    imu_uart_data_callback_t callback;
};

/*
 * Initialize one serial IMU.
 *
 * uart_name example:
 *   "uart1", "uart2", "uart3", "uart4", "uart5", "uart6"
 *
 * baudrate:
 *   The Python script uses 115200.
 */
int imu_uart_init(imu_uart_dev_t *imu,
                  const char *uart_name,
                  rt_uint32_t baudrate,
                  imu_uart_data_callback_t callback);

int imu_uart_deinit(imu_uart_dev_t *imu);

const imu_uart_data_t *imu_uart_get_latest(imu_uart_dev_t *imu);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_IMU_UART_H__ */
