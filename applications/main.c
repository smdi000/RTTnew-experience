#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_spi.h"
/*
 * AD7606 current wiring:
 *
 * SPI:
 *   PA5 -> AD7606 RD/SCLK
 *   PA6 -> AD7606 D7/DOUTA
 *   PA0 -> AD7606 CS
 *
 * Control:
 *   PB0 -> AD7606 CA / CONVST_A
 *   PB1 -> AD7606 BUSY
 *   PB2 -> AD7606 RST
 *
 * Current stage:
 *   1. Test AD7606 GPIO control
 *   2. Only find spi1
 *   3. Do NOT attach SPI device下载
 *   4. Do NOT configure SPI
 *   5. Do NOT transfer SPI data
 */
static struct rt_spi_configuration ad7606_spi_cfg;
static struct rt_spi_device *ad7606_spi_dev = RT_NULL;
#define AD7606_CONVST_PIN    GET_PIN(B, 0)
#define AD7606_BUSY_PIN      GET_PIN(B, 1)
#define AD7606_RESET_PIN     GET_PIN(B, 2)

static void delay_us_soft(volatile int us)
{
    while (us--)
    {
        for (volatile int i = 0; i < 200; i++)
        {
            __NOP();
        }
    }
}

static void ad7606_gpio_init(void)
{
    rt_pin_mode(AD7606_CONVST_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(AD7606_RESET_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(AD7606_BUSY_PIN, PIN_MODE_INPUT);

    rt_pin_write(AD7606_CONVST_PIN, PIN_LOW);
    rt_pin_write(AD7606_RESET_PIN, PIN_LOW);

    rt_kprintf("ad7606 gpio init done\r\n");
}

static void spi_find_test(void)
{
    rt_device_t spi_dev;

    rt_kprintf("before find spi1\r\n");

    spi_dev = rt_device_find("spi1");

    if (spi_dev == RT_NULL)
    {
        rt_kprintf("spi1 not found\r\n");
    }
    else
    {
        rt_kprintf("spi1 found\r\n");
    }

    rt_kprintf("after find spi1\r\n");
}
static void spi_attach_test(void)
{
    rt_err_t ret;
    rt_device_t dev;

    rt_kprintf("before attach ad7606\r\n");

    /*
     * PA0 is used as software CS.
     * Make sure GPIOA clock is enabled before HAL_GPIO_Init in attach.
     */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*
     * Important:
     * RT_NAME_MAX = 8, so object name must be <= 7 chars.
     * "ad7606" is safe.
     */
    ret = rt_hw_spi_device_attach("spi1",
                                  "ad7606",
                                  GPIOA,
                                  GPIO_PIN_0);

    rt_kprintf("after attach ad7606, ret = %d\r\n", ret);

    if (ret != RT_EOK)
    {
        rt_kprintf("attach ad7606 failed\r\n");
        return;
    }

    dev = rt_device_find("ad7606");

    if (dev == RT_NULL)
    {
        rt_kprintf("ad7606 device not found\r\n");
    }
    else
    {
        rt_kprintf("ad7606 device found\r\n");
    }

    rt_kprintf("spi attach test done\r\n");
}
static void spi_config_test(void)
{
    rt_err_t ret;
    struct rt_spi_device *spi_dev;

    rt_kprintf("before find ad7606 for configure\r\n");

    spi_dev = (struct rt_spi_device *)rt_device_find("ad7606");

    if (spi_dev == RT_NULL)
    {
        rt_kprintf("ad7606 not found before configure\r\n");
        return;
    }

    rt_kprintf("ad7606 found before configure\r\n");

    rt_memset(&ad7606_spi_cfg, 0, sizeof(ad7606_spi_cfg));

    ad7606_spi_cfg.data_width = 8;
    ad7606_spi_cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
    ad7606_spi_cfg.max_hz = 1000 * 1000;

    rt_kprintf("before rt_spi_configure\r\n");

    ret = rt_spi_configure(spi_dev, &ad7606_spi_cfg);

    rt_kprintf("after rt_spi_configure, ret = %d\r\n", ret);

    if (ret != RT_EOK)
    {
        rt_kprintf("configure ad7606 failed\r\n");
        return;
    }

    ad7606_spi_dev = spi_dev;

    rt_kprintf("spi configure test done\r\n");
}

static void ad7606_reset_test(void)
{
    rt_kprintf("reset pulse start\r\n");

    rt_pin_write(AD7606_RESET_PIN, PIN_LOW);
    rt_thread_mdelay(1);

    rt_pin_write(AD7606_RESET_PIN, PIN_HIGH);
    rt_thread_mdelay(1);

    rt_pin_write(AD7606_RESET_PIN, PIN_LOW);
    rt_thread_mdelay(1);

    rt_kprintf("reset pulse done\r\n");
}

static void ad7606_convst_test(void)
{
    int busy_before;

    busy_before = rt_pin_read(AD7606_BUSY_PIN);

    rt_kprintf("BUSY before CONVST = %d\r\n", busy_before);

    rt_pin_write(AD7606_CONVST_PIN, PIN_LOW);
    delay_us_soft(5);

    rt_pin_write(AD7606_CONVST_PIN, PIN_HIGH);
    delay_us_soft(5);

    rt_pin_write(AD7606_CONVST_PIN, PIN_LOW);

    rt_kprintf("CONVST pulse done\r\n");

    /*
     * 不死等 BUSY，只采样几次。
     * BUSY 高电平可能只有几微秒，所以这里一直读到 0 不一定是错。
     */
    for (int i = 0; i < 10; i++)
    {
        rt_kprintf("BUSY sample %d = %d\r\n",
                   i,
                   rt_pin_read(AD7606_BUSY_PIN));

        rt_thread_mdelay(10);
    }
}
static void ad7606_spi_read_test(void)
{
    uint8_t tx_buf[16];
    uint8_t rx_buf[16];
    rt_size_t ret;

    if (ad7606_spi_dev == RT_NULL)
    {
        rt_kprintf("ad7606 spi dev is NULL\r\n");
        return;
    }

    rt_memset(tx_buf, 0xFF, sizeof(tx_buf));
    rt_memset(rx_buf, 0x00, sizeof(rx_buf));

    rt_kprintf("before spi transfer\r\n");

    /*
     * Single DOUTA read:
     * AD7606 has 8 channels.
     * 8 channels * 16 bits = 128 bits = 16 bytes.
     *
     * Current wiring:
     *   PA5 -> RD/SCLK
     *   PA6 -> D7/DOUTA
     *   PA0 -> CS
     */
    ret = rt_spi_transfer(ad7606_spi_dev,
                          tx_buf,
                          rx_buf,
                          sizeof(rx_buf));

    rt_kprintf("after spi transfer, ret = %d\r\n", ret);

    if (ret != sizeof(rx_buf))
    {
        rt_kprintf("spi transfer length error\r\n");
        return;
    }

    rt_kprintf("RX bytes: ");
    for (int i = 0; i < 16; i++)
    {
        rt_kprintf("%02X ", rx_buf[i]);
    }
    rt_kprintf("\r\n");

    for (int ch = 0; ch < 6; ch++)
    {
        int16_t raw;
        int mv;
        int negative = 0;

        raw = (int16_t)(((uint16_t)rx_buf[ch * 2] << 8) |
                         rx_buf[ch * 2 + 1]);

        /*
         * Current assumption:
         * RANGE = high, so input range is +/-10V.
         *
         * voltage_mV = raw * 10000 / 32768
         */
        mv = ((int)raw) * 10000 / 32768;

        if (mv < 0)
        {
            negative = 1;
            mv = -mv;
        }

        rt_kprintf("CH%d raw = %6d, voltage = %s%d.%03d V\r\n",
                   ch + 1,
                   raw,
                   negative ? "-" : "",
                   mv / 1000,
                   mv % 1000);
    }

    rt_kprintf("-------------------------\r\n");
}
int main(void)
{
    rt_kprintf("main start\n");
    rt_kprintf("ad7606 disabled, clock debug mode\n");

    return 0;
}
