#include <applications/ad7606_app.h>
/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-06-10     30818       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include "drv_spi.h"

/*
 * AD7606 wiring:
 *
 * SPI1:
 *   PA5 -> AD7606 RD/SCLK
 *   PA6 -> AD7606 D7/DOUTA
 *   PA0 -> AD7606 CS
 *
 * Control:
 *   PB0 -> AD7606 CONVST_A / CONVST_B
 *   PB1 -> AD7606 BUSY
 *   PB2 -> AD7606 RESET
 *
 * Mode pins:
 *   PAR/SER       -> serial mode
 *   D15/BYTE_SEL  -> GND
 *   OS0/OS1/OS2   -> GND
 *   RANGE         -> high for +/-10V, low for +/-5V
 *   STBY          -> high
 */

#define AD7606_SPI_BUS_NAME     "spi1"
#define AD7606_SPI_DEV_NAME     "ad7606"

#define AD7606_CS_GPIO_PORT     GPIOA
#define AD7606_CS_GPIO_PIN      GPIO_PIN_0

#define AD7606_CONVST_PIN       GET_PIN(B, 0)
#define AD7606_BUSY_PIN         GET_PIN(B, 1)
#define AD7606_RESET_PIN        GET_PIN(B, 2)

/*
 * RANGE = high: +/-10V -> 10000mV full-scale positive side
 * RANGE = low : +/-5V  -> change this to 5000
 */
#define AD7606_RANGE_MV         10000

#define AD7606_SPI_MAX_HZ       (1000 * 1000)

static struct rt_spi_configuration ad7606_spi_cfg;
static struct rt_spi_device *ad7606_spi_dev = RT_NULL;

static rt_bool_t ad7606_gpio_ready = RT_FALSE;
static rt_bool_t ad7606_spi_ready = RT_FALSE;

static ad7606_sample_t ad7606_latest;
static rt_uint32_t ad7606_seq = 0;

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

static void ad7606_gpio_init_once(void)
{
    if (ad7606_gpio_ready)
    {
        return;
    }

    rt_pin_mode(AD7606_CONVST_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(AD7606_RESET_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(AD7606_BUSY_PIN, PIN_MODE_INPUT);

    rt_pin_write(AD7606_CONVST_PIN, PIN_LOW);
    rt_pin_write(AD7606_RESET_PIN, PIN_LOW);

    ad7606_gpio_ready = RT_TRUE;
}

static void ad7606_reset_pulse(void)
{
    ad7606_gpio_init_once();

    rt_pin_write(AD7606_RESET_PIN, PIN_LOW);
    rt_thread_mdelay(1);

    rt_pin_write(AD7606_RESET_PIN, PIN_HIGH);
    rt_thread_mdelay(1);

    rt_pin_write(AD7606_RESET_PIN, PIN_LOW);
    rt_thread_mdelay(1);
}

static void ad7606_start_conversion(void)
{
    ad7606_gpio_init_once();

    rt_pin_write(AD7606_CONVST_PIN, PIN_LOW);
    delay_us_soft(5);

    rt_pin_write(AD7606_CONVST_PIN, PIN_HIGH);
    delay_us_soft(5);

    rt_pin_write(AD7606_CONVST_PIN, PIN_LOW);
}

static void ad7606_wait_conversion_done(void)
{
    /*
     * AD7606 conversion time is several us when OS=000.
     * BUSY high may be too short to catch with rt_kprintf-level sampling.
     * Here we wait enough margin and then optionally wait BUSY low.
     */
    delay_us_soft(20);

    for (int i = 0; i < 1000; i++)
    {
        if (rt_pin_read(AD7606_BUSY_PIN) == PIN_LOW)
        {
            return;
        }

        delay_us_soft(2);
    }

    rt_kprintf("warning: ad7606 busy timeout\r\n");
}

static rt_err_t ad7606_spi_attach_once(void)
{
    rt_device_t dev;
    rt_err_t ret;

    dev = rt_device_find(AD7606_SPI_DEV_NAME);
    if (dev != RT_NULL)
    {
        ad7606_spi_dev = (struct rt_spi_device *)dev;
        return RT_EOK;
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();

    ret = rt_hw_spi_device_attach(AD7606_SPI_BUS_NAME,
                                  AD7606_SPI_DEV_NAME,
                                  AD7606_CS_GPIO_PORT,
                                  AD7606_CS_GPIO_PIN);

    if (ret != RT_EOK)
    {
        rt_kprintf("ad7606 attach failed, ret=%d\r\n", ret);
        return ret;
    }

    dev = rt_device_find(AD7606_SPI_DEV_NAME);
    if (dev == RT_NULL)
    {
        rt_kprintf("ad7606 device not found after attach\r\n");
        return -RT_ERROR;
    }

    ad7606_spi_dev = (struct rt_spi_device *)dev;
    return RT_EOK;
}

static rt_err_t ad7606_spi_config_once(void)
{
    rt_err_t ret;

    if (ad7606_spi_dev == RT_NULL)
    {
        rt_kprintf("ad7606 spi dev is NULL before config\r\n");
        return -RT_ERROR;
    }

    rt_memset(&ad7606_spi_cfg, 0, sizeof(ad7606_spi_cfg));

    ad7606_spi_cfg.data_width = 8;
    ad7606_spi_cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
    ad7606_spi_cfg.max_hz = AD7606_SPI_MAX_HZ;

    ret = rt_spi_configure(ad7606_spi_dev, &ad7606_spi_cfg);
    if (ret != RT_EOK)
    {
        rt_kprintf("ad7606 spi configure failed, ret=%d\r\n", ret);
        return ret;
    }

    ad7606_spi_ready = RT_TRUE;
    return RT_EOK;
}

int ad7606_hw_init(void)
{
    rt_err_t ret;

    ad7606_gpio_init_once();
    ad7606_reset_pulse();

    ret = ad7606_spi_attach_once();
    if (ret != RT_EOK)
    {
        return ret;
    }

    ret = ad7606_spi_config_once();
    if (ret != RT_EOK)
    {
        return ret;
    }

    return RT_EOK;
}

static rt_err_t ad7606_read_raw_bytes(uint8_t rx_buf[16])
{
    uint8_t tx_buf[16];
    rt_size_t ret;

    if (!ad7606_spi_ready || ad7606_spi_dev == RT_NULL)
    {
        rt_kprintf("ad7606 not initialized, run ad7606_init first\r\n");
        return -RT_ERROR;
    }

    rt_memset(tx_buf, 0xFF, sizeof(tx_buf));
    rt_memset(rx_buf, 0x00, 16);

    ret = rt_spi_transfer(ad7606_spi_dev,
                          tx_buf,
                          rx_buf,
                          16);

    if (ret != 16)
    {
        rt_kprintf("ad7606 spi transfer length error, ret=%d\r\n", ret);
        return -RT_ERROR;
    }

    return RT_EOK;
}

static void ad7606_parse_sample(const uint8_t rx_buf[16],
                                ad7606_sample_t *sample)
{
    for (int ch = 0; ch < AD7606_CH_NUM; ch++)
    {
        int16_t raw;

        raw = (int16_t)(((uint16_t)rx_buf[ch * 2] << 8) |
                         rx_buf[ch * 2 + 1]);

        sample->raw[ch] = raw;
        sample->mv[ch] = ((int)raw) * AD7606_RANGE_MV / 32768;
    }
}

int ad7606_sample_once(ad7606_sample_t *out)
{
    uint8_t rx_buf[16];
    rt_err_t ret;
    ad7606_sample_t sample;

    if (out == RT_NULL)
    {
        return -RT_ERROR;
    }

    if (!ad7606_spi_ready)
    {
        ret = ad7606_hw_init();
        if (ret != RT_EOK)
        {
            return ret;
        }
    }

    ad7606_start_conversion();
    ad7606_wait_conversion_done();

    ret = ad7606_read_raw_bytes(rx_buf);
    if (ret != RT_EOK)
    {
        return ret;
    }

    rt_memset(&sample, 0, sizeof(sample));

    sample.seq = ad7606_seq++;
    sample.tick = rt_tick_get();

    ad7606_parse_sample(rx_buf, &sample);

    ad7606_latest = sample;
    *out = sample;

    return RT_EOK;
}

const ad7606_sample_t *ad7606_get_latest(void)
{
    return &ad7606_latest;
}

void ad7606_print_sample(const ad7606_sample_t *sample)
{
    if (sample == RT_NULL)
    {
        rt_kprintf("ad7606 sample NULL\r\n");
        return;
    }

    rt_kprintf("AD7606 seq=%d tick=%d\r\n",
               sample->seq,
               sample->tick);

    for (int ch = 0; ch < AD7606_CH_NUM; ch++)
    {
        int mv = sample->mv[ch];
        int negative = 0;

        if (mv < 0)
        {
            negative = 1;
            mv = -mv;
        }

        rt_kprintf("CH%d raw=%6d voltage=%s%d.%03d V\r\n",
                   ch + 1,
                   sample->raw[ch],
                   negative ? "-" : "",
                   mv / 1000,
                   mv % 1000);
    }

    rt_kprintf("-------------------------\r\n");
}

/* ================= MSH commands ================= */

static int ad7606_init(int argc, char **argv)
{
    rt_err_t ret;

    (void)argc;
    (void)argv;

    ret = ad7606_hw_init();

    if (ret == RT_EOK)
    {
        rt_kprintf("ad7606 init OK\r\n");
    }
    else
    {
        rt_kprintf("ad7606 init failed, ret=%d\r\n", ret);
    }

    return ret;
}
MSH_CMD_EXPORT(ad7606_init, init ad7606);

static int ad7606_read_once(int argc, char **argv)
{
    ad7606_sample_t sample;
    rt_err_t ret;

    (void)argc;
    (void)argv;

    ret = ad7606_sample_once(&sample);
    if (ret != RT_EOK)
    {
        rt_kprintf("ad7606 read once failed, ret=%d\r\n", ret);
        return ret;
    }

    ad7606_print_sample(&sample);
    return RT_EOK;
}
MSH_CMD_EXPORT(ad7606_read_once, read ad7606 once);

static int ad7606_show_latest(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    ad7606_print_sample(ad7606_get_latest());
    return RT_EOK;
}
MSH_CMD_EXPORT(ad7606_show_latest, show latest ad7606 sample);

static int ad7606_reset_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    ad7606_gpio_init_once();
    ad7606_reset_pulse();

    rt_kprintf("ad7606 reset done\r\n");
    return RT_EOK;
}
MSH_CMD_EXPORT(ad7606_reset_cmd, reset ad7606);
