/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-06-13     30818       the first version
 */
/*
 * go_motor_min.c
 *
 * Minimal GO-M8010 / Unitree motor ping test for RT-Thread.
 *
 * Hardware:
 *   UART2:
 *     PA2 -> USART2_TX -> RS485 DI/TXD
 *     PA3 -> USART2_RX <- RS485 RO/RXD
 *
 * RS485 direction:
 *   PH8 -> DE
 *   PH7 -> /RE or RE control pin
 *
 * Direction logic:
 *   RX mode: DE=0, RE=0
 *   TX mode: DE=1, RE=1
 *
 * MSH:
 *   go_ping_min
 *   go_ping_min 1
 *   go_ping_min 1 1
 *   go_ping_min 1 1 20
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include <stdint.h>
#include <stddef.h>

#include "stm32h7xx.h"
#include "crc_ccitt.h"

/* ---------- USART register fallback definitions ---------- */

#ifndef USART_ISR_RXNE_RXFNE
#define USART_ISR_RXNE_RXFNE    (1UL << 5)
#endif

#ifndef USART_ISR_TC
#define USART_ISR_TC            (1UL << 6)
#endif

#ifndef USART_ISR_TXE_TXFNF
#define USART_ISR_TXE_TXFNF     (1UL << 7)
#endif

#ifndef USART_ISR_PE
#define USART_ISR_PE            (1UL << 0)
#endif

#ifndef USART_ISR_FE
#define USART_ISR_FE            (1UL << 1)
#endif

#ifndef USART_ISR_NE
#define USART_ISR_NE            (1UL << 2)
#endif

#ifndef USART_ISR_ORE
#define USART_ISR_ORE           (1UL << 3)
#endif

#ifndef USART_ICR_PECF
#define USART_ICR_PECF          (1UL << 0)
#endif

#ifndef USART_ICR_FECF
#define USART_ICR_FECF          (1UL << 1)
#endif

#ifndef USART_ICR_NECF
#define USART_ICR_NECF          (1UL << 2)
#endif

#ifndef USART_ICR_ORECF
#define USART_ICR_ORECF         (1UL << 3)
#endif

#ifndef USART_ICR_TCCF
#define USART_ICR_TCCF          (1UL << 6)
#endif

/* ---------- Config ---------- */

#define GO_MIN_UART_NAME             "uart2"
#define GO_MIN_BAUDRATE              4000000

#define GO_MIN_RS485_DE_PIN          GET_PIN(H, 8)
#define GO_MIN_RS485_RE_PIN          GET_PIN(H, 7)

#define GO_MIN_TX_LEN                17
#define GO_MIN_RX_LEN                16

#define GO_MIN_RX_TIMEOUT_US         5000

#define GO_MIN_PING_MAX_COUNT        100
#define GO_MIN_PING_INTERVAL_MS      30

static rt_device_t go_min_uart = RT_NULL;
static int go_min_uart_ready = 0;

/* ---------- tiny argument parser ---------- */

static int parse_i32_min(const char *s, int *out)
{
    int sign = 1;
    int v = 0;
    int any = 0;

    if (s == RT_NULL || out == RT_NULL)
    {
        return -1;
    }

    while (*s == ' ' || *s == '\t')
    {
        s++;
    }

    if (*s == '-')
    {
        sign = -1;
        s++;
    }
    else if (*s == '+')
    {
        s++;
    }

    while (*s >= '0' && *s <= '9')
    {
        any = 1;
        v = v * 10 + (*s - '0');
        s++;
    }

    if (!any)
    {
        return -1;
    }

    *out = sign * v;
    return 0;
}

/* ---------- basic helpers ---------- */

static void dump_hex_min(const uint8_t *buf, int len)
{
    int i;

    for (i = 0; i < len; i++)
    {
        rt_kprintf("%02X ", buf[i]);
    }

    rt_kprintf("\r\n");
}

static void put_u16_le_min(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static int16_t get_i16_le_min(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int32_t get_i32_le_min(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] |
                     ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) |
                     ((uint32_t)p[3] << 24));
}

/* ---------- RS485 direction ---------- */

static void go_min_rs485_dir_init(void)
{
    rt_pin_mode(GO_MIN_RS485_DE_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(GO_MIN_RS485_RE_PIN, PIN_MODE_OUTPUT);

    rt_pin_write(GO_MIN_RS485_DE_PIN, PIN_LOW);
    rt_pin_write(GO_MIN_RS485_RE_PIN, PIN_LOW);
}

static void go_min_rs485_tx_mode(void)
{
    rt_pin_write(GO_MIN_RS485_DE_PIN, PIN_HIGH);
    rt_pin_write(GO_MIN_RS485_RE_PIN, PIN_HIGH);

    rt_hw_us_delay(2);
}

static void go_min_rs485_rx_mode(void)
{
    rt_pin_write(GO_MIN_RS485_DE_PIN, PIN_LOW);
    rt_pin_write(GO_MIN_RS485_RE_PIN, PIN_LOW);
}

/* ---------- raw USART2 helpers ---------- */

static void go_min_uart2_clear_errors(void)
{
    USART2->ICR = USART_ICR_PECF |
                  USART_ICR_FECF |
                  USART_ICR_NECF |
                  USART_ICR_ORECF |
                  USART_ICR_TCCF;
}

static void go_min_uart2_clear_rx(void)
{
    volatile uint32_t tmp;
    int guard = 512;

    go_min_uart2_clear_errors();

    while ((USART2->ISR & USART_ISR_RXNE_RXFNE) && guard > 0)
    {
        tmp = USART2->RDR;
        (void)tmp;
        guard--;
    }

    go_min_uart2_clear_errors();
}

static int go_min_uart2_raw_write(const uint8_t *buf, int len)
{
    int i;

    USART2->ICR = USART_ICR_TCCF;

    for (i = 0; i < len; i++)
    {
        int timeout = 10000;

        while ((USART2->ISR & USART_ISR_TXE_TXFNF) == 0)
        {
            if (--timeout <= 0)
            {
                rt_kprintf("raw write TXE timeout, i=%d ISR=0x%08x\r\n",
                           i,
                           (rt_uint32_t)USART2->ISR);
                return -1;
            }
        }

        USART2->TDR = buf[i];
    }

    {
        int timeout = 10000;

        while ((USART2->ISR & USART_ISR_TC) == 0)
        {
            if (--timeout <= 0)
            {
                rt_kprintf("raw write TC timeout, ISR=0x%08x\r\n",
                           (rt_uint32_t)USART2->ISR);
                return -2;
            }
        }
    }

    return len;
}

static int go_min_uart2_raw_read(uint8_t *buf, int max_len, int timeout_us)
{
    int len = 0;
    int t;

    for (t = 0; t < timeout_us; t++)
    {
        while ((USART2->ISR & USART_ISR_RXNE_RXFNE) && len < max_len)
        {
            buf[len++] = (uint8_t)(USART2->RDR & 0xFF);
        }

        if (len >= max_len)
        {
            break;
        }

        rt_hw_us_delay(1);
    }

    return len;
}

/* ---------- UART2 init ---------- */

static int go_min_uart_init(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_err_t ret;

    if (go_min_uart_ready && go_min_uart != RT_NULL)
    {
        return 0;
    }

    go_min_uart = rt_device_find(GO_MIN_UART_NAME);
    if (go_min_uart == RT_NULL)
    {
        rt_kprintf("cannot find %s\r\n", GO_MIN_UART_NAME);
        return -1;
    }

    config.baud_rate = GO_MIN_BAUDRATE;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity    = PARITY_NONE;
    config.bit_order = BIT_ORDER_LSB;
    config.invert    = NRZ_NORMAL;
    config.bufsz     = 256;

    ret = rt_device_control(go_min_uart, RT_DEVICE_CTRL_CONFIG, &config);
    if (ret != RT_EOK)
    {
        rt_kprintf("uart2 config failed, ret=%d\r\n", ret);
        return -1;
    }

    /*
     * Do not use RT_DEVICE_FLAG_INT_RX here.
     * This file reads USART2->RDR directly.
     */
    ret = rt_device_open(go_min_uart, RT_DEVICE_FLAG_RDWR);
    if (ret != RT_EOK && ret != -RT_EBUSY)
    {
        rt_kprintf("uart2 open failed, ret=%d\r\n", ret);
        return -1;
    }

    go_min_rs485_dir_init();
    go_min_uart2_clear_errors();
    go_min_uart2_clear_rx();

    go_min_uart_ready = 1;

    rt_kprintf("go_min uart2 init OK, baud=%d\r\n", GO_MIN_BAUDRATE);

    return 0;
}

/* ---------- protocol ---------- */

static void go_min_build_zero_cmd(uint8_t id,
                                  uint8_t mode,
                                  uint8_t tx[GO_MIN_TX_LEN])
{
    uint16_t crc;

    rt_memset(tx, 0, GO_MIN_TX_LEN);

    tx[0] = 0xFE;
    tx[1] = 0xEE;
    tx[2] = (uint8_t)((id & 0x0F) | ((mode & 0x07) << 4));

    /*
     * tx[3]~tx[14]:
     * torque, speed, position, kp, kd are all zero.
     */

    crc = crc_ccitt(0, tx, 15);
    put_u16_le_min(&tx[15], crc);
}

static int go_min_feedback_valid(const uint8_t rx[GO_MIN_RX_LEN])
{
    uint16_t crc_recv;
    uint16_t crc_calc;

    if (!((rx[0] == 0xFD && rx[1] == 0xEE) ||
          (rx[0] == 0xFE && rx[1] == 0xEE)))
    {
        return 0;
    }

    crc_recv = (uint16_t)rx[14] | ((uint16_t)rx[15] << 8);
    crc_calc = crc_ccitt(0, rx, 14);

    return crc_recv == crc_calc;
}

static void go_min_parse_feedback(const uint8_t rx[GO_MIN_RX_LEN])
{
    uint16_t crc_recv;
    uint16_t crc_calc;

    uint8_t id;
    uint8_t mode;
    uint8_t err;
    int8_t temp;

    int16_t torque_raw;
    int16_t speed_raw;
    int32_t pos_raw;

    id = rx[2] & 0x0F;
    mode = (rx[2] >> 4) & 0x07;

    torque_raw = get_i16_le_min(&rx[3]);
    speed_raw  = get_i16_le_min(&rx[5]);
    pos_raw    = get_i32_le_min(&rx[7]);

    temp = (int8_t)rx[11];
    err  = rx[12] & 0x07;

    crc_recv = (uint16_t)rx[14] | ((uint16_t)rx[15] << 8);
    crc_calc = crc_ccitt(0, rx, 14);

    rt_kprintf("feedback:\r\n");
    rt_kprintf("  header=%02X %02X\r\n", rx[0], rx[1]);
    rt_kprintf("  id=%d mode=%d temp=%d err=%d\r\n", id, mode, temp, err);
    rt_kprintf("  torque_raw=%d speed_raw=%d pos_raw=%d\r\n",
               torque_raw,
               speed_raw,
               pos_raw);
    rt_kprintf("  crc recv=0x%04X calc=0x%04X %s\r\n",
               crc_recv,
               crc_calc,
               (crc_recv == crc_calc) ? "OK" : "BAD");
}

static int go_min_send_recv_frame(const uint8_t tx[GO_MIN_TX_LEN],
                                  uint8_t rx[GO_MIN_RX_LEN],
                                  int timeout_us,
                                  int verbose)
{
    int n;
    int rx_len;

    go_min_uart2_clear_rx();

    if (verbose)
    {
        rt_kprintf("TX frame:\r\n");
        dump_hex_min(tx, GO_MIN_TX_LEN);
    }

    go_min_rs485_tx_mode();

    n = go_min_uart2_raw_write(tx, GO_MIN_TX_LEN);
    if (n != GO_MIN_TX_LEN)
    {
        rt_kprintf("raw write failed, n=%d\r\n", n);
        go_min_rs485_rx_mode();
        return -1;
    }

    go_min_rs485_rx_mode();

    rx_len = go_min_uart2_raw_read(rx, GO_MIN_RX_LEN, timeout_us);

    if (verbose)
    {
        rt_kprintf("rx_len=%d\r\n", rx_len);
        if (rx_len > 0)
        {
            rt_kprintf("RX frame:\r\n");
            dump_hex_min(rx, rx_len);
        }
    }

    return rx_len;
}

/* ---------- MSH: go_ping_min ---------- */

static int go_ping_min(int argc, char **argv)
{
    uint8_t tx[GO_MIN_TX_LEN];
    uint8_t rx[GO_MIN_RX_LEN];

    uint8_t id = 1;
    uint8_t mode = 1;
    int count = 1;
    int tmp;
    int i;

    int ok_count = 0;
    int len_err_count = 0;
    int crc_err_count = 0;
    int header_err_count = 0;

    if (argc >= 2 && parse_i32_min(argv[1], &tmp) == 0)
    {
        id = (uint8_t)tmp;
    }

    if (argc >= 3 && parse_i32_min(argv[2], &tmp) == 0)
    {
        mode = (uint8_t)tmp;
    }

    if (argc >= 4 && parse_i32_min(argv[3], &tmp) == 0)
    {
        count = tmp;
    }

    if (id > 15)
    {
        rt_kprintf("invalid id=%d, use 0~14, 15 broadcast has no response\r\n", id);
        return -RT_ERROR;
    }

    if (mode > 7)
    {
        rt_kprintf("invalid mode=%d, use 0~7\r\n", mode);
        return -RT_ERROR;
    }

    if (count < 1)
    {
        count = 1;
    }

    if (count > GO_MIN_PING_MAX_COUNT)
    {
        count = GO_MIN_PING_MAX_COUNT;
    }

    if (go_min_uart_init() != 0)
    {
        return -RT_ERROR;
    }

    rt_kprintf("go_ping_min: id=%d mode=%d count=%d\r\n", id, mode, count);

    for (i = 0; i < count; i++)
    {
        int rx_len;
        int verbose;

        /*
         * count==1: print TX/RX details.
         * count>1 : no per-frame print, only summary.
         */
        verbose = (count == 1);

        rt_memset(tx, 0, sizeof(tx));
        rt_memset(rx, 0, sizeof(rx));

        go_min_build_zero_cmd(id, mode, tx);

        rx_len = go_min_send_recv_frame(tx,
                                        rx,
                                        GO_MIN_RX_TIMEOUT_US,
                                        verbose);

        if (rx_len != GO_MIN_RX_LEN)
        {
            len_err_count++;

            if (count == 1)
            {
                rt_kprintf("[1/1] length error, rx_len=%d\r\n", rx_len);
            }
        }
        else if (!((rx[0] == 0xFD && rx[1] == 0xEE) ||
                   (rx[0] == 0xFE && rx[1] == 0xEE)))
        {
            header_err_count++;

            if (count == 1)
            {
                rt_kprintf("[1/1] header error\r\n");
                dump_hex_min(rx, rx_len);
            }
        }
        else if (!go_min_feedback_valid(rx))
        {
            crc_err_count++;

            if (count == 1)
            {
                rt_kprintf("[1/1] crc error\r\n");
                go_min_parse_feedback(rx);
            }
        }
        else
        {
            ok_count++;

            if (count == 1)
            {
                rt_kprintf("[1/1] OK\r\n");
                go_min_parse_feedback(rx);
            }
        }

        if (count > 1)
        {
            rt_thread_mdelay(GO_MIN_PING_INTERVAL_MS);
        }
    }

    rt_kprintf("go_ping_min result:\r\n");
    rt_kprintf("  ok=%d count=%d\r\n", ok_count, count);
    rt_kprintf("  len_err=%d header_err=%d crc_err=%d\r\n",
               len_err_count,
               header_err_count,
               crc_err_count);

    return (ok_count == count) ? RT_EOK : -RT_ERROR;
}
MSH_CMD_EXPORT(go_ping_min, minimal go motor ping);
