/*
 * GO-M8010-6 RS485 test for RT-Thread
 *
 * UART2: PA2 TX, PA3 RX
 * RS485 DIR:
 *   PH8 -> DE
 *   PH7 -> RE
 *
 * TTL-RS485 module official logic:
 *   DE = 1: transmit enable
 *   DE = 0: transmit high-Z
 *   RE = 0: receive enable
 *   RE = 1: receive high-Z
 *
 * This version:
 *   - uses RT-Thread only to configure UART2 baudrate
 *   - does NOT open UART2 with INT_RX
 *   - uses USART2 registers for precise 4Mbps TX/RX
 *   - exports only one MSH command: go_ping
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

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

#define GO_UART_NAME            "uart2"
#define GO_BAUDRATE             4000000

#define RS485_DE_PIN            GET_PIN(H, 8)
#define RS485_RE_PIN            GET_PIN(H, 7)

#define GO_TX_LEN               17
#define GO_RX_LEN               16

static rt_device_t go_uart = RT_NULL;

/* ---------- Basic utilities ---------- */

static void dump_hex(const uint8_t *buf, int len)
{
    int i;

    for (i = 0; i < len; i++)
    {
        rt_kprintf("%02X ", buf[i]);
    }

    rt_kprintf("\r\n");
}

static void put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static int16_t get_i16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int32_t get_i32_le(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] |
                     ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) |
                     ((uint32_t)p[3] << 24));
}

/* ---------- RS485 direction control ---------- */

static void rs485_dir_init(void)
{
    rt_pin_mode(RS485_DE_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(RS485_RE_PIN, PIN_MODE_OUTPUT);

    /*
     * Default receive mode:
     *   DE=0: transmitter disabled
     *   RE=0: receiver enabled
     */
    rt_pin_write(RS485_DE_PIN, PIN_LOW);
    rt_pin_write(RS485_RE_PIN, PIN_LOW);
}

static void rs485_tx_mode(void)
{
    /*
     * Transmit mode:
     *   DE=1: transmitter enabled
     *   RE=1: receiver disabled
     */
    rt_pin_write(RS485_DE_PIN, PIN_HIGH);
    rt_pin_write(RS485_RE_PIN, PIN_HIGH);

    /*
     * Give the RS485 driver a very small enable setup time.
     * Keep this short because motor response is fast.
     */
    rt_hw_us_delay(2);
}

static void rs485_rx_mode(void)
{
    /*
     * Receive mode:
     *   DE=0: transmitter disabled
     *   RE=0: receiver enabled
     *
     * Do not delay here. At 4Mbps, 1 byte is only 2.5us.
     */
    rt_pin_write(RS485_DE_PIN, PIN_LOW);
    rt_pin_write(RS485_RE_PIN, PIN_LOW);
}

/* ---------- UART2 init ---------- */

static int go_uart_init(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_err_t ret;

    go_uart = rt_device_find(GO_UART_NAME);
    if (go_uart == RT_NULL)
    {
        rt_kprintf("cannot find %s\r\n", GO_UART_NAME);
        return -1;
    }

    config.baud_rate = GO_BAUDRATE;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity    = PARITY_NONE;
    config.bit_order = BIT_ORDER_LSB;
    config.invert    = NRZ_NORMAL;
    config.bufsz     = 256;

    ret = rt_device_control(go_uart, RT_DEVICE_CTRL_CONFIG, &config);
    if (ret != RT_EOK)
    {
        rt_kprintf("uart config failed: %d\r\n", ret);
        return -1;
    }

    /*
     * IMPORTANT:
     * Do NOT open with RT_DEVICE_FLAG_INT_RX here.
     * We read USART2->RDR directly below.
     * If INT_RX is enabled, RT-Thread ISR may consume received bytes first.
     */
    ret = rt_device_open(go_uart, RT_DEVICE_FLAG_RDWR);
    if (ret != RT_EOK && ret != -RT_EBUSY)
    {
        rt_kprintf("uart open failed: %d\r\n", ret);
        return -1;
    }

    rs485_dir_init();

    return 0;
}

/* ---------- Raw USART2 helpers ---------- */

static void uart2_raw_clear_errors(void)
{
    USART2->ICR = USART_ICR_PECF |
                  USART_ICR_FECF |
                  USART_ICR_NECF |
                  USART_ICR_ORECF |
                  USART_ICR_TCCF;
}

static void uart2_raw_clear_rx(void)
{
    volatile uint32_t tmp;

    uart2_raw_clear_errors();

    while (USART2->ISR & USART_ISR_RXNE_RXFNE)
    {
        tmp = USART2->RDR;
        (void)tmp;
    }

    uart2_raw_clear_errors();
}

static int uart2_raw_write(const uint8_t *buf, int len)
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
                return -1;
            }
        }

        USART2->TDR = buf[i];
    }

    /*
     * Wait until the shift register is also empty.
     * This means the final stop bit has gone out on TX.
     */
    {
        int timeout = 10000;

        while ((USART2->ISR & USART_ISR_TC) == 0)
        {
            if (--timeout <= 0)
            {
                return -2;
            }
        }
    }

    return len;
}

static int uart2_raw_read(uint8_t *buf, int max_len, int timeout_us)
{
    int len = 0;
    int t;

    for (t = 0; t < timeout_us; t++)
    {
        uint32_t isr = USART2->ISR;

        if (isr & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE))
        {
            rt_kprintf("USART2 error ISR=0x%08X\r\n", isr);
            uart2_raw_clear_errors();
        }

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

/* ---------- GO-M8010 protocol ---------- */

static void build_zero_cmd(uint8_t id, uint8_t mode, uint8_t tx[GO_TX_LEN])
{
    uint16_t crc;

    rt_memset(tx, 0, GO_TX_LEN);

    tx[0] = 0xFE;
    tx[1] = 0xEE;

    /*
     * RIS_Mode_t:
     *   bit0~3: motor id
     *   bit4~6: status/mode
     *   bit7: reserved
     */
    tx[2] = (uint8_t)((id & 0x0F) | ((mode & 0x07) << 4));

    /*
     * tx[3]~tx[14]:
     *   tor_des, spd_des, pos_des, k_pos, k_spd
     * All zero here for safe communication test.
     */

    crc = crc_ccitt(0, tx, 15);
    put_u16_le(&tx[15], crc);
}

static int go_send_recv(uint8_t id,
                        uint8_t mode,
                        uint8_t *rx,
                        int rx_size,
                        int timeout_us)
{
    uint8_t tx[GO_TX_LEN];
    int n;
    int rx_len;

    if (go_uart_init() != 0)
    {
        return -1;
    }

    build_zero_cmd(id, mode, tx);

    rt_kprintf("send %d bytes, id=%d mode=%d:\r\n", GO_TX_LEN, id, mode);
    dump_hex(tx, GO_TX_LEN);

    /*
     * Raw register path.
     */
    uart2_raw_clear_rx();

    rs485_tx_mode();

    n = uart2_raw_write(tx, GO_TX_LEN);
    if (n != GO_TX_LEN)
    {
        rt_kprintf("raw write failed, n=%d\r\n", n);
        rs485_rx_mode();
        return -1;
    }

    /*
     * Official logic equivalent:
     * HAL_UART_Transmit finished -> switch to receive immediately.
     */
    rs485_rx_mode();

    /*
     * Motor feedback:
     *   16 bytes @ 4Mbps 8N1 ~= 40us.
     * Give a wider polling window.
     */
    rx_len = uart2_raw_read(rx, rx_size, timeout_us);

    rt_kprintf("rx_len=%d\r\n", rx_len);

    if (rx_len > 0)
    {
        dump_hex(rx, rx_len);
    }

    return rx_len;
}

static void parse_feedback(const uint8_t rx[GO_RX_LEN])
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

    id   = rx[2] & 0x0F;
    mode = (rx[2] >> 4) & 0x07;

    torque_raw = get_i16_le(&rx[3]);
    speed_raw  = get_i16_le(&rx[5]);
    pos_raw    = get_i32_le(&rx[7]);

    temp = (int8_t)rx[11];
    err  = rx[12] & 0x07;

    crc_recv = (uint16_t)rx[14] | ((uint16_t)rx[15] << 8);
    crc_calc = crc_ccitt(0, rx, 14);

    rt_kprintf("feedback:\r\n");
    rt_kprintf("  header = %02X %02X\r\n", rx[0], rx[1]);
    rt_kprintf("  id=%d mode=%d temp=%d err=%d\r\n", id, mode, temp, err);
    rt_kprintf("  raw torque=%d speed=%d pos=%d\r\n",
               torque_raw, speed_raw, pos_raw);
    rt_kprintf("  crc recv=0x%04X calc=0x%04X %s\r\n",
               crc_recv,
               crc_calc,
               (crc_recv == crc_calc) ? "OK" : "BAD");
}

/* ---------- MSH command ---------- */

/*
 * Usage:
 *   go_ping
 *   go_ping 1
 *   go_ping 1 1
 *   go_ping 1 1 500
 *
 * Args:
 *   argv[1] = id, default 1
 *   argv[2] = mode, default 1
 *   argv[3] = rx timeout in us, default 500
 *
 * Recommended:
 *   go_ping 1 1
 */
static int go_ping(int argc, char **argv)
{
    uint8_t rx[32];
    uint8_t id = 1;
    uint8_t mode = 1;
    int timeout_us = 500;
    int rx_len;

    if (argc >= 2)
    {
        id = (uint8_t)atoi(argv[1]);
    }

    if (argc >= 3)
    {
        mode = (uint8_t)atoi(argv[2]);
    }

    if (argc >= 4)
    {
        timeout_us = atoi(argv[3]);
    }

    if (id > 15)
    {
        rt_kprintf("invalid id, use 0~14, 15 is broadcast without response\r\n");
        return -1;
    }

    if (mode > 7)
    {
        rt_kprintf("invalid mode, use 0~7\r\n");
        return -1;
    }

    if (timeout_us < 50 || timeout_us > 5000)
    {
        rt_kprintf("invalid timeout_us, suggest 500~1000\r\n");
        return -1;
    }

    rx_len = go_send_recv(id, mode, rx, GO_RX_LEN, timeout_us);

    if (rx_len != GO_RX_LEN)
    {
        rt_kprintf("no full 16-byte response\r\n");
        return -1;
    }

    /*
     * Real test showed GO-M8010 feedback header can be FD EE.
     * Some official sample code checks FE EE.
     * Accept both while debugging.
     */
    if (!((rx[0] == 0xFD && rx[1] == 0xEE) ||
          (rx[0] == 0xFE && rx[1] == 0xEE)))
    {
        rt_kprintf("unexpected feedback header\r\n");
        return -1;
    }

    parse_feedback(rx);

    return 0;
}
MSH_CMD_EXPORT(go_ping, go_ping);
