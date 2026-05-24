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
static int g_verbose = 1;
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
static void put_i16_le(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((uint16_t)v & 0xFF);
    p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFF);
}

static void put_i32_le(uint8_t *p, int32_t v)
{
    p[0] = (uint8_t)((uint32_t)v & 0xFF);
    p[1] = (uint8_t)(((uint32_t)v >> 8) & 0xFF);
    p[2] = (uint8_t)(((uint32_t)v >> 16) & 0xFF);
    p[3] = (uint8_t)(((uint32_t)v >> 24) & 0xFF);
}

/*
 * Build raw command:
 * tx[0..1]  = FE EE
 * tx[2]     = id + mode
 * tx[3..4]  = tor_des
 * tx[5..6]  = spd_des
 * tx[7..10] = pos_des
 * tx[11..12]= k_pos
 * tx[13..14]= k_spd
 * tx[15..16]= CRC16
 */
static void build_raw_cmd(uint8_t id,
                          uint8_t mode,
                          int16_t tor_raw,
                          int16_t spd_raw,
                          int32_t pos_raw,
                          int16_t kp_raw,
                          int16_t kd_raw,
                          uint8_t tx[GO_TX_LEN])
{
    uint16_t crc;

    rt_memset(tx, 0, GO_TX_LEN);

    tx[0] = 0xFE;
    tx[1] = 0xEE;
    tx[2] = (uint8_t)((id & 0x0F) | ((mode & 0x07) << 4));

    put_i16_le(&tx[3],  tor_raw);
    put_i16_le(&tx[5],  spd_raw);
    put_i32_le(&tx[7],  pos_raw);
    put_i16_le(&tx[11], kp_raw);
    put_i16_le(&tx[13], kd_raw);

    crc = crc_ccitt(0, tx, 15);
    put_u16_le(&tx[15], crc);
}

static int feedback_crc_ok(const uint8_t rx[GO_RX_LEN])
{
    uint16_t crc_recv;
    uint16_t crc_calc;

    crc_recv = (uint16_t)rx[14] | ((uint16_t)rx[15] << 8);
    crc_calc = crc_ccitt(0, rx, 14);

    return crc_recv == crc_calc;
}

static int go_send_prebuilt(const uint8_t tx[GO_TX_LEN],
                            uint8_t *rx,
                            int rx_size,
                            int timeout_us,
                            int verbose)
{
    int n;
    int rx_len;

    if (go_uart_init() != 0)
    {
        return -1;
    }

    if (verbose)
    {
        rt_kprintf("send raw frame:\r\n");
        dump_hex(tx, GO_TX_LEN);
    }

    uart2_raw_clear_rx();

    rs485_tx_mode();

    n = uart2_raw_write(tx, GO_TX_LEN);
    if (n != GO_TX_LEN)
    {
        rt_kprintf("raw write failed, n=%d\r\n", n);
        rs485_rx_mode();
        return -1;
    }

    rs485_rx_mode();

    rx_len = uart2_raw_read(rx, rx_size, timeout_us);

    if (verbose)
    {
        rt_kprintf("rx_len=%d\r\n", rx_len);
        if (rx_len > 0)
        {
            dump_hex(rx, rx_len);
        }
    }

    return rx_len;
}
/*
 * Usage:
 *   go_nudge
 *   go_nudge 1 1 20 20
 *   go_nudge 1 -1 20 20
 *
 * Args:
 *   argv[1] = id, default 1
 *   argv[2] = dir, +1 or -1, default +1
 *   argv[3] = torque_raw, default 20
 *             actual torque command = torque_raw / 256 Nm
 *             20 -> about 0.078 Nm
 *   argv[4] = pulse_count, default 20
 *             each pulse interval approx 1ms
 */
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
static int go_nudge(int argc, char **argv)
{
    uint8_t tx[GO_TX_LEN];
    uint8_t rx[32];

    uint8_t id = 1;
    int dir = 1;
    int torque_raw_abs = 20;
    int pulse_count = 20;

    int tor_raw;
    int i;

    int ok = 0;
    int len_err = 0;
    int crc_err = 0;

    if (argc >= 2)
    {
        id = (uint8_t)atoi(argv[1]);
    }

    if (argc >= 3)
    {
        dir = atoi(argv[2]);
    }

    if (argc >= 4)
    {
        torque_raw_abs = atoi(argv[3]);
    }

    if (argc >= 5)
    {
        pulse_count = atoi(argv[4]);
    }

    if (id > 14)
    {
        rt_kprintf("invalid id, use 0~14\r\n");
        return -1;
    }

    if (dir >= 0)
    {
        dir = 1;
    }
    else
    {
        dir = -1;
    }

    /*
     * Tonight safety limit.
     * 80 / 256 = 0.3125 Nm command.
     */
    if (torque_raw_abs < 1)
    {
        torque_raw_abs = 1;
    }

    if (torque_raw_abs > 80)
    {
        rt_kprintf("torque_raw too large for first test, limit to 80\r\n");
        torque_raw_abs = 80;
    }

    if (pulse_count < 1)
    {
        pulse_count = 1;
    }

    if (pulse_count > 100)
    {
        pulse_count = 100;
    }

    tor_raw = dir * torque_raw_abs;

    rt_kprintf("go_nudge: id=%d dir=%d torque_raw=%d pulse_count=%d\r\n",
               id, dir, tor_raw, pulse_count);
    rt_kprintf("actual torque command approx %.4f Nm\r\n",
               (float)tor_raw / 256.0f);

    /*
     * 1. Tiny torque pulse.
     * mode=1, Kp=0, Kd=0, speed=0, position=0.
     */
    build_raw_cmd(id, 1, (int16_t)tor_raw, 0, 0, 0, 0, tx);

    for (i = 0; i < pulse_count; i++)
    {
        int rx_len;

        rt_memset(rx, 0, sizeof(rx));

        rx_len = go_send_prebuilt(tx, rx, GO_RX_LEN, 500, 0);

        if (rx_len != GO_RX_LEN)
        {
            len_err++;
        }
        else if (!feedback_crc_ok(rx))
        {
            crc_err++;
        }
        else
        {
            ok++;
        }

        rt_hw_us_delay(1000);
    }

    /*
     * 2. Immediately send zero torque several times.
     */
    build_zero_cmd(id, 1, tx);

    for (i = 0; i < 50; i++)
    {
        go_send_prebuilt(tx, rx, GO_RX_LEN, 500, 0);
        rt_hw_us_delay(1000);
    }

    rt_kprintf("go_nudge done:\r\n");
    rt_kprintf("  pulse_ok=%d\r\n", ok);
    rt_kprintf("  len_err=%d\r\n", len_err);
    rt_kprintf("  crc_err=%d\r\n", crc_err);

    rt_kprintf("last feedback:\r\n");
    dump_hex(rx, GO_RX_LEN);
    parse_feedback(rx);

    return 0;
}
MSH_CMD_EXPORT(go_nudge, go_nudge);
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

    if (g_verbose)
    {
        rt_kprintf("send %d bytes, id=%d mode=%d:\r\n", GO_TX_LEN, id, mode);
        dump_hex(tx, GO_TX_LEN);
    }

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

    if (g_verbose)
    {
        rt_kprintf("rx_len=%d\r\n", rx_len);

        if (rx_len > 0)
        {
            dump_hex(rx, rx_len);
        }
    }

    return rx_len;
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
    int count = 1;
    int i;

    int ok_count = 0;
    int len_err_count = 0;
    int header_err_count = 0;
    int crc_err_count = 0;

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
        count = atoi(argv[3]);
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

    if (count < 1 || count > 10000)
    {
        rt_kprintf("invalid count, use 1~10000\r\n");
        return -1;
    }

    /*
     * Single test: print full frame.
     * Multi test: quiet mode, only print statistics.
     */
    g_verbose = (count == 1) ? 1 : 0;

    for (i = 0; i < count; i++)
    {
        int rx_len;
        uint16_t crc_recv;
        uint16_t crc_calc;

        rt_memset(rx, 0, sizeof(rx));

        rx_len = go_send_recv(id, mode, rx, GO_RX_LEN, 500);
        /*
         * Avoid hammering the RS485 bus continuously during stress test.
         * 200us is still far faster than 40Hz control.
         */
        if (count > 1)
        {
            rt_hw_us_delay(200);
        }
        if (rx_len != GO_RX_LEN)
        {
            len_err_count++;
            continue;
        }

        if (!((rx[0] == 0xFD && rx[1] == 0xEE) ||
              (rx[0] == 0xFE && rx[1] == 0xEE)))
        {
            header_err_count++;
            continue;
        }

        crc_recv = (uint16_t)rx[14] | ((uint16_t)rx[15] << 8);
        crc_calc = crc_ccitt(0, rx, 14);

        if (crc_recv != crc_calc)
        {
            crc_err_count++;
            continue;
        }

        ok_count++;
    }

    g_verbose = 1;

    rt_kprintf("go_ping stress result:\r\n");
    rt_kprintf("  id=%d mode=%d count=%d\r\n", id, mode, count);
    rt_kprintf("  ok=%d\r\n", ok_count);
    rt_kprintf("  len_err=%d\r\n", len_err_count);
    rt_kprintf("  header_err=%d\r\n", header_err_count);
    rt_kprintf("  crc_err=%d\r\n", crc_err_count);

    if (count == 1 && ok_count == 1)
    {
        parse_feedback(rx);
    }

    return (ok_count == count) ? 0 : -1;
}
MSH_CMD_EXPORT(go_ping, go_ping);

