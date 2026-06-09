/*
 * GO-M8010-6 RS485 test for RT-Thread - soft control v5 robust startup/read recovery
 *
 * UART2:
 *   PA2 -> USART2_TX
 *   PA3 -> USART2_RX
 *
 * RS485 DIR:
 *   PH8 -> DE
 *   PH7 -> RE
 *
 * Unitree TTL-RS485 module logic:
 *   DE = 1: transmit enable
 *   DE = 0: transmit high-Z
 *   RE = 0: receive enable
 *   RE = 1: receive high-Z
 *
 * Commands:
 *   go_ping 1 1
 *   go_stop 1
 *   go_nudge 1 1 77 20
 *   go_torque_angle 1 1 20 2 500
 *   go_state 1
 *   go_hold 1 20 2 1000
 *   go_soft 1 10 1500 20 2 20
 *   go_assist 1 -3000 75 77 1200 800
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

#define GO_DEFAULT_RX_TIMEOUT_US    5000

static rt_device_t go_uart = RT_NULL;
static int g_verbose = 1;
static int g_uart_ready = 0;

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

static int32_t abs_i32(int32_t x)
{
    return (x >= 0) ? x : -x;
}

static void put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
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

/*
 * GO-M8010 feedback pos is q15 angle:
 *   rad = raw / 32768 * 2pi
 *
 * For small relative-angle tests, normalize delta to [-16384, 16384].
 */
static int32_t pos_delta_wrap_q15(int32_t now, int32_t start)
{
    int32_t d = now - start;

    while (d > 16384)
    {
        d -= 32768;
    }

    while (d < -16384)
    {
        d += 32768;
    }

    return d;
}

/* ---------- RS485 direction control ---------- */

static void rs485_dir_init(void)
{
    rt_pin_mode(RS485_DE_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(RS485_RE_PIN, PIN_MODE_OUTPUT);

    /* receive mode: DE=0, RE=0 */
    rt_pin_write(RS485_DE_PIN, PIN_LOW);
    rt_pin_write(RS485_RE_PIN, PIN_LOW);
}

static void rs485_tx_mode(void)
{
    /* transmit mode: DE=1, RE=1 */
    rt_pin_write(RS485_DE_PIN, PIN_HIGH);
    rt_pin_write(RS485_RE_PIN, PIN_HIGH);

    /* short setup time for RS485 driver */
    rt_hw_us_delay(2);
}

static void rs485_rx_mode(void)
{
    /* receive mode: DE=0, RE=0 */
    rt_pin_write(RS485_DE_PIN, PIN_LOW);
    rt_pin_write(RS485_RE_PIN, PIN_LOW);
}

/* ---------- Raw USART2 helper prototypes ---------- */

static void uart2_raw_clear_errors(void);
static void uart2_raw_clear_rx(void);
static void build_zero_cmd(uint8_t id, uint8_t mode, uint8_t tx[GO_TX_LEN]);
static int go_send_only_frame_no_clear(const uint8_t tx[GO_TX_LEN]);

/* ---------- UART2 init ---------- */

static int go_uart_init(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_err_t ret;

    if (g_uart_ready && go_uart != RT_NULL)
    {
        return 0;
    }

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
     * Do NOT use RT_DEVICE_FLAG_INT_RX here.
     * We read USART2->RDR directly.
     */
    ret = rt_device_open(go_uart, RT_DEVICE_FLAG_RDWR);
    if (ret != RT_EOK && ret != -RT_EBUSY)
    {
        rt_kprintf("uart open failed: %d\r\n", ret);
        return -1;
    }

    rs485_dir_init();
    uart2_raw_clear_errors();
    uart2_raw_clear_rx();

    g_uart_ready = 1;

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
    int guard = 512;

    uart2_raw_clear_errors();

    /*
     * Never use an unbounded RX drain loop here.
     * On a noisy RS485 bus, or when old motor replies keep arriving, RXNE may
     * stay asserted long enough to make an MSH command look dead.
     */
    while ((USART2->ISR & USART_ISR_RXNE_RXFNE) && guard > 0)
    {
        tmp = USART2->RDR;
        (void)tmp;
        guard--;
    }

    uart2_raw_clear_errors();
}

static void go_bus_recover(void)
{
    /*
     * Half-duplex RS485 recovery window.
     * A delayed motor reply may arrive after we have already drained RX once,
     * so use two quiet gaps instead of a single short clear.
     */
    rs485_rx_mode();
    uart2_raw_clear_rx();
    rt_thread_mdelay(5);
    uart2_raw_clear_rx();
    rt_thread_mdelay(5);
    uart2_raw_clear_rx();
}

static void go_send_zero_burst(uint8_t id, uint8_t mode, int burst_ms)
{
    uint8_t tx[GO_TX_LEN];
    int i;

    if (burst_ms < 1)
    {
        burst_ms = 1;
    }

    build_zero_cmd(id, mode, tx);

    for (i = 0; i < burst_ms; i++)
    {
        go_send_only_frame_no_clear(tx);

        if ((i % 5) == 0)
        {
            uart2_raw_clear_rx();
        }

        rt_thread_mdelay(1);
    }

    go_bus_recover();
}

static void go_motor_wakeup(uint8_t id)
{
    /*
     * Some runs fail only at the first read after reset/stop.
     * Send several zero-torque frames first, discard their replies, then read.
     * This does not apply active torque.
     */
    go_send_zero_burst(id, 1, 20);
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

/* ---------- GO-M8010 protocol builders ---------- */

static void build_zero_cmd(uint8_t id, uint8_t mode, uint8_t tx[GO_TX_LEN])
{
    uint16_t crc;

    rt_memset(tx, 0, GO_TX_LEN);

    tx[0] = 0xFE;
    tx[1] = 0xEE;
    tx[2] = (uint8_t)((id & 0x0F) | ((mode & 0x07) << 4));

    /*
     * tx[3]~tx[14]:
     * tor_des, spd_des, pos_des, k_pos, k_spd
     * all zero.
     */

    crc = crc_ccitt(0, tx, 15);
    put_u16_le(&tx[15], crc);
}

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

/* ---------- Feedback helpers ---------- */

static int feedback_valid(const uint8_t rx[GO_RX_LEN])
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

static int32_t feedback_get_pos_raw(const uint8_t rx[GO_RX_LEN])
{
    return get_i32_le(&rx[7]);
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

/* ---------- Low-level send helpers ---------- */

static int go_send_only_frame_no_clear(const uint8_t tx[GO_TX_LEN])
{
    int n;

    rs485_tx_mode();

    n = uart2_raw_write(tx, GO_TX_LEN);

    rs485_rx_mode();

    return n;
}

static int go_send_only_frame(const uint8_t tx[GO_TX_LEN])
{
    uart2_raw_clear_rx();
    return go_send_only_frame_no_clear(tx);
}

static int go_send_recv_frame(const uint8_t tx[GO_TX_LEN],
                              uint8_t *rx,
                              int rx_size,
                              int timeout_us,
                              int verbose)
{
    int n;
    int rx_len;

    uart2_raw_clear_rx();

    if (verbose)
    {
        rt_kprintf("send raw frame:\r\n");
        dump_hex(tx, GO_TX_LEN);
    }

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

static int go_send_recv(uint8_t id,
                        uint8_t mode,
                        uint8_t *rx,
                        int rx_size,
                        int timeout_us)
{
    uint8_t tx[GO_TX_LEN];

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

    return go_send_recv_frame(tx, rx, rx_size, timeout_us, g_verbose);
}

static void go_force_zero_stop(uint8_t id)
{
    uint8_t tx[GO_TX_LEN];
    int i;

    uart2_raw_clear_rx();

    /*
     * mode=1 zero torque for 200ms.
     * Use send-only frames here. The stop path must never wait for feedback.
     */
    build_zero_cmd(id, 1, tx);

    for (i = 0; i < 200; i++)
    {
        go_send_only_frame_no_clear(tx);

        if ((i % 20) == 0)
        {
            uart2_raw_clear_rx();
        }

        rt_thread_mdelay(1);
    }

    /*
     * mode=0 zero command for 50ms.
     */
    build_zero_cmd(id, 0, tx);

    for (i = 0; i < 50; i++)
    {
        go_send_only_frame_no_clear(tx);

        if ((i % 20) == 0)
        {
            uart2_raw_clear_rx();
        }

        rt_thread_mdelay(1);
    }

    go_bus_recover();
}

/* ---------- Reusable motor driver/control layer ---------- */

#define GO_POS_RAW_PER_REV          32768
#define GO_SOFT_Q15                 32768
#define GO_SOFT_CTRL_PERIOD_US      5000
#define GO_SOFT_PRINT_PERIOD_MS     50
#define GO_SOFT_HOLD_AFTER_MS       100

/*
 * v4 safety additions:
 * - q_ref lead clamp: do not let position reference run far ahead of feedback.
 * - near-target early stop: stop before the last sample overshoots the target.
 * - speed guard: emergency stop if feedback speed becomes too high.
 */
#define GO_SOFT_MAX_REF_LEAD_DEG    8
#define GO_SOFT_STOP_MARGIN_DEG     2
#define GO_SOFT_SPEED_RAW_LIMIT     800
#define GO_SOFT_FC_RAW_MAX           30
#define GO_SOFT_BREAKAWAY_DEG        1
#define GO_SOFT_FC_NEAR_TARGET_DEG   6

#define GO_SAFE_TORQUE_RAW_MAX      100
#define GO_SAFE_KP_RAW_MAX          300
#define GO_SAFE_KD_RAW_MAX          50
#define GO_SOFT_MAX_ANGLE_DEG       30
#define GO_SOFT_MAX_DURATION_MS     5000

typedef struct
{
    uint8_t id;
    uint8_t mode;
    uint8_t err;
    int8_t temp;

    int16_t torque_raw;
    int16_t speed_raw;
    int32_t pos_raw;

    rt_tick_t tick;
} go_motor_state_t;

static int clamp_int(int v, int min_v, int max_v)
{
    if (v < min_v)
    {
        return min_v;
    }

    if (v > max_v)
    {
        return max_v;
    }

    return v;
}

static int angle_deg_abs_to_raw_delta(int angle_deg_abs)
{
    return (angle_deg_abs * GO_POS_RAW_PER_REV) / 360;
}

static int smoothstep_q15(int x_q15)
{
    int64_t x;
    int64_t x2;
    int64_t y;

    x_q15 = clamp_int(x_q15, 0, GO_SOFT_Q15);

    x = x_q15;
    x2 = (x * x) / GO_SOFT_Q15;
    y = (x2 * (3 * GO_SOFT_Q15 - 2 * x)) / GO_SOFT_Q15;

    return (int)y;
}

static int torque_envelope_q15(int x_q15)
{
    int64_t x;
    int64_t y;

    x_q15 = clamp_int(x_q15, 0, GO_SOFT_Q15);

    x = x_q15;
    y = (4 * x * (GO_SOFT_Q15 - x)) / GO_SOFT_Q15;

    if (y < 0)
    {
        y = 0;
    }

    if (y > GO_SOFT_Q15)
    {
        y = GO_SOFT_Q15;
    }

    return (int)y;
}

static void feedback_to_state(const uint8_t rx[GO_RX_LEN], go_motor_state_t *state)
{
    if (state == RT_NULL)
    {
        return;
    }

    state->id         = rx[2] & 0x0F;
    state->mode       = (rx[2] >> 4) & 0x07;
    state->torque_raw = get_i16_le(&rx[3]);
    state->speed_raw  = get_i16_le(&rx[5]);
    state->pos_raw    = get_i32_le(&rx[7]);
    state->temp       = (int8_t)rx[11];
    state->err        = rx[12] & 0x07;
    state->tick       = rt_tick_get();
}

static void print_motor_state(const go_motor_state_t *state)
{
    if (state == RT_NULL)
    {
        return;
    }

    rt_kprintf("motor state:\r\n");
    rt_kprintf("  id=%d mode=%d temp=%d err=%d\r\n",
               state->id, state->mode, state->temp, state->err);
    rt_kprintf("  raw torque=%d speed=%d pos=%d\r\n",
               state->torque_raw, state->speed_raw, state->pos_raw);
}

/*
 * Read one valid feedback frame without applying active torque.
 * mode=1 zero command is used because your existing go_ping path already works in this mode.
 */
static int motor_read_state_raw(uint8_t id, go_motor_state_t *state, int verbose)
{
    uint8_t tx[GO_TX_LEN];
    uint8_t rx[32];
    int rx_len;

    if (id > 14)
    {
        return -1;
    }

    build_zero_cmd(id, 1, tx);
    rt_memset(rx, 0, sizeof(rx));

    rx_len = go_send_recv_frame(tx, rx, GO_RX_LEN, GO_DEFAULT_RX_TIMEOUT_US, verbose);
    if (rx_len != GO_RX_LEN)
    {
        go_bus_recover();
        return -2;
    }

    if (!feedback_valid(rx))
    {
        go_bus_recover();
        return -3;
    }

    feedback_to_state(rx, state);
    return 0;
}

static int motor_read_state_raw_retry(uint8_t id,
                                      go_motor_state_t *state,
                                      int verbose,
                                      int retries)
{
    int i;
    int ret = -9;

    if (retries < 1)
    {
        retries = 1;
    }

    for (i = 0; i < retries; i++)
    {
        /* Drain any delayed reply before each query. */
        go_bus_recover();

        ret = motor_read_state_raw(id, state, verbose && (i == 0));
        if (ret == 0)
        {
            return 0;
        }

        /* After a missed reply, give the motor/RS485 module enough quiet time. */
        rt_thread_mdelay(10);
        go_bus_recover();
    }

    return ret;
}

static int motor_read_state_after_wakeup(uint8_t id, go_motor_state_t *state, int retries)
{
    int ret;

    go_motor_wakeup(id);

    ret = motor_read_state_raw_retry(id, state, 0, retries);
    if (ret == 0)
    {
        return 0;
    }

    /* One more zero burst helps after a previous emergency stop or overshoot event. */
    go_motor_wakeup(id);
    ret = motor_read_state_raw_retry(id, state, 0, retries);

    return ret;
}

/*
 * Send one mixed-control frame and optionally parse the returned feedback.
 * This is the reusable low-level command interface for outer-loop control.
 */
static int motor_send_ctrl_raw(uint8_t id,
                               int16_t tor_raw,
                               int16_t spd_raw,
                               int32_t pos_raw,
                               int16_t kp_raw,
                               int16_t kd_raw,
                               go_motor_state_t *feedback,
                               int verbose)
{
    uint8_t tx[GO_TX_LEN];
    uint8_t rx[32];
    int rx_len;

    if (id > 14)
    {
        return -1;
    }

    build_raw_cmd(id, 1, tor_raw, spd_raw, pos_raw, kp_raw, kd_raw, tx);
    rt_memset(rx, 0, sizeof(rx));

    rx_len = go_send_recv_frame(tx, rx, GO_RX_LEN, GO_DEFAULT_RX_TIMEOUT_US, verbose);
    if (rx_len != GO_RX_LEN)
    {
        return -2;
    }

    if (!feedback_valid(rx))
    {
        return -3;
    }

    if (feedback != RT_NULL)
    {
        feedback_to_state(rx, feedback);
    }

    return 0;
}

static int motor_hold_raw_position(uint8_t id,
                                   int32_t pos_raw,
                                   int16_t kp_raw,
                                   int16_t kd_raw,
                                   int hold_ms)
{
    uint8_t tx[GO_TX_LEN];
    int loops;
    int i;

    if (hold_ms <= 0)
    {
        return 0;
    }

    loops = hold_ms * 1000 / GO_SOFT_CTRL_PERIOD_US;
    if (loops < 1)
    {
        loops = 1;
    }

    /*
     * Hold is deliberately send-only.
     * Waiting for feedback inside a hold loop can make MSH appear stuck when
     * one RS485 reply is missed. Use go_state before/after hold to observe.
     */
    build_raw_cmd(id, 1, 0, 0, pos_raw, kp_raw, kd_raw, tx);
    uart2_raw_clear_rx();

    for (i = 0; i < loops; i++)
    {
        int n = go_send_only_frame_no_clear(tx);
        if (n != GO_TX_LEN)
        {
            return -1;
        }

        if ((i % 10) == 0)
        {
            uart2_raw_clear_rx();
        }

        rt_thread_mdelay(GO_SOFT_CTRL_PERIOD_US / 1000);
    }

    uart2_raw_clear_rx();
    return 0;
}

/*
 * Outer-loop soft relative motion:
 *   1. read current rotor position
 *   2. generate q_ref = q0 + smoothstep(t) * delta
 *   3. generate tau_ff = 0 -> max -> 0 envelope
 *   4. send one mixed-control frame every GO_SOFT_CTRL_PERIOD_US
 *   5. stop on timeout, comm errors, or overshoot guard
 */
static int motor_soft_move_relative_raw(uint8_t id,
                                        int delta_deg,
                                        int duration_ms,
                                        int16_t kp_raw,
                                        int16_t kd_raw,
                                        int16_t tau_raw_abs,
                                        int lead_deg,
                                        int16_t fc_raw_abs)
{
    go_motor_state_t state;

    int dir;
    int angle_abs;
    int target_raw_delta;
    int guard_raw_delta;
    int lead_raw_delta;
    int stop_margin_raw;
    int breakaway_raw;
    int fc_near_target_raw;
    int32_t start_pos;
    int32_t target_pos;

    rt_tick_t tick_start;
    int elapsed_ms = 0;
    int last_print_ms = -GO_SOFT_PRINT_PERIOD_MS;

    int x_q15;
    int s_q15;
    int env_q15;
    int32_t q_profile;
    int32_t q_ref;
    int32_t ref_err;
    int16_t tau_ff;
    int16_t tau_env;
    int16_t tau_fc;
    int32_t fb_delta = 0;
    int32_t progress = 0;

    int comm_err = 0;
    int emergency = 0;
    int reached = 0;
    int ret;

    if (id > 14)
    {
        rt_kprintf("invalid id, use 0~14\r\n");
        return -1;
    }

    if (delta_deg == 0)
    {
        rt_kprintf("delta_deg cannot be 0\r\n");
        return -1;
    }

    dir = (delta_deg >= 0) ? 1 : -1;
    angle_abs = (delta_deg >= 0) ? delta_deg : -delta_deg;

    if (angle_abs > GO_SOFT_MAX_ANGLE_DEG)
    {
        rt_kprintf("angle too large, limit to %d deg\r\n", GO_SOFT_MAX_ANGLE_DEG);
        angle_abs = GO_SOFT_MAX_ANGLE_DEG;
        delta_deg = dir * angle_abs;
    }

    duration_ms = clamp_int(duration_ms, 100, GO_SOFT_MAX_DURATION_MS);
    kp_raw = (int16_t)clamp_int(kp_raw, 0, GO_SAFE_KP_RAW_MAX);
    kd_raw = (int16_t)clamp_int(kd_raw, 0, GO_SAFE_KD_RAW_MAX);
    tau_raw_abs = (int16_t)clamp_int(tau_raw_abs, 0, GO_SAFE_TORQUE_RAW_MAX);
    fc_raw_abs = (int16_t)clamp_int(fc_raw_abs, 0, GO_SOFT_FC_RAW_MAX);

    /* Runtime-tunable reference lead limit.
     * 3 deg was too conservative for this motor/load: it produced almost no motion.
     * Keep this within a safe early-test range.
     */
    lead_deg = clamp_int(lead_deg, 2, 20);

    target_raw_delta = angle_deg_abs_to_raw_delta(angle_abs);
    if (target_raw_delta < 1)
    {
        target_raw_delta = 1;
    }

    /* 5 deg extra guard prevents runaway during early tests. */
    guard_raw_delta = target_raw_delta + angle_deg_abs_to_raw_delta(5);
    lead_raw_delta = angle_deg_abs_to_raw_delta(lead_deg);
    stop_margin_raw = angle_deg_abs_to_raw_delta(GO_SOFT_STOP_MARGIN_DEG);
    breakaway_raw = angle_deg_abs_to_raw_delta(GO_SOFT_BREAKAWAY_DEG);
    fc_near_target_raw = angle_deg_abs_to_raw_delta(GO_SOFT_FC_NEAR_TARGET_DEG);

    if (lead_raw_delta < 1)
    {
        lead_raw_delta = 1;
    }

    if (stop_margin_raw < 1)
    {
        stop_margin_raw = 1;
    }

    ret = motor_read_state_after_wakeup(id, &state, 8);
    if (ret != 0)
    {
        rt_kprintf("failed to read initial state after wakeup, ret=%d\r\n", ret);
        go_force_zero_stop(id);
        return -2;
    }

    start_pos = state.pos_raw;
    target_pos = start_pos + dir * target_raw_delta;

    rt_kprintf("go_soft start:\r\n");
    rt_kprintf("  id=%d delta_deg=%d duration_ms=%d\r\n", id, delta_deg, duration_ms);
    rt_kprintf("  kp_raw=%d kd_raw=%d tau_raw_abs=%d fc_raw_abs=%d\r\n",
               kp_raw, kd_raw, tau_raw_abs, fc_raw_abs);
    rt_kprintf("  start_pos=%d target_pos=%d target_delta=%d guard_delta=%d\r\n",
               start_pos, target_pos, target_raw_delta, guard_raw_delta);
    rt_kprintf("  v7 safety: lead_deg=%d lead_limit=%d stop_margin=%d speed_limit=%d\r\n",
               lead_deg, lead_raw_delta, stop_margin_raw, GO_SOFT_SPEED_RAW_LIMIT);
    rt_kprintf("  v7 friction compensation: fc_raw_abs=%d breakaway=%d fc_near_target=%d\r\n",
               fc_raw_abs, breakaway_raw, fc_near_target_raw);

    tick_start = rt_tick_get();

    while (elapsed_ms < duration_ms)
    {
        rt_tick_t now_tick = rt_tick_get();

        elapsed_ms = (int)(((now_tick - tick_start) * 1000) / RT_TICK_PER_SECOND);
        if (elapsed_ms > duration_ms)
        {
            elapsed_ms = duration_ms;
        }

        x_q15 = (elapsed_ms * GO_SOFT_Q15) / duration_ms;
        s_q15 = smoothstep_q15(x_q15);
        env_q15 = torque_envelope_q15(x_q15);

        q_profile = start_pos + (int32_t)(((int64_t)(dir * target_raw_delta) * s_q15) / GO_SOFT_Q15);
        q_ref = q_profile;

        /*
         * v4: reference lead clamp.
         * If q_profile runs far ahead of real feedback, cap q_ref to feedback + lead_deg.
         * This avoids static-friction breakaway after a large hidden position error builds up.
         */
        ref_err = q_profile - state.pos_raw;
        if ((dir * ref_err) > lead_raw_delta)
        {
            q_ref = state.pos_raw + dir * lead_raw_delta;
        }
        else if ((dir * ref_err) < 0)
        {
            /* Actual rotor is already ahead of the profile; avoid pulling backward hard. */
            q_ref = state.pos_raw;
        }

        /*
         * v7: friction compensation.
         * tau_env is the original smooth 0 -> max -> 0 feedforward.
         * tau_fc is a Coulomb/static-friction bias: high before breakaway, then reduced
         * after motion starts, and disabled near the target. This avoids requiring a
         * large hidden position error before the rotor starts moving.
         */
        tau_env = (int16_t)((dir * (int32_t)tau_raw_abs * env_q15) / GO_SOFT_Q15);
        tau_fc = 0;

        if (fc_raw_abs > 0)
        {
            int32_t remaining = target_raw_delta - progress;

            if (remaining > fc_near_target_raw)
            {
                if (progress < breakaway_raw)
                {
                    tau_fc = (int16_t)(dir * fc_raw_abs);
                }
                else
                {
                    /* Running friction is lower than static friction. */
                    tau_fc = (int16_t)(dir * ((fc_raw_abs + 2) / 3));
                }
            }
        }

        tau_ff = (int16_t)(tau_env + tau_fc);
        tau_ff = (int16_t)clamp_int(tau_ff, -GO_SAFE_TORQUE_RAW_MAX, GO_SAFE_TORQUE_RAW_MAX);

        /* If actual rotor is already ahead of the profile, remove feedforward torque. */
        if ((dir * (state.pos_raw - q_profile)) > 0)
        {
            tau_ff = 0;
            tau_env = 0;
            tau_fc = 0;
        }

        ret = motor_send_ctrl_raw(id,
                                  tau_ff,
                                  0,
                                  q_ref,
                                  kp_raw,
                                  kd_raw,
                                  &state,
                                  0);
        if (ret != 0)
        {
            comm_err++;
            if (comm_err >= 3)
            {
                rt_kprintf("too many communication errors, ret=%d\r\n", ret);
                emergency = 1;
                break;
            }

            rt_thread_mdelay(GO_SOFT_CTRL_PERIOD_US / 1000);
            continue;
        }

        comm_err = 0;
        fb_delta = pos_delta_wrap_q15(state.pos_raw, start_pos);
        progress = dir * fb_delta;

        if (progress < -angle_deg_abs_to_raw_delta(2))
        {
            rt_kprintf("wrong direction guard triggered: fb_delta=%d\r\n", fb_delta);
            emergency = 1;
            break;
        }

        if (abs_i32(state.speed_raw) > GO_SOFT_SPEED_RAW_LIMIT)
        {
            rt_kprintf("speed guard triggered: speed_raw=%d limit=%d\r\n",
                       state.speed_raw, GO_SOFT_SPEED_RAW_LIMIT);
            emergency = 1;
            break;
        }

        if (progress >= (target_raw_delta - stop_margin_raw))
        {
            rt_kprintf("near-target stop: fb_delta=%d progress=%d target=%d margin=%d\r\n",
                       fb_delta, progress, target_raw_delta, stop_margin_raw);
            reached = 1;
            break;
        }

        if (abs_i32(fb_delta) > guard_raw_delta)
        {
            rt_kprintf("overshoot guard triggered: fb_delta=%d guard=%d\r\n",
                       fb_delta, guard_raw_delta);
            emergency = 1;
            break;
        }

        if ((elapsed_ms - last_print_ms) >= GO_SOFT_PRINT_PERIOD_MS)
        {
            last_print_ms = elapsed_ms;
            rt_kprintf("  t=%d q_profile=%d q_ref=%d fb_pos=%d fb_delta=%d spd=%d tau=%d env=%d fc=%d\r\n",
                       elapsed_ms, q_profile, q_ref, state.pos_raw, fb_delta, state.speed_raw,
                       tau_ff, tau_env, tau_fc);
        }

        rt_thread_mdelay(GO_SOFT_CTRL_PERIOD_US / 1000);
    }

    if (!emergency && !reached)
    {
        int16_t hold_kp = (kp_raw > 0) ? (kp_raw / 3) : 0;
        if (hold_kp < 1 && kp_raw > 0)
        {
            hold_kp = 1;
        }

        rt_kprintf("go_soft: low-gain hold at target for %d ms\r\n", GO_SOFT_HOLD_AFTER_MS);
        ret = motor_hold_raw_position(id, target_pos, hold_kp, kd_raw, GO_SOFT_HOLD_AFTER_MS);
        if (ret != 0)
        {
            rt_kprintf("go_soft hold stage send failed, ret=%d\r\n", ret);
            emergency = 1;
        }
    }
    else if (reached)
    {
        rt_kprintf("go_soft: reached/near target, skip target hold and zero stop\r\n");
    }

    go_force_zero_stop(id);

    rt_kprintf("go_soft done:\r\n");
    rt_kprintf("  emergency=%d reached=%d elapsed_ms=%d fb_delta=%d target_delta=%d\r\n",
               emergency, reached, elapsed_ms, fb_delta, target_raw_delta);

    return emergency ? -1 : 0;
}


/* ---------- MSH: go_wake ---------- */

static int go_wake(int argc, char **argv)
{
    uint8_t id = 1;

    if (argc >= 2)
    {
        id = (uint8_t)atoi(argv[1]);
    }

    if (id > 14)
    {
        rt_kprintf("invalid id, use 0~14\r\n");
        return -1;
    }

    if (go_uart_init() != 0)
    {
        return -1;
    }

    rt_kprintf("go_wake: id=%d zero-torque burst and bus recover\r\n", id);
    go_motor_wakeup(id);
    rt_kprintf("go_wake done\r\n");

    return 0;
}
MSH_CMD_EXPORT(go_wake, go_wake);

/* ---------- MSH: go_state / go_hold / go_soft ---------- */

static int go_state(int argc, char **argv)
{
    uint8_t id = 1;
    int count = 1;
    int i;
    int ok = 0;
    go_motor_state_t state;

    if (argc >= 2)
    {
        id = (uint8_t)atoi(argv[1]);
    }

    if (argc >= 3)
    {
        count = atoi(argv[2]);
    }

    if (id > 14)
    {
        rt_kprintf("invalid id, use 0~14\r\n");
        return -1;
    }

    count = clamp_int(count, 1, 100);

    if (go_uart_init() != 0)
    {
        return -1;
    }

    for (i = 0; i < count; i++)
    {
        int ret = motor_read_state_raw_retry(id, &state, count == 1 ? 1 : 0, 2);
        if (ret == 0)
        {
            ok++;
            rt_kprintf("[%d/%d] ", i + 1, count);
            print_motor_state(&state);
        }
        else
        {
            rt_kprintf("go_state read failed at %d/%d: ret=%d, stop batch read\r\n",
                       i + 1, count, ret);
            go_bus_recover();
            break;
        }

        if (count > 1)
        {
            rt_thread_mdelay(20);
        }
    }

    rt_kprintf("go_state result: ok=%d count=%d\r\n", ok, count);
    return (ok == count) ? 0 : -1;
}
MSH_CMD_EXPORT(go_state, go_state);

static int go_hold(int argc, char **argv)
{
    uint8_t id = 1;
    int kp_raw = 20;
    int kd_raw = 2;
    int hold_ms = 1000;
    go_motor_state_t state;
    int ret;

    if (argc >= 2)
    {
        id = (uint8_t)atoi(argv[1]);
    }

    if (argc >= 3)
    {
        kp_raw = atoi(argv[2]);
    }

    if (argc >= 4)
    {
        kd_raw = atoi(argv[3]);
    }

    if (argc >= 5)
    {
        hold_ms = atoi(argv[4]);
    }

    if (id > 14)
    {
        rt_kprintf("invalid id, use 0~14\r\n");
        return -1;
    }

    kp_raw = clamp_int(kp_raw, 0, GO_SAFE_KP_RAW_MAX);
    kd_raw = clamp_int(kd_raw, 0, GO_SAFE_KD_RAW_MAX);
    hold_ms = clamp_int(hold_ms, 50, 3000);

    if (go_uart_init() != 0)
    {
        return -1;
    }

    ret = motor_read_state_raw_retry(id, &state, 0, 3);
    if (ret != 0)
    {
        rt_kprintf("go_hold failed to read state, ret=%d\r\n", ret);
        go_force_zero_stop(id);
        return -1;
    }

    rt_kprintf("go_hold: id=%d pos=%d kp_raw=%d kd_raw=%d hold_ms=%d\r\n",
               id, state.pos_raw, kp_raw, kd_raw, hold_ms);

    ret = motor_hold_raw_position(id, state.pos_raw, (int16_t)kp_raw, (int16_t)kd_raw, hold_ms);
    go_force_zero_stop(id);

    rt_kprintf("go_hold done, ret=%d\r\n", ret);
    return ret;
}
MSH_CMD_EXPORT(go_hold, go_hold);

/*
 * Usage:
 *   go_soft
 *   go_soft 1 2 1500 20 2 0
 *   go_soft 1 10 1500 20 2 20
 *   go_soft 1 -10 1500 20 2 20
 *   go_soft 1 30 4500 20 3 12 8 10
 *
 * Args:
 *   argv[1] = id, default 1
 *   argv[2] = relative angle in deg, default 2, range [-30, 30], cannot be 0
 *   argv[3] = duration_ms, default 1500, range [100, 5000]
 *   argv[4] = kp_raw, default 20, safe limit 0~300
 *   argv[5] = kd_raw, default 2, safe limit 0~50
 *   argv[6] = tau_raw_abs, default 0, safe limit 0~100
 *   argv[7] = lead_deg, default GO_SOFT_MAX_REF_LEAD_DEG, range 2~20
 *   argv[8] = fc_raw_abs, friction compensation raw, default 0, safe limit 0~30
 */
static int go_soft(int argc, char **argv)
{
    uint8_t id = 1;
    int delta_deg = 2;
    int duration_ms = 1500;
    int kp_raw = 20;
    int kd_raw = 2;
    int tau_raw_abs = 0;
    int lead_deg = GO_SOFT_MAX_REF_LEAD_DEG;
    int fc_raw_abs = 0;

    if (argc >= 2)
    {
        id = (uint8_t)atoi(argv[1]);
    }

    if (argc >= 3)
    {
        delta_deg = atoi(argv[2]);
    }

    if (argc >= 4)
    {
        duration_ms = atoi(argv[3]);
    }

    if (argc >= 5)
    {
        kp_raw = atoi(argv[4]);
    }

    if (argc >= 6)
    {
        kd_raw = atoi(argv[5]);
    }

    if (argc >= 7)
    {
        tau_raw_abs = atoi(argv[6]);
    }

    if (argc >= 8)
    {
        lead_deg = atoi(argv[7]);
    }

    if (argc >= 9)
    {
        fc_raw_abs = atoi(argv[8]);
    }

    if (go_uart_init() != 0)
    {
        return -1;
    }

    return motor_soft_move_relative_raw(id,
                                        delta_deg,
                                        duration_ms,
                                        (int16_t)kp_raw,
                                        (int16_t)kd_raw,
                                        (int16_t)tau_raw_abs,
                                        lead_deg,
                                        (int16_t)fc_raw_abs);
}
MSH_CMD_EXPORT(go_soft, go_soft);


/* ---------- MSH: go_assist ---------- */

/*
 * Exoskeleton assist logic ported from the Python test:
 *   1. Read current rotor position q0_raw.
 *   2. q_goal_raw = q0_raw + delta_mrad / 6283 * 32768.
 *   3. During assist stage, keep sending:
 *        torque_raw = 0
 *        speed_raw  = 0
 *        pos_raw    = q_goal_raw
 *        kp_raw     = kp_raw argument
 *        kd_raw     = 0 by default
 *   4. Compute progress from feedback position.
 *   5. If progress >= stop_percent, or if target is crossed, switch to zero-torque mode.
 *
 * Command:
 *   go_assist <id> <delta_mrad> <stop_percent> <kp_raw> <timeout_ms> <zero_ms> [kd_raw]
 * Example:
 *   go_assist 1 -3000 75 77 1200 800
 *
 * Notes:
 *   - delta_mrad is milliradian. -3000 means -3.000 rad.
 *   - kp_raw=77 corresponds roughly to Python kp=0.06.
 */
#define GO_ASSIST_TWO_PI_MRAD           6283
#define GO_ASSIST_CTRL_PERIOD_MS        10
#define GO_ASSIST_DEFAULT_DELTA_MRAD    (-3000)
#define GO_ASSIST_DEFAULT_STOP_PERCENT  75
#define GO_ASSIST_DEFAULT_KP_RAW        77
#define GO_ASSIST_DEFAULT_KD_RAW        0
#define GO_ASSIST_DEFAULT_TIMEOUT_MS    1200
#define GO_ASSIST_DEFAULT_ZERO_MS       800
#define GO_ASSIST_MAX_ABS_DELTA_MRAD    6000
#define GO_ASSIST_MAX_TIMEOUT_MS        5000
#define GO_ASSIST_MAX_ZERO_MS           3000

enum
{
    GO_ASSIST_STOP_TIMEOUT = 0,
    GO_ASSIST_STOP_PROGRESS,
    GO_ASSIST_STOP_CROSSED,
    GO_ASSIST_STOP_COMM_ERROR,
};

/*
 * Do not use atoi()/strtol() here.  If the shell/newlib path is fragile on this
 * target, argument parsing can HardFault before the command prints anything.
 * This tiny parser is enough for signed decimal MSH arguments.
 */
static int parse_i32_arg(const char *s, int *out)
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

static const char *go_assist_stop_reason_str(int reason)
{
    switch (reason)
    {
    case GO_ASSIST_STOP_PROGRESS:
        return "progress_reached";
    case GO_ASSIST_STOP_CROSSED:
        return "crossed_goal";
    case GO_ASSIST_STOP_COMM_ERROR:
        return "comm_error";
    case GO_ASSIST_STOP_TIMEOUT:
    default:
        return "timeout";
    }
}

static int32_t mrad_to_pos_raw_delta(int delta_mrad)
{
    int64_t num;

    num = (int64_t)delta_mrad * GO_POS_RAW_PER_REV;

    /* Round to nearest integer raw count. */
    if (num >= 0)
    {
        num += GO_ASSIST_TWO_PI_MRAD / 2;
    }
    else
    {
        num -= GO_ASSIST_TWO_PI_MRAD / 2;
    }

    return (int32_t)(num / GO_ASSIST_TWO_PI_MRAD);
}

static void motor_zero_torque_mode_ms(uint8_t id, int zero_ms)
{
    uint8_t tx[GO_TX_LEN];
    int loops;
    int i;

    zero_ms = clamp_int(zero_ms, 1, GO_ASSIST_MAX_ZERO_MS);
    loops = zero_ms / GO_ASSIST_CTRL_PERIOD_MS;
    if (loops < 1)
    {
        loops = 1;
    }

    build_zero_cmd(id, 1, tx);
    uart2_raw_clear_rx();

    rt_kprintf("zero torque mode: id=%d zero_ms=%d loops=%d\r\n",
               id, zero_ms, loops);

    for (i = 0; i < loops; i++)
    {
        int n = go_send_only_frame_no_clear(tx);
        if (n != GO_TX_LEN)
        {
            rt_kprintf("zero torque send failed at i=%d, n=%d\r\n", i, n);
            break;
        }

        if ((i % 10) == 0)
        {
            uart2_raw_clear_rx();
        }

        rt_thread_mdelay(GO_ASSIST_CTRL_PERIOD_MS);
    }

    /* Keep this light; a long bus recovery at command end makes MSH look stuck. */
    uart2_raw_clear_rx();
}

static int motor_assist_until_progress_then_zero_raw(uint8_t id,
                                                     int delta_mrad,
                                                     int stop_percent,
                                                     int16_t kp_raw,
                                                     int16_t kd_raw,
                                                     int timeout_ms,
                                                     int zero_ms)
{
    go_motor_state_t state;
    int ret;
    int comm_err = 0;

    int32_t start_pos;
    int32_t goal_pos;
    int32_t delta_raw;
    int32_t total_raw;
    int direction;

    rt_tick_t tick_start;
    rt_tick_t tick_timeout;
    int loop_i = 0;
    int progress_percent = 0;
    int32_t err_raw = 0;
    int stop_reason = GO_ASSIST_STOP_TIMEOUT;

    if (id > 14)
    {
        rt_kprintf("invalid id, use 0~14\r\n");
        return -1;
    }

    if (delta_mrad == 0)
    {
        rt_kprintf("delta_mrad cannot be 0\r\n");
        return -1;
    }

    delta_mrad = clamp_int(delta_mrad,
                           -GO_ASSIST_MAX_ABS_DELTA_MRAD,
                           GO_ASSIST_MAX_ABS_DELTA_MRAD);
    stop_percent = clamp_int(stop_percent, 1, 100);
    kp_raw = (int16_t)clamp_int(kp_raw, 0, GO_SAFE_KP_RAW_MAX);
    kd_raw = (int16_t)clamp_int(kd_raw, 0, GO_SAFE_KD_RAW_MAX);
    timeout_ms = clamp_int(timeout_ms, 50, GO_ASSIST_MAX_TIMEOUT_MS);
    zero_ms = clamp_int(zero_ms, 50, GO_ASSIST_MAX_ZERO_MS);

    delta_raw = mrad_to_pos_raw_delta(delta_mrad);
    if (delta_raw == 0)
    {
        rt_kprintf("delta_mrad too small after raw conversion\r\n");
        return -1;
    }

    direction = (delta_raw >= 0) ? 1 : -1;
    total_raw = abs_i32(delta_raw);

    ret = motor_read_state_after_wakeup(id, &state, 8);
    if (ret != 0)
    {
        rt_kprintf("go_assist: failed to read initial state, ret=%d\r\n", ret);
        go_force_zero_stop(id);
        return -2;
    }

    start_pos = state.pos_raw;
    goal_pos = start_pos + delta_raw;

    rt_kprintf("go_assist start:\r\n");
    rt_kprintf("  id=%d delta_mrad=%d delta_raw=%d\r\n", id, delta_mrad, delta_raw);
    rt_kprintf("  start_pos=%d goal_pos=%d total_raw=%d\r\n", start_pos, goal_pos, total_raw);
    rt_kprintf("  stop_percent=%d kp_raw=%d kd_raw=%d timeout_ms=%d zero_ms=%d\r\n",
               stop_percent, kp_raw, kd_raw, timeout_ms, zero_ms);

    tick_start = rt_tick_get();
    tick_timeout = rt_tick_from_millisecond(timeout_ms);

    while ((rt_tick_get() - tick_start) < tick_timeout)
    {
        int64_t moved_raw_i64;
        int32_t moved_raw;

        ret = motor_send_ctrl_raw(id,
                                  0,
                                  0,
                                  goal_pos,
                                  kp_raw,
                                  kd_raw,
                                  &state,
                                  0);
        if (ret != 0)
        {
            comm_err++;
            if (comm_err >= 3)
            {
                rt_kprintf("go_assist: too many communication errors, ret=%d\r\n", ret);
                stop_reason = GO_ASSIST_STOP_COMM_ERROR;
                break;
            }

            rt_thread_mdelay(GO_ASSIST_CTRL_PERIOD_MS);
            continue;
        }

        comm_err = 0;

        moved_raw_i64 = ((int64_t)state.pos_raw - (int64_t)start_pos) * direction;
        if (moved_raw_i64 < 0)
        {
            moved_raw_i64 = 0;
        }
        if (moved_raw_i64 > 2147483647LL)
        {
            moved_raw_i64 = 2147483647LL;
        }

        moved_raw = (int32_t)moved_raw_i64;
        progress_percent = (int)(((int64_t)moved_raw * 100) / total_raw);
        err_raw = goal_pos - state.pos_raw;

        if ((loop_i % 5) == 0)
        {
            rt_kprintf("  i=%d q=%d goal=%d err=%d moved=%d progress=%d%% spd=%d temp=%d err=%d\r\n",
                       loop_i,
                       state.pos_raw,
                       goal_pos,
                       err_raw,
                       moved_raw,
                       progress_percent,
                       state.speed_raw,
                       state.temp,
                       state.err);
        }

        if (progress_percent >= stop_percent)
        {
            stop_reason = GO_ASSIST_STOP_PROGRESS;
            break;
        }

        if ((int64_t)direction * (int64_t)err_raw <= 0)
        {
            stop_reason = GO_ASSIST_STOP_CROSSED;
            break;
        }

        loop_i++;
        rt_thread_mdelay(GO_ASSIST_CTRL_PERIOD_MS);
    }

    rt_kprintf("go_assist stop:\r\n");
    rt_kprintf("  reason=%s loop_i=%d progress=%d%%\r\n",
               go_assist_stop_reason_str(stop_reason), loop_i, progress_percent);
    rt_kprintf("  final_pos=%d goal_pos=%d err=%d speed_raw=%d temp=%d err=%d\r\n",
               state.pos_raw, goal_pos, err_raw, state.speed_raw, state.temp, state.err);

    motor_zero_torque_mode_ms(id, zero_ms);

    rt_kprintf("go_assist done\r\n");

    return (stop_reason == GO_ASSIST_STOP_COMM_ERROR) ? -1 : 0;
}

static int go_assist(int argc, char **argv)
{
    uint8_t id = 1;
    int delta_mrad = GO_ASSIST_DEFAULT_DELTA_MRAD;
    int stop_percent = GO_ASSIST_DEFAULT_STOP_PERCENT;
    int kp_raw = GO_ASSIST_DEFAULT_KP_RAW;
    int kd_raw = GO_ASSIST_DEFAULT_KD_RAW;
    int timeout_ms = GO_ASSIST_DEFAULT_TIMEOUT_MS;
    int zero_ms = GO_ASSIST_DEFAULT_ZERO_MS;
    int tmp;

    rt_kprintf("go_assist entry: argc=%d\r\n", argc);

    if (argc >= 2 && parse_i32_arg(argv[1], &tmp) == 0)
    {
        id = (uint8_t)tmp;
    }
    if (argc >= 3 && parse_i32_arg(argv[2], &tmp) == 0)
    {
        delta_mrad = tmp;
    }
    if (argc >= 4 && parse_i32_arg(argv[3], &tmp) == 0)
    {
        stop_percent = tmp;
    }
    if (argc >= 5 && parse_i32_arg(argv[4], &tmp) == 0)
    {
        kp_raw = tmp;
    }
    if (argc >= 6 && parse_i32_arg(argv[5], &tmp) == 0)
    {
        timeout_ms = tmp;
    }
    if (argc >= 7 && parse_i32_arg(argv[6], &tmp) == 0)
    {
        zero_ms = tmp;
    }
    if (argc >= 8 && parse_i32_arg(argv[7], &tmp) == 0)
    {
        kd_raw = tmp;
    }

    rt_kprintf("go_assist parsed: id=%d delta_mrad=%d stop=%d kp_raw=%d timeout=%d zero=%d kd_raw=%d\r\n",
               id, delta_mrad, stop_percent, kp_raw, timeout_ms, zero_ms, kd_raw);

    if (go_uart_init() != 0)
    {
        return -1;
    }

    return motor_assist_until_progress_then_zero_raw(id,
                                                     delta_mrad,
                                                     stop_percent,
                                                     (int16_t)kp_raw,
                                                     (int16_t)kd_raw,
                                                     timeout_ms,
                                                     zero_ms);
}
MSH_CMD_EXPORT(go_assist, go_assist);

/* ---------- MSH: go_ping ---------- */

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

    g_verbose = (count == 1) ? 1 : 0;

    for (i = 0; i < count; i++)
    {
        int rx_len;

        rt_memset(rx, 0, sizeof(rx));

        rx_len = go_send_recv(id, mode, rx, GO_RX_LEN, GO_DEFAULT_RX_TIMEOUT_US);

        if (count > 1)
        {
            rt_hw_us_delay(500);
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

        if (!feedback_valid(rx))
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

/* ---------- MSH: go_stop ---------- */

static int go_stop(int argc, char **argv)
{
    uint8_t id = 1;

    if (argc >= 2)
    {
        id = (uint8_t)atoi(argv[1]);
    }

    if (id > 14)
    {
        rt_kprintf("invalid id, use 0~14\r\n");
        return -1;
    }

    rt_kprintf("go_stop: id=%d\r\n", id);

    if (go_uart_init() != 0)
    {
        return -1;
    }

    go_force_zero_stop(id);

    rt_kprintf("go_stop done\r\n");

    return 0;
}
MSH_CMD_EXPORT(go_stop, go_stop);

/* ---------- MSH: go_nudge ---------- */

/*
 * Usage:
 *   go_nudge
 *   go_nudge 1 1 30 10
 *   go_nudge 1 1 77 20
 *   go_nudge 1 -1 77 20
 *
 * Args:
 *   argv[1] = id, default 1
 *   argv[2] = dir, +1 or -1, default +1
 *   argv[3] = torque_raw_abs, default 77
 *             torque Nm approximately = raw / 256
 *   argv[4] = duration_ms, default 20
 */
static int go_nudge(int argc, char **argv)
{
    uint8_t tx[GO_TX_LEN];

    uint8_t id = 1;
    int dir = 1;
    int torque_raw_abs = 77;
    int duration_ms = 20;

    int tor_raw;
    int i;

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
        duration_ms = atoi(argv[4]);
    }

    if (id > 14)
    {
        rt_kprintf("invalid id, use 0~14\r\n");
        return -1;
    }

    dir = (dir >= 0) ? 1 : -1;

    if (torque_raw_abs < 1)
    {
        torque_raw_abs = 1;
    }

    /*
     * First-stage safety limit.
     * 100 / 256 ~= 0.39 Nm.
     */
    if (torque_raw_abs > 100)
    {
        rt_kprintf("torque_raw too large, limit to 100\r\n");
        torque_raw_abs = 100;
    }

    if (duration_ms < 1)
    {
        duration_ms = 1;
    }

    if (duration_ms > 100)
    {
        rt_kprintf("duration too long, limit to 100ms\r\n");
        duration_ms = 100;
    }

    tor_raw = dir * torque_raw_abs;

    rt_kprintf("go_nudge: id=%d dir=%d torque_raw=%d duration_ms=%d\r\n",
               id, dir, tor_raw, duration_ms);
    rt_kprintf("torque command raw=%d, Nm ~= raw/256\r\n", tor_raw);

    if (go_uart_init() != 0)
    {
        return -1;
    }

    /*
     * Phase 1: torque pulse.
     * mode=1, torque=tor_raw, speed=0, pos=0, Kp=0, Kd=0.
     */
    build_raw_cmd(id, 1, (int16_t)tor_raw, 0, 0, 0, 0, tx);

    for (i = 0; i < duration_ms; i++)
    {
        go_send_only_frame(tx);
        rt_thread_mdelay(1);
    }

    /*
     * Phase 2 + 3: force stop.
     */
    go_force_zero_stop(id);

    rt_kprintf("go_nudge done\r\n");

    return 0;
}
MSH_CMD_EXPORT(go_nudge, go_nudge);

/* ---------- MSH: go_torque_angle ---------- */

/*
 * Usage:
 *   go_torque_angle 1 1 20 2 500
 *   go_torque_angle 1 -1 20 2 500
 *
 * Args:
 *   argv[1] = id, default 1
 *   argv[2] = torque direction, +1 or -1, default +1
 *   argv[3] = torque_raw_abs, default 20
 *             torque Nm approximately = raw / 256
 *   argv[4] = angle_deg, default 2
 *   argv[5] = timeout_ms, default 500
 *
 * Logic:
 *   1. Read start_pos.
 *   2. Output small torque.
 *   3. Keep reading now_pos.
 *   4. delta = now_pos - start_pos, with wrap handling.
 *   5. Stop when abs(delta) reaches target_raw_delta.
 *   6. Always force zero stop at the end.
 */
static int go_torque_angle(int argc, char **argv)
{
    uint8_t tx[GO_TX_LEN];
    uint8_t rx[32];

    uint8_t id = 1;
    int dir = 1;
    int torque_raw_abs = 20;
    int angle_deg = 2;
    int timeout_ms = 500;

    int tor_raw;
    int target_raw_delta;

    int32_t start_pos = 0;
    int32_t now_pos = 0;
    int32_t delta = 0;

    int ok = 0;
    int len_err = 0;
    int crc_err = 0;
    int reached = 0;
    int rx_len;

    rt_tick_t tick_start;
    rt_tick_t tick_timeout;

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
        angle_deg = atoi(argv[4]);
    }

    if (argc >= 6)
    {
        timeout_ms = atoi(argv[5]);
    }

    if (id > 14)
    {
        rt_kprintf("invalid id, use 0~14\r\n");
        return -1;
    }

    dir = (dir >= 0) ? 1 : -1;

    if (torque_raw_abs < 1)
    {
        torque_raw_abs = 1;
    }

    if (torque_raw_abs > 100)
    {
        rt_kprintf("torque_raw too large, limit to 100\r\n");
        torque_raw_abs = 100;
    }

    if (angle_deg < 1)
    {
        angle_deg = 1;
    }

    if (angle_deg > 30)
    {
        rt_kprintf("angle too large for this test, limit to 30 deg\r\n");
        angle_deg = 30;
    }

    if (timeout_ms < 50)
    {
        timeout_ms = 50;
    }

    if (timeout_ms > 3000)
    {
        rt_kprintf("timeout too long, limit to 3000ms\r\n");
        timeout_ms = 3000;
    }

    tor_raw = dir * torque_raw_abs;

    /*
     * target raw delta = angle_deg / 360 * 32768
     */
    target_raw_delta = (angle_deg * 32768) / 360;

    rt_kprintf("go_torque_angle RELATIVE:\r\n");
    rt_kprintf("  id=%d dir=%d torque_raw=%d angle_deg=%d timeout_ms=%d\r\n",
               id, dir, tor_raw, angle_deg, timeout_ms);
    rt_kprintf("  target_raw_delta=%d\r\n", target_raw_delta);
    rt_kprintf("  stop rule: abs(now_pos - start_pos) >= target_raw_delta\r\n");

    if (go_uart_init() != 0)
    {
        return -1;
    }

    /*
     * Step 1: read start position.
     */
    build_zero_cmd(id, 1, tx);
    rt_memset(rx, 0, sizeof(rx));

    rx_len = go_send_recv_frame(tx, rx, GO_RX_LEN, GO_DEFAULT_RX_TIMEOUT_US, 0);

    if (rx_len != GO_RX_LEN || !feedback_valid(rx))
    {
        rt_kprintf("failed to read start position, rx_len=%d\r\n", rx_len);
        go_force_zero_stop(id);
        return -1;
    }

    start_pos = feedback_get_pos_raw(rx);
    now_pos = start_pos;

    rt_kprintf("  start_pos_raw=%d\r\n", start_pos);

    /*
     * Step 2: output torque while reading position feedback.
     * mode=1, torque=tor_raw, speed=0, pos=0, Kp=0, Kd=0.
     */
    build_raw_cmd(id, 1, (int16_t)tor_raw, 0, 0, 0, 0, tx);

    tick_start = rt_tick_get();
    tick_timeout = rt_tick_from_millisecond(timeout_ms);

    while ((rt_tick_get() - tick_start) < tick_timeout)
    {
        rt_memset(rx, 0, sizeof(rx));

        rx_len = go_send_recv_frame(tx, rx, GO_RX_LEN, GO_DEFAULT_RX_TIMEOUT_US, 0);

        if (rx_len != GO_RX_LEN)
        {
            len_err++;

            if (len_err >= 3)
            {
                rt_kprintf("too many len_err, force stop\r\n");
                break;
            }

            rt_thread_mdelay(1);
            continue;
        }

        if (!feedback_valid(rx))
        {
            crc_err++;

            if (crc_err >= 2)
            {
                rt_kprintf("too many crc/header err, force stop\r\n");
                break;
            }

            rt_thread_mdelay(1);
            continue;
        }

        ok++;

        now_pos = feedback_get_pos_raw(rx);
        delta = pos_delta_wrap_q15(now_pos, start_pos);

        /*
         * Relative-angle stop:
         * stop when absolute moved angle reaches target.
         */
        if (abs_i32(delta) >= target_raw_delta)
        {
            reached = 1;
            break;
        }

        rt_thread_mdelay(1);
    }

    /*
     * Step 3: always force stop.
     */
    go_force_zero_stop(id);

    rt_kprintf("go_torque_angle done:\r\n");
    rt_kprintf("  reached=%d\r\n", reached);
    rt_kprintf("  ok=%d len_err=%d crc_err=%d\r\n", ok, len_err, crc_err);
    rt_kprintf("  start_pos=%d now_pos=%d delta=%d abs_delta=%d target=%d\r\n",
               start_pos, now_pos, delta, abs_i32(delta), target_raw_delta);

    if (ok > 0 && feedback_valid(rx))
    {
        parse_feedback(rx);
    }

    return reached ? 0 : -1;
}
MSH_CMD_EXPORT(go_torque_angle, go_torque_angle);
/* ---------- MSH: go_assist_group ---------- */

#define GO_ASSIST_GROUP_MAX_MOTORS     6

#define GO_ASSIST_REASON_NONE          0
#define GO_ASSIST_REASON_PROGRESS      1
#define GO_ASSIST_REASON_CROSSED       2
#define GO_ASSIST_REASON_TIMEOUT       3
#define GO_ASSIST_REASON_COMM_ERR      4

typedef struct
{
    uint8_t id;

    int32_t delta_mrad;
    int stop_percent;

    int16_t kp_raw;
    int16_t kd_raw;

    uint32_t timeout_ms;
    uint32_t zero_ms;
} go_assist_motor_cfg_t;

typedef struct
{
    const char *name;
    const go_assist_motor_cfg_t *motors;
    int count;
} go_assist_group_cfg_t;

typedef struct
{
    const go_assist_motor_cfg_t *cfg;

    int32_t delta_raw;
    int32_t start_pos;
    int32_t goal_pos;
    int32_t total_raw;
    int32_t final_pos;

    int dir;
    int progress_percent;

    uint32_t start_ms;

    int active;
    int stop_reason;
    int comm_err;

    go_motor_state_t state;
} go_assist_motor_rt_t;

static uint32_t go_now_ms(void)
{
    return (uint32_t)(((uint64_t)rt_tick_get() * 1000ULL) / RT_TICK_PER_SECOND);
}

/*
 * mrad -> GO-M8010 pos raw:
 *   raw = rad / 6.2832 * 32768
 *   6283 mrad ~= 2pi rad
 */
static int32_t assist_mrad_to_pos_raw(int32_t mrad)
{
    return (int32_t)(((int64_t)mrad * 32768LL) / 6283LL);
}

static const char *assist_reason_str(int reason)
{
    switch (reason)
    {
    case GO_ASSIST_REASON_PROGRESS:
        return "progress";
    case GO_ASSIST_REASON_CROSSED:
        return "crossed";
    case GO_ASSIST_REASON_TIMEOUT:
        return "timeout";
    case GO_ASSIST_REASON_COMM_ERR:
        return "comm_err";
    default:
        return "none";
    }
}

/* ============================================================
 * 预设动作组
 * ============================================================
 */

/*
 * group 0:
 * 电机 1 和 3 同方向小角度测试
 * -300 mrad ~= -0.3 rad ~= -17.2 deg rotor angle
 */
static const go_assist_motor_cfg_t assist_group_0_motors[] =
{
    {
        .id = 1,
        .delta_mrad = -300,
        .stop_percent = 75,
        .kp_raw = 77,
        .kd_raw = 0,
        .timeout_ms = 500,
        .zero_ms = 300,
    },
    {
        .id = 3,
        .delta_mrad = -300,
        .stop_percent = 75,
        .kp_raw = 77,
        .kd_raw = 0,
        .timeout_ms = 500,
        .zero_ms = 300,
    },
};

/*
 * group 1:
 * 电机 1 和 3 反方向小角度测试
 */
static const go_assist_motor_cfg_t assist_group_1_motors[] =
{
    {
        .id = 1,
        .delta_mrad = -300,
        .stop_percent = 75,
        .kp_raw = 77,
        .kd_raw = 0,
        .timeout_ms = 500,
        .zero_ms = 300,
    },
    {
        .id = 3,
        .delta_mrad = 300,
        .stop_percent = 75,
        .kp_raw = 77,
        .kd_raw = 0,
        .timeout_ms = 500,
        .zero_ms = 300,
    },
};

/*
 * group 2:
 * 电机 1 和 3 正方向小角度测试
 */
static const go_assist_motor_cfg_t assist_group_2_motors[] =
{
    {
        .id = 1,
        .delta_mrad = 300,
        .stop_percent = 75,
        .kp_raw = 77,
        .kd_raw = 0,
        .timeout_ms = 500,
        .zero_ms = 300,
    },
    {
        .id = 3,
        .delta_mrad = 300,
        .stop_percent = 75,
        .kp_raw = 77,
        .kd_raw = 0,
        .timeout_ms = 500,
        .zero_ms = 300,
    },
};
/*
 * group 3:
 * 电机 0~5 同方向小角度测试
 * -300 mrad ~= -0.3 rad ~= -17.2 deg rotor angle
 */
static const go_assist_motor_cfg_t assist_group_3_motors[] =
{
    { .id = 0, .delta_mrad = -1000, .stop_percent = 75, .kp_raw = 77, .kd_raw = 0, .timeout_ms = 500, .zero_ms = 300 },
    { .id = 1, .delta_mrad = -1000, .stop_percent = 75, .kp_raw = 77, .kd_raw = 0, .timeout_ms = 500, .zero_ms = 300 },
    { .id = 2, .delta_mrad = -1000, .stop_percent = 75, .kp_raw = 77, .kd_raw = 0, .timeout_ms = 500, .zero_ms = 300 },
    { .id = 3, .delta_mrad = -1000, .stop_percent = 75, .kp_raw = 77, .kd_raw = 0, .timeout_ms = 500, .zero_ms = 300 },
    { .id = 4, .delta_mrad = -1000, .stop_percent = 75, .kp_raw = 77, .kd_raw = 0, .timeout_ms = 500, .zero_ms = 300 },
    { .id = 5, .delta_mrad = -1000, .stop_percent = 75, .kp_raw = 77, .kd_raw = 0, .timeout_ms = 500, .zero_ms = 300 },
};
static const go_assist_group_cfg_t assist_groups[] =
{
    {
        .name = "m1_m3_small_negative",
        .motors = assist_group_0_motors,
        .count = sizeof(assist_group_0_motors) / sizeof(assist_group_0_motors[0]),
    },
    {
        .name = "m1_m3_small_opposite",
        .motors = assist_group_1_motors,
        .count = sizeof(assist_group_1_motors) / sizeof(assist_group_1_motors[0]),
    },
    {
        .name = "m1_m3_small_positive",
        .motors = assist_group_2_motors,
        .count = sizeof(assist_group_2_motors) / sizeof(assist_group_2_motors[0]),
    },
    {
        .name = "m0_m1_m2_m3_m4_m5_small_negative",
        .motors = assist_group_3_motors,
        .count = sizeof(assist_group_3_motors) / sizeof(assist_group_3_motors[0]),
    },
};

static int go_assist_group_zero_all(go_assist_motor_rt_t *rt_items, int count)
{
    int i;
    int max_zero_ms = 0;
    uint32_t t0;
    go_motor_state_t fb;

    for (i = 0; i < count; i++)
    {
        if ((int)rt_items[i].cfg->zero_ms > max_zero_ms)
        {
            max_zero_ms = (int)rt_items[i].cfg->zero_ms;
        }
    }

    if (max_zero_ms <= 0)
    {
        max_zero_ms = 100;
    }

    rt_kprintf("group zero torque: max_zero_ms=%d\r\n", max_zero_ms);

    t0 = go_now_ms();

    while ((go_now_ms() - t0) < (uint32_t)max_zero_ms)
    {
        for (i = 0; i < count; i++)
        {
            if ((go_now_ms() - t0) < rt_items[i].cfg->zero_ms)
            {
                motor_send_ctrl_raw(rt_items[i].cfg->id,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    &fb,
                                    0);
            }
        }

        rt_thread_mdelay(10);
    }

    uart2_raw_clear_rx();
    return 0;
}

static int go_assist_run_group(const go_assist_motor_cfg_t *cfgs, int count)
{
    go_assist_motor_rt_t rt_items[GO_ASSIST_GROUP_MAX_MOTORS];

    int i;
    int active_count;
    int loop_i = 0;

    rt_kprintf("DBG A: run_group entry cfgs=%p count=%d max=%d\r\n",
               cfgs, count, GO_ASSIST_GROUP_MAX_MOTORS);

    if (cfgs == RT_NULL)
    {
        rt_kprintf("DBG B: null cfgs\r\n");
        return -1;
    }

    rt_kprintf("DBG C: cfgs not null\r\n");

    if (count < 1 || count > GO_ASSIST_GROUP_MAX_MOTORS)
    {
        rt_kprintf("DBG D: invalid count=%d\r\n", count);
        return -1;
    }

    rt_kprintf("DBG E: count ok\r\n");

    if (go_uart_init() != 0)
    {
        rt_kprintf("DBG F: uart init failed\r\n");
        return -1;
    }

    rt_kprintf("DBG G: uart init ok\r\n");

    rt_memset(rt_items, 0, sizeof(rt_items));

    rt_kprintf("DBG H: rt_items memset ok, size=%d\r\n", (int)sizeof(rt_items));

    rt_memset(rt_items, 0, sizeof(rt_items));

    /*
     * Step 1:
     * 先把所有电机初始位置读完，再进入统一启动循环。
     */
    for (i = 0; i < count; i++)
    {
        int ret;
        rt_kprintf("cfg[%d] addr=%p id=%d delta=%d stop=%d kp=%d kd=%d timeout=%d zero=%d\r\n",
                   i,
                   &cfgs[i],
                   cfgs[i].id,
                   cfgs[i].delta_mrad,
                   cfgs[i].stop_percent,
                   cfgs[i].kp_raw,
                   cfgs[i].kd_raw,
                   cfgs[i].timeout_ms,
                   cfgs[i].zero_ms);
        rt_items[i].cfg = &cfgs[i];

        rt_items[i].delta_raw = assist_mrad_to_pos_raw(cfgs[i].delta_mrad);

        if (rt_items[i].delta_raw == 0)
        {
            rt_kprintf("motor %d id=%d delta too small\r\n", i, cfgs[i].id);
            return -1;
        }

        rt_items[i].dir = (rt_items[i].delta_raw >= 0) ? 1 : -1;
        rt_items[i].total_raw = abs_i32(rt_items[i].delta_raw);

        if (cfgs[i].id > 14)
        {
            rt_kprintf("motor %d invalid id=%d\r\n", i, cfgs[i].id);
            return -1;
        }

        ret = motor_read_state_after_wakeup(cfgs[i].id, &rt_items[i].state, 8);
        if (ret != 0)
        {
            rt_kprintf("motor %d id=%d failed to read initial state, ret=%d\r\n",
                       i, cfgs[i].id, ret);

            go_assist_group_zero_all(rt_items, count);
            return -1;
        }

        rt_items[i].start_pos = rt_items[i].state.pos_raw;
        rt_items[i].goal_pos = rt_items[i].start_pos + rt_items[i].delta_raw;
        rt_items[i].final_pos = rt_items[i].start_pos;

        rt_items[i].active = 1;
        rt_items[i].stop_reason = GO_ASSIST_REASON_NONE;
        rt_items[i].comm_err = 0;
        rt_items[i].progress_percent = 0;

        rt_kprintf("group motor %d:\r\n", i);
        rt_kprintf("  id=%d delta_mrad=%d delta_raw=%d\r\n",
                   cfgs[i].id, cfgs[i].delta_mrad, rt_items[i].delta_raw);
        rt_kprintf("  start=%d goal=%d stop=%d%% kp=%d kd=%d timeout=%d zero=%d\r\n",
                   rt_items[i].start_pos,
                   rt_items[i].goal_pos,
                   cfgs[i].stop_percent,
                   cfgs[i].kp_raw,
                   cfgs[i].kd_raw,
                   cfgs[i].timeout_ms,
                   cfgs[i].zero_ms);
    }

    /*
     * Step 2:
     * 统一记录 start_ms，让多个电机尽可能同时开始。
     */
    {
        uint32_t start_ms = go_now_ms();

        for (i = 0; i < count; i++)
        {
            rt_items[i].start_ms = start_ms;
        }
    }

    active_count = count;

    rt_kprintf("go_assist_group movement start, count=%d\r\n", count);

    while (active_count > 0)
    {
        for (i = 0; i < count; i++)
        {
            const go_assist_motor_cfg_t *cfg;
            int ret;
            int32_t moved_raw;
            int32_t err_raw;
            int progress_percent;
            uint32_t elapsed_ms;

            if (!rt_items[i].active)
            {
                continue;
            }

            cfg = rt_items[i].cfg;

            ret = motor_send_ctrl_raw(cfg->id,
                                      0,
                                      0,
                                      rt_items[i].goal_pos,
                                      cfg->kp_raw,
                                      cfg->kd_raw,
                                      &rt_items[i].state,
                                      0);

            if (ret != 0)
            {
                rt_items[i].comm_err++;

                if (rt_items[i].comm_err >= 3)
                {
                    rt_items[i].active = 0;
                    rt_items[i].stop_reason = GO_ASSIST_REASON_COMM_ERR;
                    active_count--;

                    rt_kprintf("group motor %d id=%d stop: comm_err ret=%d\r\n",
                               i, cfg->id, ret);
                }

                continue;
            }

            rt_items[i].comm_err = 0;
            rt_items[i].final_pos = rt_items[i].state.pos_raw;

            moved_raw = (rt_items[i].state.pos_raw - rt_items[i].start_pos) * rt_items[i].dir;
            if (moved_raw < 0)
            {
                moved_raw = 0;
            }

            progress_percent = (int)(((int64_t)moved_raw * 100LL) / rt_items[i].total_raw);
            rt_items[i].progress_percent = progress_percent;

            err_raw = rt_items[i].goal_pos - rt_items[i].state.pos_raw;
            elapsed_ms = go_now_ms() - rt_items[i].start_ms;

            if ((loop_i % 5) == 0)
            {
                rt_kprintf("  m%d id=%d q=%d goal=%d progress=%d%% spd=%d temp=%d err=%d\r\n",
                           i,
                           cfg->id,
                           rt_items[i].state.pos_raw,
                           rt_items[i].goal_pos,
                           progress_percent,
                           rt_items[i].state.speed_raw,
                           rt_items[i].state.temp,
                           rt_items[i].state.err);
            }

            if (progress_percent >= cfg->stop_percent)
            {
                rt_items[i].active = 0;
                rt_items[i].stop_reason = GO_ASSIST_REASON_PROGRESS;
                active_count--;

                rt_kprintf("group motor %d id=%d stop: progress=%d%%\r\n",
                           i, cfg->id, progress_percent);
                continue;
            }

            if (rt_items[i].dir * err_raw <= 0)
            {
                rt_items[i].active = 0;
                rt_items[i].stop_reason = GO_ASSIST_REASON_CROSSED;
                active_count--;

                rt_kprintf("group motor %d id=%d stop: crossed goal\r\n",
                           i, cfg->id);
                continue;
            }

            if (elapsed_ms > cfg->timeout_ms)
            {
                rt_items[i].active = 0;
                rt_items[i].stop_reason = GO_ASSIST_REASON_TIMEOUT;
                active_count--;

                rt_kprintf("group motor %d id=%d stop: timeout\r\n",
                           i, cfg->id);
                continue;
            }
        }

        rt_thread_mdelay(10);
        loop_i++;
    }

    rt_kprintf("go_assist_group movement done, zero torque all\r\n");

    go_assist_group_zero_all(rt_items, count);

    rt_kprintf("go_assist_group summary:\r\n");

    for (i = 0; i < count; i++)
    {
        rt_kprintf("  m%d id=%d reason=%s progress=%d%% start=%d final=%d goal=%d spd=%d\r\n",
                   i,
                   rt_items[i].cfg->id,
                   assist_reason_str(rt_items[i].stop_reason),
                   rt_items[i].progress_percent,
                   rt_items[i].start_pos,
                   rt_items[i].final_pos,
                   rt_items[i].goal_pos,
                   rt_items[i].state.speed_raw);
    }

    rt_kprintf("go_assist_group done\r\n");

    return 0;
}

/*
 * Usage:
 *   go_assist_group
 *   go_assist_group 0
 *   go_assist_group 1
 *   go_assist_group 2
 *
 * group 0: motor 1 and 3 both -300 mrad
 * group 1: motor 1 -300 mrad, motor 3 +300 mrad
 * group 2: motor 1 and 3 both +300 mrad
 */
static int go_assist_group(int argc, char **argv)
{
    int group_id = 0;

    if (argc >= 2)
    {
        group_id = atoi(argv[1]);
    }

    if (group_id < 0 ||
        group_id >= (int)(sizeof(assist_groups) / sizeof(assist_groups[0])))
    {
        rt_kprintf("invalid group id=%d\r\n", group_id);
        rt_kprintf("available groups:\r\n");
        rt_kprintf("  0: m1_m3_small_negative\r\n");
        rt_kprintf("  1: m1_m3_small_opposite\r\n");
        rt_kprintf("  2: m1_m3_small_positive\r\n");
        return -1;
    }

    rt_kprintf("go_assist_group: id=%d name=%s count=%d\r\n",
               group_id,
               assist_groups[group_id].name,
               assist_groups[group_id].count);

    return go_assist_run_group(assist_groups[group_id].motors,
                               assist_groups[group_id].count);
}
MSH_CMD_EXPORT(go_assist_group, go_assist_group);
