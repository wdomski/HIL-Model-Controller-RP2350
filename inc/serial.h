#ifndef SERIAL_HEADER
#define SERIAL_HEADER

#include <stdint.h>

#include "hardware/uart.h"

#include "circ_buff/circ_buff.h"
#include "circ_buff/circ_buff_hw.h"

/*
 * One DMA-backed circular-buffer UART instance. Fill the configuration fields,
 * then call init_serial(). The same code drives any UART, so uart0 (console /
 * outside world) and uart1 (controller <-> model link) share this one path.
 */
typedef struct
{
    /* configuration */
    uart_inst_t *uart;
    uart_hw_t   *uart_hw;
    uint32_t     baudrate;
    uint32_t     tx_pin;
    uint32_t     rx_pin;
    uint32_t     dreq_rx;
    uint8_t     *buf_tx;
    uint8_t     *buf_rx;
    uint32_t     buf_tx_size;
    uint32_t     buf_rx_size;
    uint32_t     rx_ring_bits;   /* log2(buf_rx_size) for the RX DMA ring */
    uint32_t     rx_idle_us;     /* RX idle timeout */

    /* runtime state */
    int            dma_tx;
    int            dma_rx;
    circ_buff_hw_t cb_tx_hw;
    circ_buff_hw_t cb_rx_hw;
    circ_buff_t    cb_tx;
    circ_buff_t    cb_rx;
} serial_t;

extern serial_t serial_console;  /* uart0: outside world / telemetry */
extern serial_t serial_link;     /* uart1: controller <-> model link */

void init_serial(serial_t *s);

int send_with_repeat(serial_t *s, uint8_t *buffer, uint32_t len);

#endif