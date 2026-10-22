#include "pico/stdlib.h"

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#include "config_hw.h"

#include "main.h"
#include "serial.h"

/* uart0 (console) buffers */
uint8_t buffer_uart_rx[UART_BUFFER_RX_SIZE] __attribute__((aligned(2048)));
uint8_t buffer_uart_tx[UART_BUFFER_TX_SIZE] __attribute__((aligned(2048)));
uint8_t buffer[1024] __attribute__((aligned(2048)));

/* uart1 (link) buffers */
static uint8_t buffer_link_rx[LINK_BUFFER_RX_SIZE] __attribute__((aligned(2048)));
static uint8_t buffer_link_tx[LINK_BUFFER_TX_SIZE] __attribute__((aligned(2048)));

serial_t serial_console = {
    .uart = UART, .uart_hw = UART_HW, .baudrate = UART_BAUDRATE,
    .tx_pin = UART_TX_PIN, .rx_pin = UART_RX_PIN, .dreq_rx = UART_DREQ,
    .buf_tx = buffer_uart_tx, .buf_rx = buffer_uart_rx,
    .buf_tx_size = sizeof(buffer_uart_tx), .buf_rx_size = sizeof(buffer_uart_rx),
    .rx_ring_bits = UART_BUFFER_RX_POWER_TWO, .rx_idle_us = 600,
};

serial_t serial_link = {
    .uart = LINK_UART, .uart_hw = LINK_UART_HW, .baudrate = LINK_UART_BAUDRATE,
    .tx_pin = LINK_UART_TX_PIN, .rx_pin = LINK_UART_RX_PIN, .dreq_rx = LINK_UART_DREQ,
    .buf_tx = buffer_link_tx, .buf_rx = buffer_link_rx,
    .buf_tx_size = sizeof(buffer_link_tx), .buf_rx_size = sizeof(buffer_link_rx),
    .rx_ring_bits = LINK_BUFFER_RX_POWER_TWO, .rx_idle_us = 600,
};

/* Shared DMA TX-complete ISR: service whichever instance's TX channel fired. */
#define SERIAL_MAX 2
static serial_t *serials[SERIAL_MAX];
static int serial_count = 0;

static void dma_serial_tx_completed(void)
{
    for (int i = 0; i < serial_count; i++)
    {
        if (dma_hw->ints0 & (1u << serials[i]->dma_tx))
        {
            circ_buff_hw_isr_tx_completed(serials[i]->cb_tx.hw, NULL);
        }
    }
}

void init_serial(serial_t *s)
{
    uart_init(s->uart, s->baudrate);

    gpio_set_function(s->tx_pin, GPIO_FUNC_UART);
    gpio_set_function(s->rx_pin, GPIO_FUNC_UART);

    // get junk byte
    if (uart_is_readable(s->uart))
    {
        uart_getc(s->uart);
    }

    s->dma_rx = dma_claim_unused_channel(true);
    s->dma_tx = dma_claim_unused_channel(true);

    // DMA receiver configuration (endless ring into buf_rx)
    dma_channel_config c = dma_channel_get_default_config(s->dma_rx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, s->rx_ring_bits);
    channel_config_set_dreq(&c, s->dreq_rx);

    dma_channel_set_read_addr(s->dma_rx, &s->uart_hw->dr, false);
    dma_channel_set_write_addr(s->dma_rx, s->buf_rx, false);
    dma_channel_set_trans_count(s->dma_rx, UINT32_MAX, false);
    dma_channel_set_config(s->dma_rx, &c, false);

    // DMA transmitter configuration
    c = dma_channel_get_default_config(s->dma_tx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, uart_get_dreq(s->uart, true));

    dma_channel_set_read_addr(s->dma_tx, s->buf_tx, false);
    dma_channel_set_write_addr(s->dma_tx, &s->uart_hw->dr, false);
    dma_channel_set_trans_count(s->dma_tx, 1, false);
    dma_channel_set_config(s->dma_tx, &c, false);

    dma_channel_set_irq0_enabled(s->dma_tx, true);

    // install the shared handler once, then register this instance
    if (serial_count == 0)
    {
        irq_set_exclusive_handler(DMA_IRQ_0, dma_serial_tx_completed);
        irq_set_enabled(DMA_IRQ_0, true);
    }
    serials[serial_count++] = s;

    circ_buff_hw_init(&s->cb_tx_hw, (void *)s->dma_tx, s->uart_hw);
    circ_buff_hw_init(&s->cb_rx_hw, (void *)s->dma_rx, s->uart_hw);

    circ_buff_hw_set_timer(&s->cb_rx_hw, NULL);

    circ_buff_init(&s->cb_tx, &s->cb_tx_hw, s->buf_tx, s->buf_tx_size, 0);
    circ_buff_init(&s->cb_rx, &s->cb_rx_hw, s->buf_rx, s->buf_rx_size, s->rx_idle_us);

    circ_buff_start_rx(&s->cb_rx);
    dma_channel_start(s->dma_rx);
}

int send_with_repeat(serial_t *s, uint8_t *buffer, uint32_t len)
{
    int repeat = 5;
    while (circ_buff_tx_busy(&s->cb_tx) && repeat > 0)
    {
        sleep_ms(1);
        repeat--;
    }
    if (repeat > 0)
    {
        circ_buff_start_tx_buff(&s->cb_tx, buffer, len);
    }
    return repeat;
}