#ifndef CONFIG_HW_HEADER
#define CONFIG_HW_HEADER

#include "hardware/structs/uart.h"
#include "hardware/structs/adc.h"
#include "hardware/dma.h"

#define UART uart0
#define UART_HW uart0_hw
#define UART_BAUDRATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define UART_DREQ DREQ_UART0_RX

#define UART_BUFFER_TX_POWER_TWO    10
#define UART_BUFFER_RX_POWER_TWO    10

#define UART_BUFFER_TX_SIZE     (1 << UART_BUFFER_TX_POWER_TWO)
#define UART_BUFFER_RX_SIZE     (1 << UART_BUFFER_RX_POWER_TWO)

// uart1: dedicated controller <-> model link (model selection commands)
#define LINK_UART uart1
#define LINK_UART_HW uart1_hw
#define LINK_UART_BAUDRATE 115200
#define LINK_UART_TX_PIN 4
#define LINK_UART_RX_PIN 5
#define LINK_UART_DREQ DREQ_UART1_RX

#define LINK_BUFFER_TX_POWER_TWO    9
#define LINK_BUFFER_RX_POWER_TWO    8

#define LINK_BUFFER_TX_SIZE     (1 << LINK_BUFFER_TX_POWER_TWO)
#define LINK_BUFFER_RX_SIZE     (1 << LINK_BUFFER_RX_POWER_TWO)

#define ADC_HW adc_hw

#endif
