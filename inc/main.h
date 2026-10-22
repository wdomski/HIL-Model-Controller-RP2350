#ifndef MAIN_HEADER
#define MAIN_HEADER

#include "pico/sync.h"

#include <stdint.h>

#include "circ_buff/circ_buff.h"

extern uint8_t buffer_uart_rx[] __attribute__((aligned(2048)));
extern uint8_t buffer_uart_tx[] __attribute__((aligned(2048)));
extern uint8_t buffer[] __attribute__((aligned(2048)));

#endif