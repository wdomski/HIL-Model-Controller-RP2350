#ifndef LED_H
#define LED_H

#include <stdbool.h>
#include <stdint.h>

/* Nonblocking heartbeat: 1000 ms idle / 200 ms active, both 50% duty. */
void led_heartbeat(uint8_t pin, uint32_t now_ms, bool active);

void led_init(uint8_t *led_pins, uint8_t length);

void led_all_off(uint8_t *led_pins, uint8_t length);

void led_all_on(uint8_t *led_pins, uint8_t length);

void led_all_toggle(uint8_t *led_pins, uint8_t length);

void led_off(uint8_t *led_pins, uint8_t index);

void led_on(uint8_t *led_pins, uint8_t index);

void led_toggle(uint8_t *led_pins, uint8_t index);

#endif // LED_H
