#ifndef PWM_OUTPUT_H
#define PWM_OUTPUT_H

#include <stdint.h>

#define PWM_OUTPUT_CLK_DIV 1 // Clock divider for main clock
#define PWM_OUTPUT_PERIOD 60000 // 2.5 kHz PWM frequency with 150 MHz clock and clkdiv=1

void pwm_output_init(uint8_t *pwm_pins, uint8_t length);

void pwm_output_set_duty_cycle(uint8_t *pwm_pins, uint8_t index, uint16_t duty_cycle);

/*
 * Map a physical value to a duty cycle and drive the channel (inverse of
 * pwm_input_get_value):
 *   duty_fraction = (value - offset) / scale + 0.5   (clamped to [0, 1])
 * so `offset` maps to 50% duty and `scale` sets the full-range span.
 */
void pwm_output_set_value(uint8_t *pwm_pins, uint8_t index, float value, float offset, float scale);

#endif // PWM_OUTPUT_H