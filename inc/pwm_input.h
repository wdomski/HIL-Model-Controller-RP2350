#ifndef PWM_INPUT_H
#define PWM_INPUT_H

#include <stdint.h>

void pwm_input_init(uint8_t *pwm_pins, uint8_t length);

/*
 * Re-arm the per-channel DMA capture. Call once per model cycle, after all
 * inputs have been read, so fresh PWM periods are ready for the next one.
 */
void pwm_input_rearm(void);

uint32_t pwm_input_get_duty_cycle(uint8_t *pwm_pins, uint8_t index);

/* Normalized duty cycle in [0, 1] (duty / period). */
float pwm_input_get_duty_fraction(uint8_t *pwm_pins, uint8_t index);

/*
 * Read a channel and map its duty cycle to a physical value:
 *   value = (duty_fraction - 0.5) * scale + offset
 * so 50% duty corresponds to `offset` and `scale` sets the full-range span.
 */
float pwm_input_get_value(uint8_t *pwm_pins, uint8_t index, float offset, float scale);

#endif // PWM_INPUT_H