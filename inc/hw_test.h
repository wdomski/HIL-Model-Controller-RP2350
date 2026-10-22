#ifndef HW_TEST_H
#define HW_TEST_H

#include <stdint.h>

extern uint8_t pwm_input_pins[];
extern uint8_t pwm_output_pins[];
extern uint8_t led_pins[];

void hw_test_routine_led_flash(void);

void hw_test_routine(uint8_t *info);

#endif // HW_TEST_H