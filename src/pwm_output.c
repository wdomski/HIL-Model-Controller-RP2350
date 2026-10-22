#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "hardware/pwm.h"
#include "hardware/gpio.h"

#include "pwm_output.h"

void pwm_output_init(uint8_t *pwm_pins, uint8_t length)
{
  for (int i = 0; i < length; ++i)
  {
    gpio_set_function(pwm_pins[i], GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(pwm_pins[i]);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config, PWM_OUTPUT_CLK_DIV); // frequency = 150 MHz / 1 = 150 MHz
    pwm_config_set_wrap(&config, PWM_OUTPUT_PERIOD - 1);    // period = (wrap + 1) / frequency
    pwm_init(slice_num, &config, false);
    pwm_set_gpio_level(pwm_pins[i], PWM_OUTPUT_PERIOD / 2); // duty cycle = level / (wrap + 1)
    pwm_set_enabled(slice_num, true);
  }
}

void pwm_output_set_duty_cycle(uint8_t *pwm_pins, uint8_t index, uint16_t duty_cycle)
{
  // make sure to always output some signal, even if the duty cycle is 0 or 100%
  // higher values are put in order to get the edges detectable by the receiver part
  if (duty_cycle == 0)
  {
    duty_cycle = 4;
  }
  else if (duty_cycle >= PWM_OUTPUT_PERIOD)
  {
    duty_cycle = PWM_OUTPUT_PERIOD - 8;
  }
  pwm_set_gpio_level(pwm_pins[index], duty_cycle);
}

void pwm_output_set_value(uint8_t *pwm_pins, uint8_t index, float value, float offset, float scale)
{
  float d = (scale != 0.0f) ? ((value - offset) / scale + 0.5f) : 0.5f;
  if (d < 0.0f) d = 0.0f;
  if (d > 1.0f) d = 1.0f;
  pwm_output_set_duty_cycle(pwm_pins, index, (uint16_t)(d * (float)PWM_OUTPUT_PERIOD));
}