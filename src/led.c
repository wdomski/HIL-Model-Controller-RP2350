#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "hardware/gpio.h"

#include "led.h"

void led_init(uint8_t *led_pins, uint8_t length)
{
  for (int i = 0; i < length; ++i)
  {
    gpio_init(led_pins[i]);
    gpio_set_dir(led_pins[i], true); // set pin as output
    gpio_put(led_pins[i], false);    // turn off LED initially
  }
}

void led_all_off(uint8_t *led_pins, uint8_t length)
{
  for (int i = 0; i < length; ++i)
  {
    gpio_put(led_pins[i], false); // turn off LED
  }
}

void led_all_on(uint8_t *led_pins, uint8_t length)
{
  for (int i = 0; i < length; ++i)
  {
    gpio_put(led_pins[i], true); // turn on LED
  }
}

void led_all_toggle(uint8_t *led_pins, uint8_t length)
{
  for (int i = 0; i < length; ++i)
  {
    gpio_xor_mask(1u << led_pins[i]); // toggle LED
  }
}

void led_off(uint8_t *led_pins, uint8_t index)
{
  gpio_put(led_pins[index], false); // turn off specified LED
}

void led_on(uint8_t *led_pins, uint8_t index)
{
  gpio_put(led_pins[index], true); // turn on specified LED
}

void led_toggle(uint8_t *led_pins, uint8_t index)
{
  gpio_xor_mask(1u << led_pins[index]); // toggle specified LED
}