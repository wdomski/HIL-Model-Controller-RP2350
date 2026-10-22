#include <stdio.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/pio.h"

#include "pico/stdlib.h"

#include "hw_test.h"
#include "led.h"
#include "pwm_output.h"
#include "pwm_input.h"
#include "serial.h"
#include "eprintfs/eprintfs.h"

float test_pwm_output_duties[] = {0.0f, 12.5f, 25.0f, 37.5f, 50.0f, 62.5f, 75.0f, 87.5f, 100.0f};
uint8_t test_pwm_output_index = 0;

static uint8_t buffer[1024] __attribute__((aligned(2048)));

/* set when anything was drained from the link during the current
 * hw_test_routine() rotation; cleared at the start of each rotation */
static uint8_t serial_link_data_seen;

/*
 * Poll the uart1 link once and drain whatever arrived as a complete burst.
 *
 * NOTE: circ_buff_new_data_amount() applies idle-line framing
 * (serial_link.rx_idle_us): the first poll after bytes land only latches the
 * new DMA position and returns 0; data is reported by a LATER poll that sees
 * the position stable for >= rx_idle_us. This must therefore be called
 * frequently (well under the peer's burst period), otherwise the received
 * data is never released (transfer_count_uart stays 0).
 */
static void hw_test_service_serial_link(void)
{
  uint32_t transfer_count_uart = circ_buff_new_data_amount(&serial_link.cb_rx);

  // handle UART
  if (transfer_count_uart > 0)
  {
    // reset number of transfers for UART RX in DMA circular mode
    circ_buff_hw_reset_transfer_count(serial_link.cb_rx.hw);
    circ_buff_read(&serial_link.cb_rx, buffer, transfer_count_uart);
    printf("Received (%lu): %s\n", (unsigned long)transfer_count_uart, buffer);
    serial_link_data_seen = 1;
  }
}

/* Sleep `ms` in short slices while servicing the link, so bursts received
 * between test iterations still pass the rx idle detection and get read. */
static void hw_test_wait_with_link(uint32_t ms)
{
  while (ms > 0)
  {
    uint32_t slice = (ms > 10) ? 10 : ms;
    sleep_ms(slice);
    ms -= slice;
    hw_test_service_serial_link();
  }
}

void hw_test_routine_led_flash(void)
{
  for(int i = 0; i < 2; ++i)
  {
    led_all_on(led_pins, 4);
    hw_test_wait_with_link(500);
    led_all_off(led_pins, 4);
    hw_test_wait_with_link(500);
  }
}

void hw_test_routine(uint8_t *info)
{
  // Test code for the model
  printf("Running hardware test routine...\n");
  uint32_t serial_link_counter = 0;

  for(;;)
  {
    serial_link_data_seen = 0;

    // set PWM duty cycle to 25% for all outputs
    printf("%s PWM Output %d%%\n", info, (int)(test_pwm_output_duties[test_pwm_output_index]));
    for (int i = 0; i < 8; ++i)
    {
      pwm_output_set_duty_cycle(pwm_output_pins, i, 
        test_pwm_output_duties[test_pwm_output_index] * PWM_OUTPUT_PERIOD / 100.0f); 
    }
    ++test_pwm_output_index;
    test_pwm_output_index %= sizeof(test_pwm_output_duties) / sizeof(test_pwm_output_duties[0]);

    // read PWM inputs and print values
    printf("PWM Inputs: ");
    for (int i = 0; i < 8; ++i)
    {
      // In a real test, we would read the actual PWM input values here
      // For this example, we'll just print a placeholder value
      uint32_t duty_cycle = pwm_input_get_duty_cycle(pwm_input_pins, i);
      duty_cycle = (duty_cycle * 1000) / PWM_OUTPUT_PERIOD; // Convert to promile
      printf("%d: %03lu, ", i, (unsigned long)duty_cycle);
    }
    printf("\n");

    hw_test_service_serial_link();

    uint32_t len_out = esprintf((char *)buffer, "From %s: %5d\n", info, (unsigned long)serial_link_counter++);
    send_with_repeat(&serial_link, buffer, len_out);

    hw_test_routine_led_flash();
    hw_test_wait_with_link(50);

    // report once per rotation if nothing arrived during the whole rotation
    if (!serial_link_data_seen)
    {
      printf("No data received on serial link\n");
    }

    pwm_input_rearm();
  }
}