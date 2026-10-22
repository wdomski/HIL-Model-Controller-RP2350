#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "pwm_input.h"
#include "pwm_in.pio.h"

#define PWM_INPUT_MAX_CH 8
#define PWM_INPUT_PERIODS 2
#define PWM_INPUT_SAMPLES (2 * PWM_INPUT_PERIODS) // PIO pushes {pulse, period} per cycle

PIO pwm_pio[] = {pio0, pio1};
PIO pwm_pio_ch[] = {pio0, pio0, pio0, pio0, pio1, pio1, pio1, pio1};
uint16_t pwm_pio_offsets[] = {0, 0}; // offsets for each PIO program instance

// One DMA channel per SM, each draining the RX FIFO into its own sample buffer.
static int pwm_dma_chan[PWM_INPUT_MAX_CH];
static uint32_t pwm_dma_buf[PWM_INPUT_MAX_CH][PWM_INPUT_SAMPLES];
static uint8_t pwm_n_channels;

void pwm_input_init(uint8_t *pwm_pins, uint8_t length)
{
  // initialize PIO PWM input program
  // maximum 4 state machines per PIO

  // configure gpio
  for (int i = 0; i < length; ++i)
  {
    pio_gpio_init(pwm_pio_ch[i], pwm_pins[i]);
  }

  uint8_t pio_active_count = (length + 1) / 4; // round up to next whole number
  for (int i = 0; i < pio_active_count; ++i)
  {
    // pio_sm_claim(pwm_pio[i], 4); // claim 4 state machines for each PIO
    pwm_pio_offsets[i] = pio_add_program(pwm_pio[i], &pwm_in_program);
  }

  // initialize state machines for each PWM input pin
  for (int i = 0; i < length; ++i)
  {
    uint8_t sm_index = i % 4;   // determine which state machine within the PIO

    pio_sm_config c = pwm_in_program_get_default_config(pwm_pio_offsets[i / 4]);
    // set the 'jmp' pin
    sm_config_set_jmp_pin(&c, pwm_pins[i]);
    //set shift direction
    sm_config_set_in_shift(&c, false, false, 0);
    // init the pio sm with the config
    pio_sm_init(pwm_pio_ch[i], sm_index, pwm_pio_offsets[i / 4], &c);
    // enable the state machine
    pio_sm_set_enabled(pwm_pio_ch[i], sm_index, true);
  }

  // set up one DMA channel per SM to capture 2 samples without blocking
  pwm_n_channels = length;
  for (int i = 0; i < length; ++i)
  {
    uint8_t sm_index = i % 4;
    PIO pio = pwm_pio_ch[i];

    pwm_dma_chan[i] = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(pwm_dma_chan[i]);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm_index, false));
    dma_channel_configure(pwm_dma_chan[i], &c,
                          pwm_dma_buf[i],          // write: 2-word buffer
                          &pio->rxf[sm_index],     // read: RX FIFO
                          PWM_INPUT_SAMPLES,       // capture complete PWM periods
                          true);                   // start now
  }
}

// Re-arm all channels to capture a fresh pair of samples. Call once per model
// cycle after all inputs have been read; the new samples land before the next.
void pwm_input_rearm(void)
{
  for (uint8_t i = 0; i < pwm_n_channels; ++i)
  {
    dma_channel_abort(pwm_dma_chan[i]);
    pio_sm_clear_fifos(pwm_pio_ch[i], i % 4); // drop stale samples
    dma_channel_set_write_addr(pwm_dma_chan[i], pwm_dma_buf[i], false);
    dma_channel_set_trans_count(pwm_dma_chan[i], PWM_INPUT_SAMPLES, true);
  }
}

// Average complete PWM periods from the last DMA capture (non-blocking).
static void pwm_input_measure(uint8_t index, uint32_t *period, uint32_t *duty)
{
  uint64_t period_sum = 0;
  uint64_t duty_sum = 0;
  for (uint8_t sample = 0; sample < PWM_INPUT_PERIODS; ++sample)
  {
    uint32_t t1 = 0xFFFFFFFFu - pwm_dma_buf[index][2 * sample];
    uint32_t t2 = 0xFFFFFFFFu - pwm_dma_buf[index][2 * sample + 1];
    period_sum += (t1 > t2) ? t1 : t2;
    duty_sum += (t1 > t2) ? t2 : t1;
  }
  *period = (uint32_t)(2u * period_sum / PWM_INPUT_PERIODS);
  *duty = (uint32_t)(2u * duty_sum / PWM_INPUT_PERIODS);
}

uint32_t pwm_input_get_duty_cycle(uint8_t *pwm_pins, uint8_t index)
{
  (void)pwm_pins;
  uint32_t period, duty;
  pwm_input_measure(index, &period, &duty);
  return duty;
}

float pwm_input_get_duty_fraction(uint8_t *pwm_pins, uint8_t index)
{
  (void)pwm_pins;
  uint32_t period, duty;
  pwm_input_measure(index, &period, &duty);
  return (period > 0) ? ((float)duty / (float)period) : 0.5f;
}

float pwm_input_get_value(uint8_t *pwm_pins, uint8_t index, float offset, float scale)
{
  float d = pwm_input_get_duty_fraction(pwm_pins, index);
  return (d - 0.5f) * scale + offset;
}