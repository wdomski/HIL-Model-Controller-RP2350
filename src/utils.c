#include "pico/stdlib.h"

#include "hardware/clocks.h"

#include "utils.h"

void set_clock_speed_khz(uint32_t khz)
{
  // set clock speed to 150 MHz
  uint32_t ret;
  uint32_t actual_freq_khz = clock_get_hz(clk_sys) / 1000;
  ret = set_sys_clock_khz(khz, true);
  if (!ret)
  {
    for (;;)
    {
      tight_loop_contents();
    }
  }

  // validate the clock speed was set correctly
  actual_freq_khz = clock_get_hz(clk_sys) / 1000;
  if (actual_freq_khz != khz)
  {
    for (;;)
    {
      tight_loop_contents();
    }
  }
}

