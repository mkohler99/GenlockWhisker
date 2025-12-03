#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// Program name: vsync
// .wrap_target 0
// .wrap 9

static const uint16_t vsync_program_instructions[] = {
    0x2020, //  0: wait   1 pin, 0
    0x2000, //  1: wait   0 pin, 0
    0xa02b, //  2: mov    x, !null
    0x00c5, //  3: jmp    pin, 5
    0x0043, //  4: jmp    x--, 3
    0x00c7, //  5: jmp    pin, 7
    0x0008, //  6: jmp    8
    0x0045, //  7: jmp    x--, 5
    0xa0c1, //  8: mov    isr, x
    0x8000, //  9: push   noblock
};

static const struct pio_program vsync_program = {
    .instructions = vsync_program_instructions,
    .length = 10,
    .origin = -1,
};

static inline pio_sm_config vsync_program_get_default_config(uint offset) {
  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_wrap(&c, offset + 0, offset + 9);
  return c;
}
