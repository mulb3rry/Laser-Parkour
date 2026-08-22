#ifndef USI_I2C_TARGET_H
#define USI_I2C_TARGET_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
  uint8_t address;
  const uint8_t *data;
  uint8_t size;
} usi_i2c_register_t;

void usi_i2c_target_init(uint8_t address, const usi_i2c_register_t *registers,
                         uint8_t register_count);

#endif
