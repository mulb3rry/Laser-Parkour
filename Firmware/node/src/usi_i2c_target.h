#ifndef USI_I2C_TARGET_H
#define USI_I2C_TARGET_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
  uint8_t address;
  const uint8_t *data;
  uint8_t size;
  uint8_t write_size;
} usi_i2c_register_t;

void usi_i2c_target_init(uint8_t address, const usi_i2c_register_t *registers,
                         uint8_t register_count);
uint8_t usi_i2c_target_take_write(uint8_t *register_address, uint8_t *data,
                                  uint8_t capacity);
uint8_t usi_i2c_target_take_completed_read(uint8_t *register_address,
                                           uint8_t *data, uint8_t capacity);
void usi_i2c_target_set_address(uint8_t address);

#endif
