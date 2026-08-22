#ifndef USI_I2C_TARGET_H
#define USI_I2C_TARGET_H

#include <stddef.h>
#include <stdint.h>

void usi_i2c_target_init(uint8_t address, const uint8_t *identity,
                         size_t identity_length);

#endif
