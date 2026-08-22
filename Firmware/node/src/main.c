#include "laser_protocol.h"
#include "usi_i2c_target.h"

#include <avr/interrupt.h>

static lp_identity_register_t identity = {
    .protocol_major = LP_PROTOCOL_MAJOR,
    .protocol_minor = LP_PROTOCOL_MINOR,
    .firmware_major = 0U,
    .firmware_minor = 1U,
    .firmware_patch = 0U,
    .role = LP_ROLE_UNCONFIGURED,
    .address = LP_ADDRESS_COMMISSIONING,
    .capabilities = {.low = (uint8_t)LP_CAPABILITY_STATUS_LED, .high = 0U},
    .config_format_version = LP_CONFIG_FORMAT_VERSION,
    .crc8 = 0U,
};

int main(void) {
  // Milestone 1.2 boot indicator: steady on means the ATtiny is powered and
  // has reached the firmware entry point. I2C success is tested separately by
  // the controller and does not change this LED yet.
  PORTB |= _BV(PB1);
  DDRB |= _BV(PB1);

  identity.crc8 = lp_register_crc8(
      LP_REGISTER_IDENTITY, (const uint8_t *)&identity,
      sizeof(identity) - sizeof(identity.crc8));
  usi_i2c_target_init(LP_ADDRESS_COMMISSIONING,
                      (const uint8_t *)&identity, sizeof(identity));
  sei();

  for (;;) {
  }
}
