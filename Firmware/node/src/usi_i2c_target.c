#include "usi_i2c_target.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include "laser_protocol.h"

enum usi_state {
  USI_STATE_ADDRESS,
  USI_STATE_RECEIVE_BYTE,
  USI_STATE_RECEIVE_BYTE_ACKNOWLEDGED,
  USI_STATE_TRANSMIT_BYTE_AFTER_ADDRESS_ACK,
  USI_STATE_REQUEST_MASTER_ACK,
  USI_STATE_CHECK_MASTER_ACK,
};

static volatile enum usi_state state;
static uint8_t target_address;
static uint8_t selected_register;
static const uint8_t *identity_data;
static uint8_t identity_size;
static uint8_t transmit_index;

static void sda_release(void) {
  DDRB &= (uint8_t)~_BV(PB0);
}

static void sda_drive(void) {
  DDRB |= _BV(PB0);
}

static void clear_usi_counter(uint8_t counter_value) {
  USISR = _BV(USISIF) | _BV(USIOIF) | _BV(USIPF) | _BV(USIDC) |
          counter_value;
}

static void enter_start_condition_mode(void) {
  sda_release();
  USICR = _BV(USISIE) | _BV(USIWM1) | _BV(USICS1);
  clear_usi_counter(0U);
}

static void receive_byte(void) {
  sda_release();
  USIDR = 0xFFU;
  clear_usi_counter(0U);
}

static void send_ack(void) {
  USIDR = 0x00U;
  sda_drive();
  clear_usi_counter(0x0EU);
}

static void receive_master_ack(void) {
  sda_release();
  // With a one-bit transfer, preload zero so the sampled master reply can be
  // tested directly: ACK remains zero, while NACK produces a non-zero value.
  USIDR = 0x00U;
  clear_usi_counter(0x0EU);
}

static uint8_t next_transmit_byte(void) {
  if (selected_register == LP_REGISTER_IDENTITY &&
      transmit_index < identity_size) {
    return identity_data[transmit_index++];
  }

  ++transmit_index;
  return 0xFFU;
}

static void send_next_byte(void) {
  USIDR = next_transmit_byte();
  sda_drive();
  clear_usi_counter(0U);
}

void usi_i2c_target_init(uint8_t address, const uint8_t *identity,
                         size_t identity_length) {
  target_address = address;
  identity_data = identity;
  identity_size = identity_length > UINT8_MAX ? UINT8_MAX
                                               : (uint8_t)identity_length;
  selected_register = LP_REGISTER_IDENTITY;
  transmit_index = 0U;
  state = USI_STATE_ADDRESS;

  // In USI two-wire mode SDA is released through its DDR bit, while SCL must
  // remain an output with its PORT latch high. The USI peripheral then pulls
  // SCL low automatically at counter overflow to clock-stretch until the ISR
  // has prepared the next data/ACK phase. Leaving SCL as an input loses that
  // ACK boundary and can leave SDA stuck low after a matched address.
  PORTB |= _BV(PB0) | _BV(PB2);
  DDRB &= (uint8_t)~_BV(PB0);
  DDRB |= _BV(PB2);
  USIDR = 0xFFU;
  enter_start_condition_mode();
}

ISR(USI_START_vect) {
  // START has already been latched by the USI hardware. Do not wait here for
  // SCL to change: the RP2040 controller waits for the target to complete its
  // START handling, so such a wait deadlocks with SDA low and SCL high.
  state = USI_STATE_ADDRESS;
  sda_release();
  USIDR = 0xFFU;
  // USIWM1:0 = 11 enables two-wire mode with SCL hold on counter
  // overflow. The hold gives this ISR time to prepare each ACK/data phase
  // before the controller supplies the next clock edge.
  USICR = _BV(USISIE) | _BV(USIOIE) | _BV(USIWM1) | _BV(USIWM0) |
          _BV(USICS1);
  clear_usi_counter(0U);
}

ISR(USI_OVF_vect) {
  switch (state) {
    case USI_STATE_ADDRESS: {
      const uint8_t address_and_direction = USIDR;
      if ((address_and_direction >> 1U) != target_address) {
        enter_start_condition_mode();
        break;
      }

      if ((address_and_direction & 0x01U) != 0U) {
        transmit_index = 0U;
        state = USI_STATE_TRANSMIT_BYTE_AFTER_ADDRESS_ACK;
        send_ack();
      } else {
        state = USI_STATE_RECEIVE_BYTE_ACKNOWLEDGED;
        send_ack();
      }
      break;
    }

    case USI_STATE_RECEIVE_BYTE:
      selected_register = USIDR;
      state = USI_STATE_RECEIVE_BYTE_ACKNOWLEDGED;
      send_ack();
      break;

    case USI_STATE_RECEIVE_BYTE_ACKNOWLEDGED:
      state = USI_STATE_RECEIVE_BYTE;
      receive_byte();
      break;

    case USI_STATE_TRANSMIT_BYTE_AFTER_ADDRESS_ACK:
      state = USI_STATE_REQUEST_MASTER_ACK;
      send_next_byte();
      break;

    case USI_STATE_REQUEST_MASTER_ACK:
      state = USI_STATE_CHECK_MASTER_ACK;
      receive_master_ack();
      break;

    case USI_STATE_CHECK_MASTER_ACK:
      if (USIDR == 0x00U) {
        state = USI_STATE_REQUEST_MASTER_ACK;
        send_next_byte();
      } else {
        enter_start_condition_mode();
      }
      break;

    default:
      enter_start_condition_mode();
      break;
  }
}
