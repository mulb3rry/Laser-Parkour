#include "usi_i2c_target.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

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
static const usi_i2c_register_t *register_table;
static uint8_t register_table_size;
static uint8_t transmit_index;
static uint8_t transmit_size;
static uint8_t transmit_buffer[LP_IDENTITY_REGISTER_SIZE];
static uint8_t receive_index;
static uint8_t receive_size;
static uint8_t receive_register;
static uint8_t receive_buffer[LP_SENSOR_CONFIG_REGISTER_SIZE];
static volatile uint8_t receive_pending;
static uint8_t pointer_received;
static volatile uint8_t completed_read_pending;
static uint8_t completed_read_register;
static uint8_t completed_read_size;
static uint8_t completed_read_buffer[LP_COMMAND_RESULT_REGISTER_SIZE];

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
  if (transmit_index < transmit_size) {
    return transmit_buffer[transmit_index++];
  }

  ++transmit_index;
  return 0xFFU;
}

static void snapshot_selected_register(void) {
  transmit_size = 0U;
  for (uint8_t index = 0U; index < register_table_size; ++index) {
    if (register_table[index].address != selected_register) {
      continue;
    }

    transmit_size = register_table[index].size;
    if (transmit_size > sizeof(transmit_buffer)) {
      transmit_size = sizeof(transmit_buffer);
    }
    for (uint8_t byte = 0U; byte < transmit_size; ++byte) {
      transmit_buffer[byte] = register_table[index].data[byte];
    }
    break;
  }
}

static uint8_t selected_register_write_size(void) {
  for (uint8_t index = 0U; index < register_table_size; ++index) {
    if (register_table[index].address == selected_register) {
      return register_table[index].write_size;
    }
  }
  return 0U;
}

static void send_next_byte(void) {
  USIDR = next_transmit_byte();
  sda_drive();
  clear_usi_counter(0U);
}

void usi_i2c_target_init(uint8_t address, const usi_i2c_register_t *registers,
                         uint8_t register_count) {
  target_address = address;
  register_table = registers;
  register_table_size = register_count;
  selected_register = LP_REGISTER_IDENTITY;
  transmit_index = 0U;
  receive_pending = 0U;
  completed_read_pending = 0U;
  pointer_received = 0U;
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

uint8_t usi_i2c_target_take_completed_read(uint8_t *register_address,
                                           uint8_t *data, uint8_t capacity) {
  uint8_t length = 0U;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (completed_read_pending != 0U && completed_read_size <= capacity) {
      *register_address = completed_read_register;
      length = completed_read_size;
      for (uint8_t index = 0U; index < length; ++index) {
        data[index] = completed_read_buffer[index];
      }
      completed_read_pending = 0U;
    }
  }
  return length;
}

void usi_i2c_target_set_address(uint8_t address) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { target_address = address; }
}

uint8_t usi_i2c_target_take_write(uint8_t *register_address, uint8_t *data,
                                  uint8_t capacity) {
  uint8_t length = 0U;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (receive_pending != 0U && receive_size <= capacity) {
      *register_address = receive_register;
      length = receive_size;
      for (uint8_t index = 0U; index < length; ++index) {
        data[index] = receive_buffer[index];
      }
      receive_pending = 0U;
    }
  }
  return length;
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
        snapshot_selected_register();
        state = USI_STATE_TRANSMIT_BYTE_AFTER_ADDRESS_ACK;
        send_ack();
      } else {
        receive_index = 0U;
        receive_size = 0U;
        pointer_received = 0U;
        state = USI_STATE_RECEIVE_BYTE_ACKNOWLEDGED;
        send_ack();
      }
      break;
    }

    case USI_STATE_RECEIVE_BYTE:
      if (pointer_received == 0U) {
        selected_register = USIDR;
        receive_register = selected_register;
        pointer_received = 1U;
        receive_size = selected_register_write_size();
      } else if (receive_index < receive_size &&
                 receive_index < sizeof(receive_buffer)) {
        receive_buffer[receive_index++] = USIDR;
        if (receive_index == receive_size) {
          receive_pending = 1U;
        }
      }
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
        if (transmit_index >= transmit_size) {
          completed_read_register = selected_register;
          completed_read_size = transmit_size;
          if (completed_read_size > sizeof(completed_read_buffer)) {
            completed_read_size = sizeof(completed_read_buffer);
          }
          for (uint8_t index = 0U; index < completed_read_size; ++index) {
            completed_read_buffer[index] = transmit_buffer[index];
          }
          completed_read_pending = 1U;
        }
        enter_start_condition_mode();
      }
      break;

    default:
      enter_start_condition_mode();
      break;
  }
}
