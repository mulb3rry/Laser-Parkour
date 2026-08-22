#ifndef LASER_PROTOCOL_H
#define LASER_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LP_PROTOCOL_MAJOR 1U
#define LP_PROTOCOL_MINOR 0U
#define LP_CONFIG_FORMAT_VERSION 1U

#define LP_ADDRESS_COMMISSIONING 0x08U
#define LP_ADDRESS_NORMAL_MIN 0x10U
#define LP_ADDRESS_NORMAL_MAX 0x6FU
#define LP_MAX_LASER_NODES 16U
#define LP_MAX_TOTAL_NODES 18U

typedef enum {
  LP_ROLE_UNCONFIGURED = 0,
  LP_ROLE_LASER = 1,
  LP_ROLE_START = 2,
  LP_ROLE_FINISH = 3,
} lp_node_role_t;

typedef enum {
  LP_MODE_SETUP = 0,
  LP_MODE_GAME = 1,
} lp_operating_mode_t;

typedef enum {
  LP_INPUT_INACTIVE = 0,
  LP_INPUT_ACTIVE = 1,
  LP_INPUT_UNSTABLE = 2,
  LP_INPUT_NOT_APPLICABLE = 255,
} lp_input_state_t;

typedef enum {
  LP_RESULT_OK = 0,
  LP_RESULT_UNKNOWN_REGISTER = 1,
  LP_RESULT_INVALID_LENGTH = 2,
  LP_RESULT_INVALID_CRC = 3,
  LP_RESULT_INVALID_VALUE = 4,
  LP_RESULT_WRONG_ROLE = 5,
  LP_RESULT_WRONG_MODE = 6,
  LP_RESULT_NOT_COMMISSIONING = 7,
  LP_RESULT_NO_STAGED_CONFIG = 8,
  LP_RESULT_EEPROM_FAILURE = 9,
  LP_RESULT_SEQUENCE_CONFLICT = 10,
  LP_RESULT_PROTECTION_FAILED = 11,
  LP_RESULT_BUSY = 12,
  LP_RESULT_UNKNOWN_COMMAND = 13,
} lp_result_t;

typedef enum {
  LP_REGISTER_IDENTITY = 0x00,
  LP_REGISTER_FAST_STATUS = 0x10,
  LP_REGISTER_DIAGNOSTICS = 0x18,
  LP_REGISTER_ACTIVE_SENSOR_CONFIG = 0x20,
  LP_REGISTER_STAGED_SENSOR_CONFIG = 0x30,
  LP_REGISTER_STAGED_IDENTITY = 0x40,
  LP_REGISTER_COMMAND = 0xF0,
  LP_REGISTER_COMMAND_RESULT = 0xF1,
} lp_register_t;

typedef enum {
  LP_COMMAND_SET_MODE = 0x01,
  LP_COMMAND_SAVE_CONFIG = 0x02,
  LP_COMMAND_RESET_COUNTER = 0x03,
  LP_COMMAND_FACTORY_RESET = 0x04,
  LP_COMMAND_IDENTIFY = 0x05,
} lp_command_t;

#define LP_IDENTIFY_DURATION_SECONDS 10U

#define LP_CAPABILITY_LASER_SENSING 0x0001U
#define LP_CAPABILITY_BUTTON_INPUT 0x0002U
#define LP_CAPABILITY_FU_OUTPUT 0x0004U
#define LP_CAPABILITY_STATUS_LED 0x0008U

#define LP_STATUS_CONFIG_VALID 0x0001U
#define LP_STATUS_COMMISSIONED 0x0002U
#define LP_STATUS_GAME_MODE 0x0004U
#define LP_STATUS_INPUT_ACTIVE 0x0008U
#define LP_STATUS_INPUT_UNSTABLE 0x0010U
#define LP_STATUS_COOLDOWN_ACTIVE 0x0020U
#define LP_STATUS_CONFIG_STAGED 0x0040U
#define LP_STATUS_IDENTITY_STAGED 0x0080U
#define LP_STATUS_LAST_OPERATION_ERROR 0x0100U
#define LP_STATUS_ADC_RANGE_WARNING 0x0200U
#define LP_STATUS_COUNTER_OVERFLOWED 0x0400U

#define LP_SENSOR_THRESHOLD_MIN 0U
#define LP_SENSOR_THRESHOLD_MAX 1023U
#define LP_SENSOR_HYSTERESIS_MIN 0U
#define LP_SENSOR_HYSTERESIS_MAX 1023U
#define LP_SENSOR_STABLE_TIME_MIN_MS 0U
#define LP_SENSOR_STABLE_TIME_MAX_MS 1000U
#define LP_SENSOR_COOLDOWN_MIN_MS 0U
#define LP_SENSOR_COOLDOWN_MAX_MS 5000U

#define LP_FACTORY_RESET_ARG0 ((uint8_t)'L')
#define LP_FACTORY_RESET_ARG1 ((uint8_t)'P')
#define LP_FACTORY_RESET_ARG2 ((uint8_t)'F')
#define LP_FACTORY_RESET_ARG3 ((uint8_t)'R')

/* Multi-byte wire values are represented as bytes to avoid alignment and
 * host-endianness assumptions when blocks are copied to or from I2C buffers. */
typedef struct {
  uint8_t low;
  uint8_t high;
} lp_u16_le_t;

#if defined(__GNUC__)
#define LP_PACKED __attribute__((packed))
#else
#define LP_PACKED
#endif

typedef struct LP_PACKED {
  uint8_t protocol_major;
  uint8_t protocol_minor;
  uint8_t firmware_major;
  uint8_t firmware_minor;
  uint8_t firmware_patch;
  uint8_t role;
  uint8_t address;
  lp_u16_le_t capabilities;
  uint8_t config_format_version;
  uint8_t crc8;
} lp_identity_register_t;

typedef struct LP_PACKED {
  lp_u16_le_t boot_counter;
  lp_u16_le_t event_counter;
  lp_u16_le_t status_flags;
  uint8_t crc8;
} lp_fast_status_register_t;

typedef struct LP_PACKED {
  lp_u16_le_t raw_adc;
  lp_u16_le_t filtered_adc;
  uint8_t input_state;
  uint8_t operating_mode;
  uint8_t last_result;
  lp_u16_le_t cooldown_remaining_ms;
  uint8_t crc8;
} lp_diagnostics_register_t;

typedef struct LP_PACKED {
  lp_u16_le_t broken_threshold;
  lp_u16_le_t hysteresis;
  lp_u16_le_t stable_time_ms;
  lp_u16_le_t cooldown_ms;
  uint8_t crc8;
} lp_sensor_config_register_t;

typedef struct LP_PACKED {
  uint8_t address;
  uint8_t role;
  uint8_t crc8;
} lp_staged_identity_register_t;

/* Bytes written after the LP_REGISTER_COMMAND register pointer. */
typedef struct LP_PACKED {
  uint8_t command;
  uint8_t sequence;
  uint8_t arguments[4];
  uint8_t crc8;
} lp_command_register_t;

typedef struct LP_PACKED {
  uint8_t sequence;
  uint8_t command;
  uint8_t result;
  lp_u16_le_t detail;
  uint8_t crc8;
} lp_command_result_register_t;

#define LP_IDENTITY_REGISTER_SIZE 11U
#define LP_FAST_STATUS_REGISTER_SIZE 7U
#define LP_DIAGNOSTICS_REGISTER_SIZE 10U
#define LP_SENSOR_CONFIG_REGISTER_SIZE 9U
#define LP_STAGED_IDENTITY_REGISTER_SIZE 3U
#define LP_COMMAND_REGISTER_SIZE 7U
#define LP_COMMAND_RESULT_REGISTER_SIZE 6U

#if defined(__cplusplus)
#define LP_STATIC_ASSERT(condition, message) static_assert(condition, message)
#else
#define LP_STATIC_ASSERT(condition, message) _Static_assert(condition, message)
#endif

LP_STATIC_ASSERT(sizeof(lp_u16_le_t) == 2U, "16-bit wire value size");
LP_STATIC_ASSERT(sizeof(lp_identity_register_t) == LP_IDENTITY_REGISTER_SIZE,
                 "identity register size");
LP_STATIC_ASSERT(
    sizeof(lp_fast_status_register_t) == LP_FAST_STATUS_REGISTER_SIZE,
    "fast status register size");
LP_STATIC_ASSERT(
    sizeof(lp_diagnostics_register_t) == LP_DIAGNOSTICS_REGISTER_SIZE,
    "diagnostics register size");
LP_STATIC_ASSERT(
    sizeof(lp_sensor_config_register_t) == LP_SENSOR_CONFIG_REGISTER_SIZE,
    "sensor config register size");
LP_STATIC_ASSERT(
    sizeof(lp_staged_identity_register_t) == LP_STAGED_IDENTITY_REGISTER_SIZE,
    "staged identity register size");
LP_STATIC_ASSERT(sizeof(lp_command_register_t) == LP_COMMAND_REGISTER_SIZE,
                 "command register size");
LP_STATIC_ASSERT(
    sizeof(lp_command_result_register_t) == LP_COMMAND_RESULT_REGISTER_SIZE,
    "command result register size");

static inline lp_u16_le_t lp_u16_encode(uint16_t value) {
  lp_u16_le_t encoded;
  encoded.low = (uint8_t)value;
  encoded.high = (uint8_t)(value >> 8U);
  return encoded;
}

static inline uint16_t lp_u16_decode(lp_u16_le_t value) {
  return (uint16_t)((uint16_t)value.low | ((uint16_t)value.high << 8U));
}

static inline uint8_t lp_crc8_update(uint8_t crc, uint8_t value) {
  crc ^= value;
  for (uint8_t bit = 0U; bit < 8U; ++bit) {
    crc = (crc & 0x80U) != 0U ? (uint8_t)((crc << 1U) ^ 0x07U)
                              : (uint8_t)(crc << 1U);
  }
  return crc;
}

static inline uint8_t lp_crc8(const uint8_t *data, size_t length) {
  uint8_t crc = 0U;
  for (size_t index = 0U; index < length; ++index) {
    crc = lp_crc8_update(crc, data[index]);
  }
  return crc;
}

/* Calculates the CRC stored at the end of a register block. data_length does
 * not include that stored CRC byte. */
static inline uint8_t lp_register_crc8(uint8_t register_address,
                                       const uint8_t *data,
                                       size_t data_length) {
  uint8_t crc = lp_crc8_update(0U, register_address);
  for (size_t index = 0U; index < data_length; ++index) {
    crc = lp_crc8_update(crc, data[index]);
  }
  return crc;
}

#undef LP_STATIC_ASSERT
#undef LP_PACKED

#ifdef __cplusplus
}
#endif

#endif
