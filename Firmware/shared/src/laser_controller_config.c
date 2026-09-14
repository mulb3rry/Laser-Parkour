#include "laser_controller_config.h"

#include <string.h>

#define LP_CONTROLLER_CONFIG_CRC_SIZE 4U

static void write_u32(uint8_t *output, uint32_t value) {
  for (uint8_t index = 0U; index < 4U; ++index) {
    output[index] = (uint8_t)(value >> (8U * index));
  }
}

static uint32_t read_u32(const uint8_t *input) {
  uint32_t value = 0U;
  for (uint8_t index = 0U; index < 4U; ++index) {
    value |= (uint32_t)input[index] << (8U * index);
  }
  return value;
}

static uint32_t crc32(const uint8_t *data, size_t length) {
  uint32_t crc = UINT32_MAX;
  for (size_t index = 0U; index < length; ++index) {
    crc ^= data[index];
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0xEDB88320U : crc >> 1U;
    }
  }
  return ~crc;
}

static size_t bounded_length(const char *value, size_t capacity) {
  size_t length = 0U;
  while (length < capacity && value[length] != '\0') {
    ++length;
  }
  return length;
}

void lp_controller_config_defaults(lp_controller_config_t *config) {
  memset(config, 0, sizeof(*config));
  config->penalty_ms = 5000U;
  config->maximum_run_ms = 10U * 60U * 1000U;
  memcpy(config->wifi_country, "DE", 3U);
  memcpy(config->wifi_ssid, "Laser-Parkour", 14U);
  memcpy(config->wifi_password, "nN7o1xt3", 9U);
}

bool lp_controller_config_valid(const lp_controller_config_t *config) {
  if (config == NULL || config->penalty_ms > 3600000U ||
      config->maximum_run_ms == 0U || config->wifi_country[0] == '\0' ||
      config->wifi_country[1] == '\0' || config->wifi_country[2] != '\0' ||
      config->wifi_ssid[LP_CONFIG_SSID_BYTES] != '\0' ||
      config->wifi_password[LP_CONFIG_PASSWORD_BYTES] != '\0') {
    return false;
  }
  const size_t ssid_length =
      bounded_length(config->wifi_ssid, sizeof(config->wifi_ssid));
  const size_t password_length =
      bounded_length(config->wifi_password, sizeof(config->wifi_password));
  return ssid_length != 0U && ssid_length <= LP_CONFIG_SSID_BYTES &&
         password_length >= 8U && password_length <= LP_CONFIG_PASSWORD_BYTES;
}

bool lp_controller_config_encode(const lp_controller_config_t *config,
                                 uint8_t *output, size_t output_size) {
  if (output == NULL || output_size != LP_CONTROLLER_CONFIG_ENCODED_SIZE ||
      !lp_controller_config_valid(config)) {
    return false;
  }
  memset(output, 0, output_size);
  memcpy(output, "LPC1", 4U);
  output[4] = LP_CONTROLLER_CONFIG_FORMAT_VERSION;
  write_u32(&output[8], config->penalty_ms);
  write_u32(&output[12], config->maximum_run_ms);
  memcpy(&output[16], config->wifi_country, 3U);
  memcpy(&output[19], config->wifi_ssid, 33U);
  memcpy(&output[52], config->wifi_password, 64U);
  write_u32(&output[output_size - LP_CONTROLLER_CONFIG_CRC_SIZE],
            crc32(output, output_size - LP_CONTROLLER_CONFIG_CRC_SIZE));
  return true;
}

bool lp_controller_config_decode(lp_controller_config_t *config,
                                 const uint8_t *input, size_t input_size) {
  if (config == NULL || input == NULL ||
      input_size != LP_CONTROLLER_CONFIG_ENCODED_SIZE ||
      memcmp(input, "LPC1", 4U) != 0 ||
      input[4] != LP_CONTROLLER_CONFIG_FORMAT_VERSION ||
      read_u32(&input[input_size - LP_CONTROLLER_CONFIG_CRC_SIZE]) !=
          crc32(input, input_size - LP_CONTROLLER_CONFIG_CRC_SIZE)) {
    return false;
  }
  lp_controller_config_t decoded;
  memset(&decoded, 0, sizeof(decoded));
  decoded.penalty_ms = read_u32(&input[8]);
  decoded.maximum_run_ms = read_u32(&input[12]);
  memcpy(decoded.wifi_country, &input[16], 3U);
  memcpy(decoded.wifi_ssid, &input[19], 33U);
  memcpy(decoded.wifi_password, &input[52], 64U);
  if (!lp_controller_config_valid(&decoded)) {
    return false;
  }
  *config = decoded;
  return true;
}
