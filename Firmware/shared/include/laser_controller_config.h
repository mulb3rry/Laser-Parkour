#ifndef LASER_CONTROLLER_CONFIG_H
#define LASER_CONTROLLER_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LP_CONFIG_SSID_BYTES 32U
#define LP_CONFIG_PASSWORD_BYTES 63U
#define LP_CONTROLLER_CONFIG_FORMAT_VERSION 1U
#define LP_CONTROLLER_CONFIG_ENCODED_SIZE 120U

typedef struct {
  uint32_t penalty_ms;
  uint32_t maximum_run_ms;
  char wifi_country[3];
  char wifi_ssid[LP_CONFIG_SSID_BYTES + 1U];
  char wifi_password[LP_CONFIG_PASSWORD_BYTES + 1U];
} lp_controller_config_t;

void lp_controller_config_defaults(lp_controller_config_t *config);
bool lp_controller_config_valid(const lp_controller_config_t *config);
bool lp_controller_config_encode(const lp_controller_config_t *config,
                                 uint8_t *output, size_t output_size);
bool lp_controller_config_decode(lp_controller_config_t *config,
                                 const uint8_t *input, size_t input_size);

#ifdef __cplusplus
}
#endif

#endif
