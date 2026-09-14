#include <string.h>

#include "laser_controller_config.h"
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

static void test_defaults_round_trip(void) {
  lp_controller_config_t original;
  lp_controller_config_defaults(&original);
  uint8_t encoded[LP_CONTROLLER_CONFIG_ENCODED_SIZE];
  TEST_ASSERT_TRUE(
      lp_controller_config_encode(&original, encoded, sizeof(encoded)));
  lp_controller_config_t restored;
  TEST_ASSERT_TRUE(
      lp_controller_config_decode(&restored, encoded, sizeof(encoded)));
  TEST_ASSERT_EQUAL_UINT32(5000U, restored.penalty_ms);
  TEST_ASSERT_EQUAL_STRING("DE", restored.wifi_country);
  TEST_ASSERT_EQUAL_STRING("Laser-Parkour", restored.wifi_ssid);
  TEST_ASSERT_EQUAL_STRING("nN7o1xt3", restored.wifi_password);
}

static void test_changed_values_round_trip(void) {
  lp_controller_config_t original;
  lp_controller_config_defaults(&original);
  original.penalty_ms = 2500U;
  strcpy(original.wifi_ssid, "Parkour-Test");
  uint8_t encoded[LP_CONTROLLER_CONFIG_ENCODED_SIZE];
  TEST_ASSERT_TRUE(
      lp_controller_config_encode(&original, encoded, sizeof(encoded)));
  lp_controller_config_t restored;
  TEST_ASSERT_TRUE(
      lp_controller_config_decode(&restored, encoded, sizeof(encoded)));
  TEST_ASSERT_EQUAL_UINT32(2500U, restored.penalty_ms);
  TEST_ASSERT_EQUAL_STRING("Parkour-Test", restored.wifi_ssid);
}

static void test_corruption_is_rejected(void) {
  lp_controller_config_t config;
  lp_controller_config_defaults(&config);
  uint8_t encoded[LP_CONTROLLER_CONFIG_ENCODED_SIZE];
  TEST_ASSERT_TRUE(
      lp_controller_config_encode(&config, encoded, sizeof(encoded)));
  encoded[20] ^= 1U;
  TEST_ASSERT_FALSE(
      lp_controller_config_decode(&config, encoded, sizeof(encoded)));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_defaults_round_trip);
  RUN_TEST(test_changed_values_round_trip);
  RUN_TEST(test_corruption_is_rejected);
  return UNITY_END();
}
