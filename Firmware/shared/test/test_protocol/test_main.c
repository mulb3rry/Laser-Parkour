/* Shared protocol unit tests for both firmware targets. */
#include <string.h>

#include "laser_protocol.h"
#include <unity.h>

void setUp(void) {
}

void tearDown(void) {
}

static void test_layout_sizes(void) {
  TEST_ASSERT_EQUAL_UINT(11U, sizeof(lp_identity_register_t));
  TEST_ASSERT_EQUAL_UINT(7U, sizeof(lp_fast_status_register_t));
  TEST_ASSERT_EQUAL_UINT(10U, sizeof(lp_diagnostics_register_t));
  TEST_ASSERT_EQUAL_UINT(9U, sizeof(lp_sensor_config_register_t));
  TEST_ASSERT_EQUAL_UINT(3U, sizeof(lp_staged_identity_register_t));
  TEST_ASSERT_EQUAL_UINT(7U, sizeof(lp_command_register_t));
  TEST_ASSERT_EQUAL_UINT(6U, sizeof(lp_command_result_register_t));
}

static void test_little_endian_helpers(void) {
  static const uint16_t values[] = {0U, 1U, 255U, 256U, 1023U, 65535U};

  for (size_t index = 0U; index < sizeof(values) / sizeof(values[0]); ++index) {
    const lp_u16_le_t encoded = lp_u16_encode(values[index]);
    TEST_ASSERT_EQUAL_UINT16(values[index], lp_u16_decode(encoded));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)values[index], encoded.low);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(values[index] >> 8U), encoded.high);
  }
}

static void test_crc_standard_vector(void) {
  static const uint8_t input[] = "123456789";
  TEST_ASSERT_EQUAL_HEX8(0xF4U, lp_crc8(input, sizeof(input) - 1U));
}

static void test_crc_documented_register_vector(void) {
  static const uint8_t data[] = {0x01U, 0x00U, 0x02U,
                                 0x00U, 0x03U, 0x00U};
  TEST_ASSERT_EQUAL_HEX8(
      0xE9U, lp_register_crc8(LP_REGISTER_FAST_STATUS, data, sizeof(data)));

  uint8_t corrupted[sizeof(data)];
  memcpy(corrupted, data, sizeof(data));
  corrupted[3] ^= 0x01U;
  TEST_ASSERT_NOT_EQUAL_HEX8(
      0xE9U,
      lp_register_crc8(LP_REGISTER_FAST_STATUS, corrupted, sizeof(corrupted)));
  TEST_ASSERT_NOT_EQUAL_HEX8(
      0xE9U, lp_register_crc8(LP_REGISTER_DIAGNOSTICS, data, sizeof(data)));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_layout_sizes);
  RUN_TEST(test_little_endian_helpers);
  RUN_TEST(test_crc_standard_vector);
  RUN_TEST(test_crc_documented_register_vector);
  return UNITY_END();
}
