#include <string.h>

#include "laser_top_storage.h"
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

static lp_game_result_t result(const char *player, uint64_t score_us) {
  lp_game_result_t value = {0};
  value.status = LP_GAME_RESULT_FINISHED;
  strncpy(value.player, player, LP_GAME_PLAYER_NAME_BYTES);
  value.start_us = 100U;
  value.end_us = 200U;
  value.raw_time_us = score_us - 50U;
  value.penalty_time_us = 50U;
  value.score_time_us = score_us;
  value.interruptions = 2U;
  return value;
}

static void test_round_trip_restores_top_but_not_recent(void) {
  lp_result_store_t original;
  lp_result_store_init(&original);
  lp_game_result_t first = result("Ada", 1000U);
  lp_game_result_t second = result("Ben", 900U);
  TEST_ASSERT_TRUE(lp_result_store_record(&original, &first));
  TEST_ASSERT_TRUE(lp_result_store_record(&original, &second));
  uint8_t encoded[LP_TOP_STORAGE_ENCODED_SIZE];
  TEST_ASSERT_TRUE(lp_top_storage_encode(&original, encoded, sizeof(encoded)));

  lp_result_store_t restored;
  TEST_ASSERT_TRUE(lp_top_storage_decode(&restored, encoded, sizeof(encoded)));
  TEST_ASSERT_EQUAL_UINT8(2U, restored.top_count);
  TEST_ASSERT_EQUAL_UINT8(0U, restored.recent_count);
  TEST_ASSERT_EQUAL_STRING("Ben", restored.top[0].result.player);
  TEST_ASSERT_EQUAL_UINT64(900U, restored.top[0].result.score_time_us);
  TEST_ASSERT_EQUAL_UINT64(original.next_completion_sequence,
                           restored.next_completion_sequence);
}

static void test_corrupted_crc_is_rejected(void) {
  lp_result_store_t original;
  lp_result_store_init(&original);
  lp_game_result_t value = result("Ada", 1000U);
  TEST_ASSERT_TRUE(lp_result_store_record(&original, &value));
  uint8_t encoded[LP_TOP_STORAGE_ENCODED_SIZE];
  TEST_ASSERT_TRUE(lp_top_storage_encode(&original, encoded, sizeof(encoded)));
  encoded[40] ^= 0x80U;
  TEST_ASSERT_FALSE(lp_top_storage_decode(&original, encoded, sizeof(encoded)));
}

static void test_wrong_size_and_version_are_rejected(void) {
  lp_result_store_t original;
  lp_result_store_init(&original);
  uint8_t encoded[LP_TOP_STORAGE_ENCODED_SIZE];
  TEST_ASSERT_TRUE(lp_top_storage_encode(&original, encoded, sizeof(encoded)));
  TEST_ASSERT_FALSE(
      lp_top_storage_decode(&original, encoded, sizeof(encoded) - 1U));
  encoded[4] = 2U;
  TEST_ASSERT_FALSE(lp_top_storage_decode(&original, encoded, sizeof(encoded)));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_round_trip_restores_top_but_not_recent);
  RUN_TEST(test_corrupted_crc_is_rejected);
  RUN_TEST(test_wrong_size_and_version_are_rejected);
  return UNITY_END();
}
