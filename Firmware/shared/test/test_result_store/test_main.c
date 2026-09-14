#include <stdio.h>
#include <string.h>

#include "laser_result_store.h"
#include <unity.h>

static lp_result_store_t store;

void setUp(void) { lp_result_store_init(&store); }
void tearDown(void) {}

static lp_game_result_t result(const char *player,
                               lp_game_result_status_t status,
                               uint64_t score_us, uint64_t penalty_us) {
  lp_game_result_t value = {0};
  value.status = status;
  strncpy(value.player, player, LP_GAME_PLAYER_NAME_BYTES);
  value.score_time_us = score_us;
  value.penalty_time_us = penalty_us;
  return value;
}

static void test_recent_is_newest_first_and_limited_to_ten(void) {
  char player[8];
  for (uint8_t index = 0U; index < 12U; ++index) {
    snprintf(player, sizeof(player), "P%u", index);
    lp_game_result_t value =
        result(player, LP_GAME_RESULT_FINISHED, index + 1U, 0U);
    TEST_ASSERT_TRUE(lp_result_store_record(&store, &value));
  }
  TEST_ASSERT_EQUAL_UINT8(10U, store.recent_count);
  TEST_ASSERT_EQUAL_STRING("P11", store.recent[0].result.player);
  TEST_ASSERT_EQUAL_STRING("P2", store.recent[9].result.player);
}

static void test_top_orders_score_penalty_then_earlier_completion(void) {
  lp_game_result_t later = result("Later", LP_GAME_RESULT_FINISHED, 100U, 20U);
  lp_game_result_t slower = result("Slower", LP_GAME_RESULT_FINISHED, 200U, 0U);
  lp_game_result_t earlier = result("Earlier", LP_GAME_RESULT_FINISHED, 100U, 20U);
  lp_game_result_t less_penalty =
      result("Less penalty", LP_GAME_RESULT_FINISHED, 100U, 10U);
  TEST_ASSERT_TRUE(lp_result_store_record(&store, &earlier));
  TEST_ASSERT_TRUE(lp_result_store_record(&store, &slower));
  TEST_ASSERT_TRUE(lp_result_store_record(&store, &later));
  TEST_ASSERT_TRUE(lp_result_store_record(&store, &less_penalty));
  TEST_ASSERT_EQUAL_STRING("Less penalty", store.top[0].result.player);
  TEST_ASSERT_EQUAL_STRING("Earlier", store.top[1].result.player);
  TEST_ASSERT_EQUAL_STRING("Later", store.top[2].result.player);
  TEST_ASSERT_EQUAL_STRING("Slower", store.top[3].result.player);
}

static void test_aborted_attempt_is_recent_but_not_ranked(void) {
  lp_game_result_t aborted =
      result("Ada", LP_GAME_RESULT_ABORTED_BY_OPERATOR, 0U, 0U);
  TEST_ASSERT_TRUE(lp_result_store_record(&store, &aborted));
  TEST_ASSERT_EQUAL_UINT8(1U, store.recent_count);
  TEST_ASSERT_EQUAL_UINT8(0U, store.top_count);
}

static void test_only_best_ten_are_retained(void) {
  char player[8];
  for (uint8_t index = 0U; index < 12U; ++index) {
    snprintf(player, sizeof(player), "P%u", index);
    lp_game_result_t value =
        result(player, LP_GAME_RESULT_FINISHED, 120U - index, 0U);
    TEST_ASSERT_TRUE(lp_result_store_record(&store, &value));
  }
  TEST_ASSERT_EQUAL_UINT8(10U, store.top_count);
  TEST_ASSERT_EQUAL_UINT64(109U, store.top[0].result.score_time_us);
  TEST_ASSERT_EQUAL_UINT64(118U, store.top[9].result.score_time_us);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_recent_is_newest_first_and_limited_to_ten);
  RUN_TEST(test_top_orders_score_penalty_then_earlier_completion);
  RUN_TEST(test_aborted_attempt_is_recent_but_not_ranked);
  RUN_TEST(test_only_best_ten_are_retained);
  return UNITY_END();
}
