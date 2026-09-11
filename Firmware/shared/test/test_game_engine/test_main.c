#include <stdint.h>

#include "laser_game_engine.h"
#include <unity.h>

static lp_game_engine_t game;

void setUp(void) {
  const lp_game_settings_t settings = {.penalty_ms = 5000U,
                                       .maximum_run_ms = 600000U};
  lp_game_init(&game, settings);
}

void tearDown(void) {}

static void make_wait_player(void) {
  TEST_ASSERT_EQUAL(LP_GAME_OK, lp_game_enter_game(&game, true));
  TEST_ASSERT_EQUAL(LP_GAME_WAIT_PLAYER, game.state);
}

static void make_wait_finish(uint64_t start_us) {
  make_wait_player();
  TEST_ASSERT_EQUAL(LP_GAME_OK, lp_game_set_player(&game, "Player One"));
  TEST_ASSERT_EQUAL(LP_GAME_OK, lp_game_start(&game, start_us, true));
}

static void test_initialization_and_setup_validation(void) {
  TEST_ASSERT_EQUAL(LP_GAME_SETUP, game.state);
  TEST_ASSERT_EQUAL(LP_GAME_INVALID_STATE, lp_game_enter_game(&game, false));
  TEST_ASSERT_EQUAL(LP_GAME_FAULT, game.state);
  TEST_ASSERT_EQUAL(LP_GAME_OK, lp_game_enter_game(&game, true));
  TEST_ASSERT_EQUAL(LP_GAME_WAIT_PLAYER, game.state);
  lp_game_enter_setup(&game);
  TEST_ASSERT_EQUAL(LP_GAME_SETUP, game.state);
}

static void test_player_name_moves_to_wait_start(void) {
  make_wait_player();
  TEST_ASSERT_EQUAL(LP_GAME_INVALID_PLAYER, lp_game_set_player(&game, "   "));
  TEST_ASSERT_EQUAL(LP_GAME_WAIT_PLAYER, game.state);
  TEST_ASSERT_EQUAL(LP_GAME_OK, lp_game_set_player(&game, "Ada"));
  TEST_ASSERT_EQUAL(LP_GAME_WAIT_START, game.state);
  TEST_ASSERT_EQUAL_STRING("Ada", game.current_player);
}

static void test_start_requires_wait_start_and_clear_beams(void) {
  make_wait_player();
  TEST_ASSERT_EQUAL(LP_GAME_INVALID_STATE, lp_game_start(&game, 1000U, true));
  TEST_ASSERT_EQUAL(LP_GAME_OK, lp_game_set_player(&game, "Ada"));
  TEST_ASSERT_EQUAL(LP_GAME_BEAM_BLOCKED, lp_game_start(&game, 1000U, false));
  TEST_ASSERT_EQUAL(LP_GAME_WAIT_START, game.state);
  TEST_ASSERT_EQUAL(LP_GAME_OK, lp_game_start(&game, 1000U, true));
  TEST_ASSERT_EQUAL(LP_GAME_WAIT_FINISH, game.state);
}

static void test_finish_scores_and_returns_to_wait_player(void) {
  make_wait_finish(1000000U);
  TEST_ASSERT_EQUAL(LP_GAME_OK, lp_game_add_interruptions(&game, 3U));
  TEST_ASSERT_EQUAL(LP_GAME_OK, lp_game_finish(&game, 11250000U));
  TEST_ASSERT_EQUAL(LP_GAME_WAIT_PLAYER, game.state);
  TEST_ASSERT_EQUAL(LP_GAME_RESULT_FINISHED, game.last_result.status);
  TEST_ASSERT_EQUAL_UINT64(10250000U, game.last_result.raw_time_us);
  TEST_ASSERT_EQUAL_UINT64(15000000U, game.last_result.penalty_time_us);
  TEST_ASSERT_EQUAL_UINT64(25250000U, game.last_result.score_time_us);
  TEST_ASSERT_EQUAL_UINT32(3U, game.last_result.interruptions);
  TEST_ASSERT_EQUAL_STRING("Player One", game.last_result.player);
  TEST_ASSERT_EQUAL_STRING("", game.current_player);
}

static void test_unexpected_events_are_ignored_by_state(void) {
  make_wait_player();
  TEST_ASSERT_EQUAL(LP_GAME_INVALID_STATE, lp_game_finish(&game, 1000U));
  TEST_ASSERT_EQUAL(LP_GAME_INVALID_STATE, lp_game_add_interruptions(&game, 1U));
  TEST_ASSERT_EQUAL(LP_GAME_WAIT_PLAYER, game.state);
  TEST_ASSERT_EQUAL(LP_GAME_OK, lp_game_set_player(&game, "Ada"));
  TEST_ASSERT_EQUAL(LP_GAME_INVALID_STATE, lp_game_finish(&game, 1000U));
  TEST_ASSERT_EQUAL(LP_GAME_INVALID_STATE, lp_game_add_interruptions(&game, 1U));
  TEST_ASSERT_EQUAL(LP_GAME_WAIT_START, game.state);
  TEST_ASSERT_EQUAL(LP_GAME_OK, lp_game_start(&game, 2000U, true));
  TEST_ASSERT_EQUAL(LP_GAME_INVALID_STATE, lp_game_start(&game, 3000U, true));
  TEST_ASSERT_EQUAL(LP_GAME_TIMESTAMP_ERROR, lp_game_finish(&game, 1000U));
  TEST_ASSERT_EQUAL(LP_GAME_WAIT_FINISH, game.state);
}

static void test_operator_abort_returns_to_wait_player(void) {
  make_wait_player();
  TEST_ASSERT_EQUAL(LP_GAME_OK, lp_game_set_player(&game, "Ada"));
  TEST_ASSERT_EQUAL(LP_GAME_OK, lp_game_abort(&game, 500U));
  TEST_ASSERT_EQUAL(LP_GAME_WAIT_PLAYER, game.state);
  TEST_ASSERT_EQUAL(LP_GAME_RESULT_ABORTED_BY_OPERATOR, game.last_result.status);
}

static void test_timeout_returns_to_wait_player(void) {
  const lp_game_settings_t settings = {.penalty_ms = 5000U,
                                       .maximum_run_ms = 10U};
  lp_game_init(&game, settings);
  make_wait_finish(1000U);
  TEST_ASSERT_FALSE(lp_game_tick(&game, 10999U));
  TEST_ASSERT_TRUE(lp_game_tick(&game, 11000U));
  TEST_ASSERT_EQUAL(LP_GAME_WAIT_PLAYER, game.state);
  TEST_ASSERT_EQUAL(LP_GAME_RESULT_ABORTED_TIMEOUT, game.last_result.status);
}

static void test_fault_aborts_active_attempt(void) {
  make_wait_finish(1000U);
  TEST_ASSERT_EQUAL(LP_GAME_OK, lp_game_add_interruptions(&game, 2U));
  lp_game_fault(&game, 5000U);
  TEST_ASSERT_EQUAL(LP_GAME_FAULT, game.state);
  TEST_ASSERT_EQUAL(LP_GAME_RESULT_ABORTED_FAULT, game.last_result.status);
  TEST_ASSERT_EQUAL_UINT64(4000U, game.last_result.raw_time_us);
}

static void test_fault_while_waiting_for_player_changes_state(void) {
  make_wait_player();
  lp_game_fault(&game, 5000U);
  TEST_ASSERT_EQUAL(LP_GAME_FAULT, game.state);
  TEST_ASSERT_EQUAL(LP_GAME_RESULT_NONE, game.last_result.status);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_initialization_and_setup_validation);
  RUN_TEST(test_player_name_moves_to_wait_start);
  RUN_TEST(test_start_requires_wait_start_and_clear_beams);
  RUN_TEST(test_finish_scores_and_returns_to_wait_player);
  RUN_TEST(test_unexpected_events_are_ignored_by_state);
  RUN_TEST(test_operator_abort_returns_to_wait_player);
  RUN_TEST(test_timeout_returns_to_wait_player);
  RUN_TEST(test_fault_aborts_active_attempt);
  RUN_TEST(test_fault_while_waiting_for_player_changes_state);
  return UNITY_END();
}
