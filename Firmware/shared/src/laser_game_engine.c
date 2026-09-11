#include "laser_game_engine.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static bool copy_player(char destination[LP_GAME_PLAYER_NAME_BYTES + 1U],
                        const char *source) {
  if (source == NULL) {
    return false;
  }
  size_t length = 0U;
  bool has_non_space = false;
  while (source[length] != '\0') {
    if (length >= LP_GAME_PLAYER_NAME_BYTES) {
      return false;
    }
    if (source[length] != ' ' && source[length] != '\t' &&
        source[length] != '\r' && source[length] != '\n') {
      has_non_space = true;
    }
    ++length;
  }
  if (!has_non_space) {
    return false;
  }
  memcpy(destination, source, length + 1U);
  return true;
}

static uint64_t saturating_add(uint64_t left, uint64_t right) {
  return UINT64_MAX - left < right ? UINT64_MAX : left + right;
}

static uint64_t saturating_multiply(uint64_t left, uint64_t right) {
  if (left != 0U && right > UINT64_MAX / left) {
    return UINT64_MAX;
  }
  return left * right;
}

static void finish_attempt(lp_game_engine_t *game,
                           lp_game_result_status_t status,
                           uint64_t timestamp_us) {
  lp_game_result_t *result = &game->last_result;
  memset(result, 0, sizeof(*result));
  result->status = status;
  memcpy(result->player, game->current_player, sizeof(result->player));
  result->start_us = game->start_us;
  result->end_us = timestamp_us;
  result->interruptions = game->interruptions;
  if (game->state == LP_GAME_WAIT_FINISH && timestamp_us >= game->start_us) {
    result->raw_time_us = timestamp_us - game->start_us;
  }
  if (status == LP_GAME_RESULT_FINISHED) {
    result->penalty_time_us = saturating_multiply(
        saturating_multiply(game->interruptions, game->settings.penalty_ms),
        1000U);
    result->score_time_us =
        saturating_add(result->raw_time_us, result->penalty_time_us);
  }
}

void lp_game_init(lp_game_engine_t *game, lp_game_settings_t settings) {
  memset(game, 0, sizeof(*game));
  game->state = LP_GAME_SETUP;
  game->settings = settings;
}

lp_game_action_result_t lp_game_enter_game(lp_game_engine_t *game,
                                            bool setup_valid) {
  if (game->state != LP_GAME_SETUP && game->state != LP_GAME_FAULT) {
    return LP_GAME_INVALID_STATE;
  }
  game->state = setup_valid ? LP_GAME_WAIT_PLAYER : LP_GAME_FAULT;
  return setup_valid ? LP_GAME_OK : LP_GAME_INVALID_STATE;
}

void lp_game_enter_setup(lp_game_engine_t *game) {
  game->state = LP_GAME_SETUP;
  game->current_player[0] = '\0';
  game->interruptions = 0U;
}

lp_game_action_result_t lp_game_set_player(lp_game_engine_t *game,
                                           const char *player) {
  if (game->state != LP_GAME_WAIT_PLAYER) {
    return LP_GAME_INVALID_STATE;
  }
  if (!copy_player(game->current_player, player)) {
    return LP_GAME_INVALID_PLAYER;
  }
  game->interruptions = 0U;
  game->start_us = 0U;
  game->state = LP_GAME_WAIT_START;
  return LP_GAME_OK;
}

lp_game_action_result_t lp_game_start(lp_game_engine_t *game,
                                      uint64_t timestamp_us,
                                      bool all_beams_clear) {
  if (game->state != LP_GAME_WAIT_START) {
    return LP_GAME_INVALID_STATE;
  }
  if (!all_beams_clear) {
    return LP_GAME_BEAM_BLOCKED;
  }
  game->start_us = timestamp_us;
  game->interruptions = 0U;
  game->state = LP_GAME_WAIT_FINISH;
  return LP_GAME_OK;
}

lp_game_action_result_t lp_game_add_interruptions(lp_game_engine_t *game,
                                                  uint32_t count) {
  if (game->state != LP_GAME_WAIT_FINISH) {
    return LP_GAME_INVALID_STATE;
  }
  game->interruptions = UINT32_MAX - game->interruptions < count
                            ? UINT32_MAX
                            : game->interruptions + count;
  return LP_GAME_OK;
}

lp_game_action_result_t lp_game_finish(lp_game_engine_t *game,
                                       uint64_t timestamp_us) {
  if (game->state != LP_GAME_WAIT_FINISH) {
    return LP_GAME_INVALID_STATE;
  }
  if (timestamp_us < game->start_us) {
    return LP_GAME_TIMESTAMP_ERROR;
  }
  finish_attempt(game, LP_GAME_RESULT_FINISHED, timestamp_us);
  game->state = LP_GAME_WAIT_PLAYER;
  game->current_player[0] = '\0';
  return LP_GAME_OK;
}

lp_game_action_result_t lp_game_abort(lp_game_engine_t *game,
                                      uint64_t timestamp_us) {
  if (game->state != LP_GAME_WAIT_START &&
      game->state != LP_GAME_WAIT_FINISH) {
    return LP_GAME_INVALID_STATE;
  }
  if (game->state == LP_GAME_WAIT_FINISH && timestamp_us < game->start_us) {
    return LP_GAME_TIMESTAMP_ERROR;
  }
  finish_attempt(game, LP_GAME_RESULT_ABORTED_BY_OPERATOR, timestamp_us);
  game->state = LP_GAME_WAIT_PLAYER;
  game->current_player[0] = '\0';
  return LP_GAME_OK;
}

bool lp_game_tick(lp_game_engine_t *game, uint64_t timestamp_us) {
  if (game->state != LP_GAME_WAIT_FINISH || timestamp_us < game->start_us) {
    return false;
  }
  const uint64_t maximum_us = (uint64_t)game->settings.maximum_run_ms * 1000U;
  if (timestamp_us - game->start_us < maximum_us) {
    return false;
  }
  finish_attempt(game, LP_GAME_RESULT_ABORTED_TIMEOUT, timestamp_us);
  game->state = LP_GAME_WAIT_PLAYER;
  game->current_player[0] = '\0';
  return true;
}

void lp_game_fault(lp_game_engine_t *game, uint64_t timestamp_us) {
  if (game->state == LP_GAME_WAIT_START ||
      game->state == LP_GAME_WAIT_FINISH) {
    finish_attempt(game, LP_GAME_RESULT_ABORTED_FAULT, timestamp_us);
  }
  game->state = LP_GAME_FAULT;
}

const char *lp_game_state_name(lp_game_state_t state) {
  static const char *const names[] = {"SETUP", "WAIT_PLAYER", "WAIT_START",
                                      "WAIT_FINISH", "FAULT"};
  return state <= LP_GAME_FAULT ? names[state] : "UNKNOWN";
}
