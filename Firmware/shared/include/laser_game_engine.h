#ifndef LASER_GAME_ENGINE_H
#define LASER_GAME_ENGINE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LP_GAME_PLAYER_NAME_BYTES 32U

typedef enum {
  LP_GAME_SETUP = 0,
  LP_GAME_WAIT_PLAYER,
  LP_GAME_WAIT_START,
  LP_GAME_WAIT_FINISH,
  LP_GAME_FAULT,
} lp_game_state_t;

typedef enum {
  LP_GAME_RESULT_NONE = 0,
  LP_GAME_RESULT_FINISHED,
  LP_GAME_RESULT_ABORTED_BY_OPERATOR,
  LP_GAME_RESULT_ABORTED_TIMEOUT,
  LP_GAME_RESULT_ABORTED_FAULT,
} lp_game_result_status_t;

typedef enum {
  LP_GAME_OK = 0,
  LP_GAME_INVALID_STATE,
  LP_GAME_INVALID_PLAYER,
  LP_GAME_BEAM_BLOCKED,
  LP_GAME_TIMESTAMP_ERROR,
} lp_game_action_result_t;

typedef struct {
  uint32_t penalty_ms;
  uint32_t maximum_run_ms;
} lp_game_settings_t;

typedef struct {
  lp_game_result_status_t status;
  char player[LP_GAME_PLAYER_NAME_BYTES + 1U];
  uint64_t start_us;
  uint64_t end_us;
  uint64_t raw_time_us;
  uint64_t penalty_time_us;
  uint64_t score_time_us;
  uint32_t interruptions;
} lp_game_result_t;

typedef struct {
  lp_game_state_t state;
  lp_game_settings_t settings;
  char current_player[LP_GAME_PLAYER_NAME_BYTES + 1U];
  uint64_t start_us;
  uint32_t interruptions;
  lp_game_result_t last_result;
} lp_game_engine_t;

void lp_game_init(lp_game_engine_t *game, lp_game_settings_t settings);
lp_game_action_result_t lp_game_enter_game(lp_game_engine_t *game,
                                            bool setup_valid);
void lp_game_enter_setup(lp_game_engine_t *game);
lp_game_action_result_t lp_game_set_player(lp_game_engine_t *game,
                                           const char *player);
lp_game_action_result_t lp_game_start(lp_game_engine_t *game,
                                      uint64_t timestamp_us,
                                      bool all_beams_clear);
lp_game_action_result_t lp_game_add_interruptions(lp_game_engine_t *game,
                                                  uint32_t count);
lp_game_action_result_t lp_game_finish(lp_game_engine_t *game,
                                       uint64_t timestamp_us);
lp_game_action_result_t lp_game_abort(lp_game_engine_t *game,
                                      uint64_t timestamp_us);
bool lp_game_tick(lp_game_engine_t *game, uint64_t timestamp_us);
void lp_game_fault(lp_game_engine_t *game, uint64_t timestamp_us);
const char *lp_game_state_name(lp_game_state_t state);

#ifdef __cplusplus
}
#endif

#endif
