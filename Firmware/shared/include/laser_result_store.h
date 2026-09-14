#ifndef LASER_RESULT_STORE_H
#define LASER_RESULT_STORE_H

#include <stdbool.h>
#include <stdint.h>

#include "laser_game_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LP_RESULT_STORE_CAPACITY 10U

typedef struct {
  lp_game_result_t result;
  uint64_t completion_sequence;
} lp_stored_result_t;

typedef struct {
  lp_stored_result_t recent[LP_RESULT_STORE_CAPACITY];
  lp_stored_result_t top[LP_RESULT_STORE_CAPACITY];
  uint8_t recent_count;
  uint8_t top_count;
  uint64_t next_completion_sequence;
} lp_result_store_t;

void lp_result_store_init(lp_result_store_t *store);
bool lp_result_store_record(lp_result_store_t *store,
                            const lp_game_result_t *result);

#ifdef __cplusplus
}
#endif

#endif
