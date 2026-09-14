#include "laser_result_store.h"

#include <stddef.h>
#include <string.h>

static bool ranks_before(const lp_stored_result_t *left,
                         const lp_stored_result_t *right) {
  if (left->result.score_time_us != right->result.score_time_us) {
    return left->result.score_time_us < right->result.score_time_us;
  }
  if (left->result.penalty_time_us != right->result.penalty_time_us) {
    return left->result.penalty_time_us < right->result.penalty_time_us;
  }
  return left->completion_sequence < right->completion_sequence;
}

void lp_result_store_init(lp_result_store_t *store) {
  memset(store, 0, sizeof(*store));
  store->next_completion_sequence = 1U;
}

bool lp_result_store_record(lp_result_store_t *store,
                            const lp_game_result_t *result) {
  if (store == NULL || result == NULL ||
      result->status == LP_GAME_RESULT_NONE) {
    return false;
  }

  lp_stored_result_t entry = {
      .result = *result,
      .completion_sequence = store->next_completion_sequence++,
  };
  const uint8_t recent_move = store->recent_count < LP_RESULT_STORE_CAPACITY
                                  ? store->recent_count
                                  : LP_RESULT_STORE_CAPACITY - 1U;
  if (recent_move != 0U) {
    memmove(&store->recent[1], &store->recent[0],
            (size_t)recent_move * sizeof(store->recent[0]));
  }
  store->recent[0] = entry;
  if (store->recent_count < LP_RESULT_STORE_CAPACITY) {
    ++store->recent_count;
  }

  if (result->status != LP_GAME_RESULT_FINISHED) {
    return true;
  }
  uint8_t position = 0U;
  while (position < store->top_count &&
         !ranks_before(&entry, &store->top[position])) {
    ++position;
  }
  if (position >= LP_RESULT_STORE_CAPACITY) {
    return true;
  }
  const uint8_t old_count = store->top_count;
  const uint8_t new_count = old_count < LP_RESULT_STORE_CAPACITY
                                ? (uint8_t)(old_count + 1U)
                                : old_count;
  const uint8_t move_count = new_count - position - 1U;
  if (move_count != 0U) {
    memmove(&store->top[position + 1U], &store->top[position],
            (size_t)move_count * sizeof(store->top[0]));
  }
  store->top[position] = entry;
  store->top_count = new_count;
  return true;
}
