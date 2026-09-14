#ifndef LASER_TOP_STORAGE_H
#define LASER_TOP_STORAGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "laser_result_store.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LP_TOP_STORAGE_FORMAT_VERSION 1U
#define LP_TOP_STORAGE_ENCODED_SIZE 880U

bool lp_top_storage_encode(const lp_result_store_t *store, uint8_t *output,
                           size_t output_size);
bool lp_top_storage_decode(lp_result_store_t *store, const uint8_t *input,
                           size_t input_size);

#ifdef __cplusplus
}
#endif

#endif
