#include "laser_top_storage.h"

#include <string.h>

#define LP_TOP_STORAGE_HEADER_SIZE 16U
#define LP_TOP_STORAGE_ENTRY_SIZE 97U
#define LP_TOP_STORAGE_CRC_SIZE 4U

static void write_u32(uint8_t *output, uint32_t value) {
  for (uint8_t index = 0U; index < 4U; ++index) {
    output[index] = (uint8_t)(value >> (8U * index));
  }
}

static void write_u64(uint8_t *output, uint64_t value) {
  for (uint8_t index = 0U; index < 8U; ++index) {
    output[index] = (uint8_t)(value >> (8U * index));
  }
}

static uint32_t read_u32(const uint8_t *input) {
  uint32_t value = 0U;
  for (uint8_t index = 0U; index < 4U; ++index) {
    value |= (uint32_t)input[index] << (8U * index);
  }
  return value;
}

static uint64_t read_u64(const uint8_t *input) {
  uint64_t value = 0U;
  for (uint8_t index = 0U; index < 8U; ++index) {
    value |= (uint64_t)input[index] << (8U * index);
  }
  return value;
}

static void write_i16(uint8_t *output, int16_t value) {
  output[0] = (uint8_t)value;
  output[1] = (uint8_t)((uint16_t)value >> 8U);
}

static int16_t read_i16(const uint8_t *input) {
  return (int16_t)((uint16_t)input[0] | (uint16_t)input[1] << 8U);
}

static uint32_t crc32(const uint8_t *data, size_t length) {
  uint32_t crc = UINT32_MAX;
  for (size_t index = 0U; index < length; ++index) {
    crc ^= data[index];
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0xEDB88320U : crc >> 1U;
    }
  }
  return ~crc;
}

static void encode_entry(uint8_t *output, const lp_stored_result_t *entry) {
  output[0] = (uint8_t)entry->result.status;
  memcpy(&output[1], entry->result.player, LP_GAME_PLAYER_NAME_BYTES + 1U);
  write_u64(&output[34], entry->result.start_us);
  write_u64(&output[42], entry->result.end_us);
  write_u64(&output[50], entry->result.raw_time_us);
  write_u64(&output[58], entry->result.penalty_time_us);
  write_u64(&output[66], entry->result.score_time_us);
  write_u32(&output[74], entry->result.interruptions);
  write_u64(&output[78], entry->completion_sequence);
  write_u64(&output[86], entry->result.completion_unix_s);
  write_i16(&output[94], entry->result.utc_offset_minutes);
  output[96] = entry->result.timestamp_valid ? 1U : 0U;
}

bool lp_top_storage_encode(const lp_result_store_t *store, uint8_t *output,
                           size_t output_size) {
  if (store == NULL || output == NULL ||
      output_size != LP_TOP_STORAGE_ENCODED_SIZE ||
      store->top_count > LP_RESULT_STORE_CAPACITY) {
    return false;
  }
  memset(output, 0, output_size);
  memcpy(output, "LPT2", 4U);
  output[4] = LP_TOP_STORAGE_FORMAT_VERSION;
  output[5] = store->top_count;
  write_u64(&output[8], store->next_completion_sequence);
  for (uint8_t index = 0U; index < store->top_count; ++index) {
    encode_entry(&output[LP_TOP_STORAGE_HEADER_SIZE +
                         (size_t)index * LP_TOP_STORAGE_ENTRY_SIZE],
                 &store->top[index]);
  }
  write_u32(&output[output_size - LP_TOP_STORAGE_CRC_SIZE],
            crc32(output, output_size - LP_TOP_STORAGE_CRC_SIZE));
  return true;
}

static bool decode_entry(lp_stored_result_t *entry, const uint8_t *input) {
  if (input[0] != LP_GAME_RESULT_FINISHED ||
      input[1U + LP_GAME_PLAYER_NAME_BYTES] != '\0') {
    return false;
  }
  memset(entry, 0, sizeof(*entry));
  entry->result.status = (lp_game_result_status_t)input[0];
  memcpy(entry->result.player, &input[1], LP_GAME_PLAYER_NAME_BYTES + 1U);
  entry->result.start_us = read_u64(&input[34]);
  entry->result.end_us = read_u64(&input[42]);
  entry->result.raw_time_us = read_u64(&input[50]);
  entry->result.penalty_time_us = read_u64(&input[58]);
  entry->result.score_time_us = read_u64(&input[66]);
  entry->result.interruptions = read_u32(&input[74]);
  entry->completion_sequence = read_u64(&input[78]);
  entry->result.completion_unix_s = read_u64(&input[86]);
  entry->result.utc_offset_minutes = read_i16(&input[94]);
  entry->result.timestamp_valid = input[96] != 0U;
  if (input[96] > 1U ||
      (entry->result.timestamp_valid &&
       (entry->result.completion_unix_s == 0U ||
        entry->result.utc_offset_minutes < -840 ||
        entry->result.utc_offset_minutes > 840))) {
    return false;
  }
  return entry->completion_sequence != 0U;
}

bool lp_top_storage_decode(lp_result_store_t *store, const uint8_t *input,
                           size_t input_size) {
  if (store == NULL || input == NULL ||
      input_size != LP_TOP_STORAGE_ENCODED_SIZE ||
      memcmp(input, "LPT2", 4U) != 0 ||
      input[4] != LP_TOP_STORAGE_FORMAT_VERSION ||
      input[5] > LP_RESULT_STORE_CAPACITY ||
      read_u32(&input[input_size - LP_TOP_STORAGE_CRC_SIZE]) !=
          crc32(input, input_size - LP_TOP_STORAGE_CRC_SIZE)) {
    return false;
  }

  lp_result_store_t decoded;
  lp_result_store_init(&decoded);
  decoded.top_count = input[5];
  decoded.next_completion_sequence = read_u64(&input[8]);
  if (decoded.next_completion_sequence == 0U) {
    return false;
  }
  for (uint8_t index = 0U; index < decoded.top_count; ++index) {
    if (!decode_entry(&decoded.top[index],
                      &input[LP_TOP_STORAGE_HEADER_SIZE +
                             (size_t)index * LP_TOP_STORAGE_ENTRY_SIZE]) ||
        decoded.top[index].completion_sequence >=
            decoded.next_completion_sequence) {
      return false;
    }
  }
  *store = decoded;
  return true;
}
