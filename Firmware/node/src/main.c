#include "laser_protocol.h"
#include "usi_i2c_target.h"

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/atomic.h>

#if F_CPU != 1000000UL
#error "The node firmware currently expects a 1 MHz ATtiny85 system clock"
#endif

enum {
  ADC_SAMPLE_TICKS = 156U,
  SAMPLE_PERIOD_MS = 10U,
  BUTTON_RELEASE_SAMPLES = 5U,
  BUTTON_LONG_PRESS_SAMPLES = 300U,
};

static uint16_t EEMEM persisted_boot_counter;

typedef struct __attribute__((packed)) {
  uint8_t magic[2];
  uint8_t format_version;
  uint8_t address;
  uint8_t role;
  lp_u16_le_t broken_threshold;
  lp_u16_le_t hysteresis;
  lp_u16_le_t stable_time_ms;
  lp_u16_le_t cooldown_ms;
  uint8_t crc8;
} persisted_config_t;

static persisted_config_t EEMEM persisted_config;

static lp_identity_register_t identity = {
    .protocol_major = LP_PROTOCOL_MAJOR,
    .protocol_minor = LP_PROTOCOL_MINOR,
    .firmware_major = 0U,
    .firmware_minor = 2U,
    .firmware_patch = 0U,
    .role = LP_ROLE_UNCONFIGURED,
    .address = LP_ADDRESS_COMMISSIONING,
    .capabilities = {.low = (uint8_t)LP_CAPABILITY_STATUS_LED, .high = 0U},
    .config_format_version = LP_CONFIG_FORMAT_VERSION,
    .crc8 = 0U,
};

static lp_fast_status_register_t fast_status;
static lp_diagnostics_register_t diagnostics;
static lp_sensor_config_register_t active_sensor_config = {
    .broken_threshold = {.low = 240U, .high = 0U},
    .hysteresis = {.low = 16U, .high = 0U},
    .stable_time_ms = {.low = 30U, .high = 0U},
    .cooldown_ms = {.low = 0xF4U, .high = 0x01U},
    .crc8 = 0U,
};
static lp_sensor_config_register_t staged_sensor_config;
static lp_staged_identity_register_t staged_identity;
static lp_command_result_register_t command_result;
static uint8_t staged_sensor_valid;
static uint8_t staged_identity_valid;
static uint8_t config_valid;
static uint8_t last_result = LP_RESULT_OK;
static uint8_t node_error;
static lp_command_register_t cached_command;
static uint8_t cached_command_valid;
static persisted_config_t pending_config;
static uint8_t pending_config_activation;
static uint8_t pending_factory_reset;
static uint16_t event_counter;
static uint8_t counter_overflowed;
static uint8_t operating_mode = LP_MODE_SETUP;
static uint16_t identify_samples_remaining;
static uint8_t button_armed;
static uint8_t button_release_samples;
static uint16_t button_pressed_samples;
static uint8_t fu_pulse_active;
static uint8_t fu_pulse_started_tick;
static uint8_t button_led_state = LP_BUTTON_LED_STANDBY;

typedef enum {
  LED_STATE_UNINITIALIZED,
  LED_STATE_OFF,
  LED_STATE_STANDBY,
  LED_STATE_STEADY,
  LED_STATE_SLOW,
  LED_STATE_NORMAL,
  LED_STATE_FAST,
} led_state_t;

static led_state_t led_state = LED_STATE_UNINITIALIZED;

static uint8_t role_is_button(void);
static uint8_t button_is_pressed(void);

static const usi_i2c_register_t register_table[] = {
    {LP_REGISTER_IDENTITY, (const uint8_t *)&identity, sizeof(identity), 0U},
    {LP_REGISTER_FAST_STATUS, (const uint8_t *)&fast_status,
     sizeof(fast_status), 0U},
    {LP_REGISTER_DIAGNOSTICS, (const uint8_t *)&diagnostics,
     sizeof(diagnostics), 0U},
    {LP_REGISTER_ACTIVE_SENSOR_CONFIG,
     (const uint8_t *)&active_sensor_config, sizeof(active_sensor_config), 0U},
    {LP_REGISTER_STAGED_SENSOR_CONFIG,
     (const uint8_t *)&staged_sensor_config, sizeof(staged_sensor_config),
     sizeof(staged_sensor_config)},
    {LP_REGISTER_STAGED_IDENTITY, (const uint8_t *)&staged_identity,
     sizeof(staged_identity), sizeof(staged_identity)},
    {LP_REGISTER_COMMAND, 0, 0U, sizeof(lp_command_register_t)},
    {LP_REGISTER_COMMAND_RESULT, (const uint8_t *)&command_result,
     sizeof(command_result), 0U},
};

static void status_led_init(void) {
  // Uncommissioned nodes use the approximately 2 Hz normal-blink pattern.
  // Timer1 drives OC1A/PB1 directly without interrupts or software toggling.
  TCCR1 = 0U;
  GTCCR = _BV(PSR1);
  TCNT1 = 0U;
  OCR1C = 121U;
  OCR1A = 60U;
  GTCCR = 0U;
  DDRB |= _BV(PB1);
  PORTB &= (uint8_t)~_BV(PB1);
  // Inverting PWM starts the pattern with its off phase when TCNT1 is reset.
  TCCR1 = _BV(PWM1A) | _BV(COM1A1) | _BV(COM1A0) | _BV(CS13) |
          _BV(CS12) | _BV(CS10);
  led_state = LED_STATE_NORMAL;
}

static void status_led_set(led_state_t next) {
  if (next == led_state) {
    return;
  }
  led_state = next;

  TCCR1 = 0U;
  GTCCR = _BV(PSR1);
  TCNT1 = 0U;
  if (next == LED_STATE_OFF) {
    PORTB &= (uint8_t)~_BV(PB1);
    return;
  }
  if (next == LED_STATE_STEADY) {
    PORTB |= _BV(PB1);
    return;
  }

  if (next == LED_STATE_STANDBY) {
    // Fast hardware PWM: approximately 1.25 kHz with 5% LED-on duty cycle.
    OCR1C = 99U;
    OCR1A = 94U;
    GTCCR = 0U;
    PORTB &= (uint8_t)~_BV(PB1);
    TCCR1 = _BV(PWM1A) | _BV(COM1A1) | _BV(COM1A0) | _BV(CS12);
    return;
  }

  if (next == LED_STATE_SLOW) {
    OCR1C = 243U;
    OCR1A = 121U;
  } else if (next == LED_STATE_FAST) {
    OCR1C = 60U;
    OCR1A = 30U;
  } else {
    OCR1C = 121U;
    OCR1A = 60U;
  }
  GTCCR = 0U;
  PORTB &= (uint8_t)~_BV(PB1);
  // Inverting PWM makes a state transition immediately visible as LED off.
  TCCR1 = _BV(PWM1A) | _BV(COM1A1) | _BV(COM1A0) | _BV(CS13) |
          _BV(CS12) | _BV(CS10);
}

static void update_status_led(uint8_t beam_broken) {
  if (identify_samples_remaining > 0U) {
    status_led_set(LED_STATE_FAST);
  } else if (node_error != 0U || identity.role == LP_ROLE_UNCONFIGURED) {
    status_led_set(LED_STATE_NORMAL);
  } else if (role_is_button() && button_is_pressed()) {
    status_led_set(button_pressed_samples >= BUTTON_LONG_PRESS_SAMPLES
                       ? LED_STATE_SLOW
                       : LED_STATE_OFF);
  } else if (beam_broken && identity.role == LP_ROLE_LASER) {
    status_led_set(LED_STATE_SLOW);
  } else if (role_is_button() && operating_mode == LP_MODE_GAME &&
             button_led_state == LP_BUTTON_LED_STANDBY) {
    status_led_set(LED_STATE_STANDBY);
  } else if (role_is_button() && operating_mode == LP_MODE_GAME &&
             button_led_state == LP_BUTTON_LED_BLOCKED) {
    status_led_set(LED_STATE_SLOW);
  } else {
    status_led_set(LED_STATE_STEADY);
  }
}

static void adc_init(void) {
  // ADC3 is PB3. VCC is the reference and /8 gives a 125 kHz ADC clock.
  ADMUX = _BV(MUX1) | _BV(MUX0);
  DIDR0 = _BV(ADC3D);
  ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0);
}

static uint16_t adc_read(void) {
  ADCSRA |= _BV(ADSC);
  while ((ADCSRA & _BV(ADSC)) != 0U) {
  }
  return ADC;
}

static uint16_t increment_boot_counter(void) {
  uint16_t previous = eeprom_read_word(&persisted_boot_counter);
  if (previous == UINT16_MAX) {
    previous = 0U;
  }
  const uint16_t current = (uint16_t)(previous + 1U);
  eeprom_update_word(&persisted_boot_counter, current);
  return current;
}

static uint8_t role_is_button(void) {
  return identity.role == LP_ROLE_START || identity.role == LP_ROLE_FINISH;
}

static uint8_t button_is_pressed(void) {
  return (PINB & _BV(PB3)) == 0U;
}

static void role_io_init(void) {
  // FU is open drain: PORT remains low, DDR output asserts and DDR input
  // releases the externally pulled-up shared line.
  PORTB &= (uint8_t)~_BV(PB4);
  DDRB &= (uint8_t)~_BV(PB4);

  if (role_is_button()) {
    ADCSRA = 0U;
    DIDR0 &= (uint8_t)~_BV(ADC3D);
    DDRB &= (uint8_t)~_BV(PB3);
    PORTB |= _BV(PB3);
    button_armed = button_is_pressed() ? 0U : 1U;
    button_pressed_samples = 0U;
  } else {
    adc_init();
  }
}

static void update_fu_pulse(void) {
  if (fu_pulse_active != 0U &&
      (uint8_t)(TCNT0 - fu_pulse_started_tick) >= ADC_SAMPLE_TICKS) {
    DDRB &= (uint8_t)~_BV(PB4);
    fu_pulse_active = 0U;
  }
}

static void process_button_edge(void) {
  if (!role_is_button()) {
    return;
  }
  if (button_armed != 0U && button_is_pressed()) {
    ++event_counter;
    if (event_counter == 0U) {
      counter_overflowed = 1U;
    }
    button_armed = 0U;
    button_release_samples = 0U;
    button_pressed_samples = 0U;
    fu_pulse_started_tick = TCNT0;
    fu_pulse_active = 1U;
    DDRB |= _BV(PB4);
  }
}

static uint8_t sensor_config_is_valid(
    const lp_sensor_config_register_t *config) {
  const uint16_t threshold = lp_u16_decode(config->broken_threshold);
  const uint16_t hysteresis = lp_u16_decode(config->hysteresis);
  return threshold <= LP_SENSOR_THRESHOLD_MAX &&
         hysteresis <= LP_SENSOR_HYSTERESIS_MAX && hysteresis <= threshold &&
         lp_u16_decode(config->stable_time_ms) <=
             LP_SENSOR_STABLE_TIME_MAX_MS &&
         lp_u16_decode(config->cooldown_ms) <= LP_SENSOR_COOLDOWN_MAX_MS;
}

static uint8_t persisted_config_is_valid(const persisted_config_t *config) {
  if (config->magic[0] != (uint8_t)'L' ||
      config->magic[1] != (uint8_t)'P' ||
      config->format_version != LP_CONFIG_FORMAT_VERSION ||
      config->address < LP_ADDRESS_NORMAL_MIN ||
      config->address > LP_ADDRESS_NORMAL_MAX ||
      config->role < LP_ROLE_LASER || config->role > LP_ROLE_FINISH ||
      config->crc8 !=
          lp_crc8((const uint8_t *)config, sizeof(*config) - 1U)) {
    return 0U;
  }

  if (config->role == LP_ROLE_LASER) {
    lp_sensor_config_register_t sensor = {
        .broken_threshold = config->broken_threshold,
        .hysteresis = config->hysteresis,
        .stable_time_ms = config->stable_time_ms,
        .cooldown_ms = config->cooldown_ms,
        .crc8 = 0U,
    };
    return sensor_config_is_valid(&sensor);
  }
  return 1U;
}

static uint16_t capabilities_for_role(uint8_t role) {
  uint16_t capabilities = LP_CAPABILITY_STATUS_LED;
  if (role == LP_ROLE_LASER) {
    capabilities |= LP_CAPABILITY_LASER_SENSING;
  } else if (role == LP_ROLE_START || role == LP_ROLE_FINISH) {
    capabilities |= LP_CAPABILITY_BUTTON_INPUT | LP_CAPABILITY_FU_OUTPUT;
  }
  return capabilities;
}

static void prepare_command_result(uint8_t sequence, uint8_t command,
                                   uint8_t result, uint16_t detail) {
  lp_command_result_register_t next = {
      .sequence = sequence,
      .command = command,
      .result = result,
      .detail = lp_u16_encode(detail),
      .crc8 = 0U,
  };
  next.crc8 = lp_register_crc8(
      LP_REGISTER_COMMAND_RESULT, (const uint8_t *)&next,
      sizeof(next) - sizeof(next.crc8));
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { command_result = next; }
  last_result = result;
  if (result == LP_RESULT_EEPROM_FAILURE) {
    node_error = 1U;
  } else if (result == LP_RESULT_OK) {
    node_error = 0U;
  }
}

static void execute_save_config(const lp_command_register_t *command) {
  if (command->arguments[0] != 0U || command->arguments[1] != 0U ||
      command->arguments[2] != 0U || command->arguments[3] != 0U) {
    prepare_command_result(command->sequence, command->command,
                           LP_RESULT_INVALID_VALUE, 0U);
    return;
  }
  if (staged_identity_valid == 0U && staged_sensor_valid == 0U) {
    prepare_command_result(command->sequence, command->command,
                           LP_RESULT_NO_STAGED_CONFIG, 0U);
    return;
  }

  persisted_config_t candidate = {
      .magic = {(uint8_t)'L', (uint8_t)'P'},
      .format_version = LP_CONFIG_FORMAT_VERSION,
      .address = staged_identity_valid ? staged_identity.address
                                       : identity.address,
      .role = staged_identity_valid ? staged_identity.role : identity.role,
      .broken_threshold = staged_sensor_valid
                              ? staged_sensor_config.broken_threshold
                              : active_sensor_config.broken_threshold,
      .hysteresis = staged_sensor_valid ? staged_sensor_config.hysteresis
                                        : active_sensor_config.hysteresis,
      .stable_time_ms = staged_sensor_valid
                            ? staged_sensor_config.stable_time_ms
                            : active_sensor_config.stable_time_ms,
      .cooldown_ms = staged_sensor_valid ? staged_sensor_config.cooldown_ms
                                         : active_sensor_config.cooldown_ms,
      .crc8 = 0U,
  };
  if (candidate.role == LP_ROLE_UNCONFIGURED ||
      (candidate.role == LP_ROLE_LASER && staged_sensor_valid == 0U &&
       config_valid == 0U)) {
    prepare_command_result(command->sequence, command->command,
                           LP_RESULT_INVALID_VALUE, 0U);
    return;
  }
  candidate.crc8 = lp_crc8((const uint8_t *)&candidate,
                           sizeof(candidate) - 1U);

  eeprom_update_block(&candidate, &persisted_config, sizeof(candidate));
  persisted_config_t readback;
  eeprom_read_block(&readback, &persisted_config, sizeof(readback));
  if (!persisted_config_is_valid(&readback) ||
      __builtin_memcmp(&candidate, &readback, sizeof(candidate)) != 0) {
    prepare_command_result(command->sequence, command->command,
                           LP_RESULT_EEPROM_FAILURE, 0U);
    return;
  }

  pending_config = readback;
  pending_config_activation = 1U;
  prepare_command_result(command->sequence, command->command, LP_RESULT_OK,
                         (uint16_t)readback.address |
                             ((uint16_t)readback.crc8 << 8U));
}

static void execute_command(const lp_command_register_t *command) {
  const uint8_t expected_crc = lp_register_crc8(
      LP_REGISTER_COMMAND, (const uint8_t *)command,
      sizeof(*command) - sizeof(command->crc8));
  if (command->crc8 != expected_crc) {
    prepare_command_result(command->sequence, command->command,
                           LP_RESULT_INVALID_CRC, 0U);
    return;
  }

  if (cached_command_valid != 0U &&
      command->sequence == cached_command.sequence) {
    if (__builtin_memcmp(command, &cached_command, sizeof(*command)) != 0) {
      prepare_command_result(command->sequence, command->command,
                             LP_RESULT_SEQUENCE_CONFLICT, 0U);
    }
    return;
  }
  cached_command = *command;
  cached_command_valid = 1U;

  switch (command->command) {
    case LP_COMMAND_SET_MODE:
      if (command->arguments[1] != 0U || command->arguments[2] != 0U ||
          command->arguments[3] != 0U ||
          command->arguments[0] > LP_MODE_GAME) {
        prepare_command_result(command->sequence, command->command,
                               LP_RESULT_INVALID_VALUE, 0U);
      } else if (command->arguments[0] == LP_MODE_GAME &&
                 config_valid == 0U) {
        prepare_command_result(command->sequence, command->command,
                               LP_RESULT_WRONG_MODE, 0U);
      } else {
        operating_mode = command->arguments[0];
        if (operating_mode == LP_MODE_SETUP) {
          button_led_state = LP_BUTTON_LED_STANDBY;
        }
        identify_samples_remaining = 0U;
        prepare_command_result(command->sequence, command->command,
                               LP_RESULT_OK, operating_mode);
      }
      break;

    case LP_COMMAND_SET_BUTTON_LED:
      if (!role_is_button()) {
        prepare_command_result(command->sequence, command->command,
                               LP_RESULT_WRONG_ROLE, 0U);
      } else if (operating_mode != LP_MODE_GAME) {
        prepare_command_result(command->sequence, command->command,
                               LP_RESULT_WRONG_MODE, 0U);
      } else if (command->arguments[0] > LP_BUTTON_LED_BLOCKED ||
                 command->arguments[1] != 0U || command->arguments[2] != 0U ||
                 command->arguments[3] != 0U) {
        prepare_command_result(command->sequence, command->command,
                               LP_RESULT_INVALID_VALUE, 0U);
      } else {
        button_led_state = command->arguments[0];
        prepare_command_result(command->sequence, command->command,
                               LP_RESULT_OK, button_led_state);
      }
      break;

    case LP_COMMAND_SAVE_CONFIG:
      if (operating_mode != LP_MODE_SETUP) {
        prepare_command_result(command->sequence, command->command,
                               LP_RESULT_WRONG_MODE, 0U);
      } else {
        execute_save_config(command);
      }
      break;

    case LP_COMMAND_RESET_COUNTER:
      if (operating_mode != LP_MODE_SETUP && !role_is_button()) {
        prepare_command_result(command->sequence, command->command,
                               LP_RESULT_WRONG_MODE, 0U);
      } else if (command->arguments[2] != 0U ||
                 command->arguments[3] != 0U) {
        prepare_command_result(command->sequence, command->command,
                               LP_RESULT_INVALID_VALUE, 0U);
      } else {
        event_counter = 0U;
        counter_overflowed = 0U;
        prepare_command_result(command->sequence, command->command,
                               LP_RESULT_OK, 0U);
      }
      break;

    case LP_COMMAND_IDENTIFY:
      if (operating_mode != LP_MODE_SETUP) {
        prepare_command_result(command->sequence, command->command,
                               LP_RESULT_WRONG_MODE, 0U);
      } else if (command->arguments[0] != 0U ||
                 command->arguments[1] != 0U ||
                 command->arguments[2] != 0U ||
                 command->arguments[3] != 0U) {
        prepare_command_result(command->sequence, command->command,
                               LP_RESULT_INVALID_VALUE, 0U);
      } else {
        identify_samples_remaining =
            (LP_IDENTIFY_DURATION_SECONDS * 1000U) / SAMPLE_PERIOD_MS;
        prepare_command_result(command->sequence, command->command,
                               LP_RESULT_OK, LP_IDENTIFY_DURATION_SECONDS);
      }
      break;

    case LP_COMMAND_FACTORY_RESET:
      if (operating_mode != LP_MODE_SETUP) {
        prepare_command_result(command->sequence, command->command,
                               LP_RESULT_WRONG_MODE, 0U);
      } else if (command->arguments[0] != LP_FACTORY_RESET_ARG0 ||
                 command->arguments[1] != LP_FACTORY_RESET_ARG1 ||
                 command->arguments[2] != LP_FACTORY_RESET_ARG2 ||
                 command->arguments[3] != LP_FACTORY_RESET_ARG3) {
        prepare_command_result(command->sequence, command->command,
                               LP_RESULT_PROTECTION_FAILED, 0U);
      } else {
        pending_factory_reset = 1U;
        prepare_command_result(command->sequence, command->command,
                               LP_RESULT_OK, LP_ADDRESS_COMMISSIONING);
      }
      break;

    default:
      prepare_command_result(command->sequence, command->command,
                             LP_RESULT_UNKNOWN_COMMAND, 0U);
      break;
  }
}

static void activate_pending_config(void) {
  identity.address = pending_config.address;
  identity.role = pending_config.role;
  identity.capabilities =
      lp_u16_encode(capabilities_for_role(pending_config.role));
  identity.crc8 = lp_register_crc8(
      LP_REGISTER_IDENTITY, (const uint8_t *)&identity,
      sizeof(identity) - sizeof(identity.crc8));

  active_sensor_config.broken_threshold = pending_config.broken_threshold;
  active_sensor_config.hysteresis = pending_config.hysteresis;
  active_sensor_config.stable_time_ms = pending_config.stable_time_ms;
  active_sensor_config.cooldown_ms = pending_config.cooldown_ms;
  active_sensor_config.crc8 = lp_register_crc8(
      LP_REGISTER_ACTIVE_SENSOR_CONFIG,
      (const uint8_t *)&active_sensor_config,
      sizeof(active_sensor_config) - sizeof(active_sensor_config.crc8));
  staged_identity_valid = 0U;
  staged_sensor_valid = 0U;
  config_valid = 1U;
  pending_config_activation = 0U;
  usi_i2c_target_set_address(identity.address);
  role_io_init();
}

static void process_pending_write(void) {
  uint8_t register_address = 0U;
  uint8_t bytes[LP_SENSOR_CONFIG_REGISTER_SIZE];
  const uint8_t length =
      usi_i2c_target_take_write(&register_address, bytes, sizeof(bytes));
  if (length == 0U) {
    return;
  }

  const uint8_t expected_crc =
      lp_register_crc8(register_address, bytes, (size_t)length - 1U);
  if (bytes[length - 1U] != expected_crc) {
    last_result = LP_RESULT_INVALID_CRC;
    return;
  }

  if (register_address == LP_REGISTER_STAGED_IDENTITY &&
      length == sizeof(lp_staged_identity_register_t)) {
    const lp_staged_identity_register_t *candidate =
        (const lp_staged_identity_register_t *)bytes;
    if (operating_mode != LP_MODE_SETUP) {
      last_result = LP_RESULT_WRONG_MODE;
    } else if (identity.role != LP_ROLE_UNCONFIGURED ||
        identity.address != LP_ADDRESS_COMMISSIONING) {
      last_result = LP_RESULT_NOT_COMMISSIONING;
    } else if (candidate->address < LP_ADDRESS_NORMAL_MIN ||
               candidate->address > LP_ADDRESS_NORMAL_MAX ||
               candidate->role < LP_ROLE_LASER ||
               candidate->role > LP_ROLE_FINISH) {
      last_result = LP_RESULT_INVALID_VALUE;
    } else {
      staged_identity = *candidate;
      staged_identity_valid = 1U;
      last_result = LP_RESULT_OK;
    }
    return;
  }

  if (register_address == LP_REGISTER_STAGED_SENSOR_CONFIG &&
      length == sizeof(lp_sensor_config_register_t)) {
    const lp_sensor_config_register_t *candidate =
        (const lp_sensor_config_register_t *)bytes;
    if (operating_mode != LP_MODE_SETUP) {
      last_result = LP_RESULT_WRONG_MODE;
    } else if (identity.role != LP_ROLE_LASER &&
        (staged_identity_valid == 0U ||
         staged_identity.role != LP_ROLE_LASER)) {
      last_result = LP_RESULT_WRONG_ROLE;
    } else if (!sensor_config_is_valid(candidate)) {
      last_result = LP_RESULT_INVALID_VALUE;
    } else {
      staged_sensor_config = *candidate;
      staged_sensor_valid = 1U;
      last_result = LP_RESULT_OK;
    }
    return;
  }

  if (register_address == LP_REGISTER_COMMAND &&
      length == sizeof(lp_command_register_t)) {
    execute_command((const lp_command_register_t *)bytes);
    return;
  }

  last_result = LP_RESULT_UNKNOWN_REGISTER;
}

static void publish_registers(uint16_t boot_counter, uint16_t event_counter,
                              uint16_t status_flags, uint16_t raw_adc,
                              uint16_t filtered_adc, uint8_t input_state,
                              uint16_t cooldown_samples) {
  lp_fast_status_register_t next_fast = {
      .boot_counter = lp_u16_encode(boot_counter),
      .event_counter = lp_u16_encode(event_counter),
      .status_flags = lp_u16_encode(status_flags),
      .crc8 = 0U,
  };
  next_fast.crc8 = lp_register_crc8(
      LP_REGISTER_FAST_STATUS, (const uint8_t *)&next_fast,
      sizeof(next_fast) - sizeof(next_fast.crc8));

  lp_diagnostics_register_t next_diagnostics = {
      .raw_adc = lp_u16_encode(raw_adc),
      .filtered_adc = lp_u16_encode(filtered_adc),
      .input_state = input_state,
      .operating_mode = operating_mode,
      .last_result = last_result,
      .cooldown_remaining_ms =
          lp_u16_encode(cooldown_samples * SAMPLE_PERIOD_MS),
      .crc8 = 0U,
  };
  next_diagnostics.crc8 = lp_register_crc8(
      LP_REGISTER_DIAGNOSTICS, (const uint8_t *)&next_diagnostics,
      sizeof(next_diagnostics) - sizeof(next_diagnostics.crc8));

  // The USI ISR snapshots a whole published block before transmitting it.
  // Prevent it from starting that copy while either block is being replaced.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    fast_status = next_fast;
    diagnostics = next_diagnostics;
  }
}

int main(void) {
  // A watchdog reset leaves the watchdog running at its fastest prescaler.
  // This is intentionally the first application action, before EEPROM access.
  wdt_reset();
  MCUSR = 0U;
  wdt_disable();

  persisted_config_t loaded_config;
  eeprom_read_block(&loaded_config, &persisted_config, sizeof(loaded_config));
  if (persisted_config_is_valid(&loaded_config)) {
    pending_config = loaded_config;
    pending_config_activation = 1U;
    activate_pending_config();
  }

  status_led_init();
  TCCR0A = 0U;
  TCCR0B = _BV(CS01) | _BV(CS00);  // /64: 64 us per Timer0 tick.
  role_io_init();

  const uint16_t boot_counter = increment_boot_counter();
  event_counter = 0U;
  uint16_t raw_adc = role_is_button() ? 0U : adc_read();
  uint16_t filtered_adc = raw_adc;
  uint8_t beam_broken = role_is_button()
                            ? 0U
                            : filtered_adc >= lp_u16_decode(
                                                  active_sensor_config
                                                      .broken_threshold);
  uint8_t pending_state = beam_broken;
  uint8_t pending_samples = 0U;
  uint16_t cooldown_samples = 0U;
  counter_overflowed = 0U;

  identity.crc8 = lp_register_crc8(
      LP_REGISTER_IDENTITY, (const uint8_t *)&identity,
      sizeof(identity) - sizeof(identity.crc8));
  active_sensor_config.crc8 = lp_register_crc8(
      LP_REGISTER_ACTIVE_SENSOR_CONFIG,
      (const uint8_t *)&active_sensor_config,
      sizeof(active_sensor_config) - sizeof(active_sensor_config.crc8));
  staged_sensor_config.crc8 = lp_register_crc8(
      LP_REGISTER_STAGED_SENSOR_CONFIG,
      (const uint8_t *)&staged_sensor_config,
      sizeof(staged_sensor_config) - sizeof(staged_sensor_config.crc8));
  staged_identity.crc8 = lp_register_crc8(
      LP_REGISTER_STAGED_IDENTITY, (const uint8_t *)&staged_identity,
      sizeof(staged_identity) - sizeof(staged_identity.crc8));
  prepare_command_result(0U, 0U, LP_RESULT_BUSY, 0U);
  // BUSY describes the empty mailbox, not a failed node operation.
  last_result = LP_RESULT_OK;
  publish_registers(boot_counter, event_counter, 0U, raw_adc, filtered_adc,
                    beam_broken ? LP_INPUT_ACTIVE : LP_INPUT_INACTIVE,
                    cooldown_samples);
  usi_i2c_target_init(identity.address, register_table,
                      sizeof(register_table) / sizeof(register_table[0]));
  sei();

  uint8_t last_sample_tick = TCNT0;
  for (;;) {
    process_pending_write();
    update_fu_pulse();
    process_button_edge();
    update_status_led(beam_broken);
    uint8_t completed_register = 0U;
    uint8_t completed_data[LP_COMMAND_RESULT_REGISTER_SIZE];
    const uint8_t completed_length = usi_i2c_target_take_completed_read(
        &completed_register, completed_data, sizeof(completed_data));
    const lp_command_result_register_t *completed_result =
        (const lp_command_result_register_t *)completed_data;
    if (completed_length == sizeof(*completed_result) &&
        completed_register == LP_REGISTER_COMMAND_RESULT &&
        pending_config_activation != 0U &&
        completed_result->result == LP_RESULT_OK &&
        completed_result->command == LP_COMMAND_SAVE_CONFIG) {
      activate_pending_config();
    }
    if (completed_length == sizeof(*completed_result) &&
        completed_register == LP_REGISTER_COMMAND_RESULT &&
        pending_factory_reset != 0U &&
        completed_result->result == LP_RESULT_OK &&
        completed_result->command == LP_COMMAND_FACTORY_RESET) {
      // Invalidating the first magic byte is sufficient to make the complete
      // record unusable after restart. Boot history remains intact.
      eeprom_update_byte(&persisted_config.magic[0], UINT8_MAX);
      wdt_enable(WDTO_15MS);
      for (;;) {
      }
    }
    const uint8_t now = TCNT0;
    if ((uint8_t)(now - last_sample_tick) < ADC_SAMPLE_TICKS) {
      continue;
    }
    last_sample_tick = (uint8_t)(last_sample_tick + ADC_SAMPLE_TICKS);

    if (cooldown_samples > 0U && !role_is_button()) {
      --cooldown_samples;
    }
    if (identify_samples_remaining > 0U) {
      --identify_samples_remaining;
    }

    if (role_is_button()) {
      raw_adc = 0U;
      filtered_adc = 0U;
      cooldown_samples = 0U;
      if (!button_is_pressed()) {
        button_pressed_samples = 0U;
        if (button_release_samples < BUTTON_RELEASE_SAMPLES) {
          ++button_release_samples;
        }
        if (button_release_samples >= BUTTON_RELEASE_SAMPLES) {
          button_armed = 1U;
        }
      } else {
        button_release_samples = 0U;
        if (button_pressed_samples < BUTTON_LONG_PRESS_SAMPLES) {
          ++button_pressed_samples;
        }
      }
    } else {
      raw_adc = adc_read();
      filtered_adc = (uint16_t)((filtered_adc * 15UL + raw_adc) / 16UL);
    }

    const uint16_t broken_threshold =
        lp_u16_decode(active_sensor_config.broken_threshold);
    const uint16_t clear_threshold =
        broken_threshold - lp_u16_decode(active_sensor_config.hysteresis);
    uint8_t stable_samples = (uint8_t)(
        (lp_u16_decode(active_sensor_config.stable_time_ms) +
         SAMPLE_PERIOD_MS - 1U) /
        SAMPLE_PERIOD_MS);
    if (stable_samples == 0U) {
      stable_samples = 1U;
    }

    uint8_t candidate = beam_broken;
    if (!role_is_button() && !beam_broken &&
        filtered_adc >= broken_threshold) {
      candidate = 1U;
    } else if (!role_is_button() && beam_broken &&
               filtered_adc <= clear_threshold) {
      candidate = 0U;
    }

    uint8_t input_state = role_is_button()
                              ? (button_is_pressed() ? LP_INPUT_ACTIVE
                                                     : LP_INPUT_INACTIVE)
                              : (beam_broken ? LP_INPUT_ACTIVE
                                             : LP_INPUT_INACTIVE);
    if (!role_is_button() && candidate != beam_broken) {
      if (candidate != pending_state) {
        pending_state = candidate;
        pending_samples = 1U;
      } else if (pending_samples < stable_samples) {
        ++pending_samples;
      }
      input_state = LP_INPUT_UNSTABLE;

      if (pending_samples >= stable_samples) {
        beam_broken = candidate;
        pending_samples = 0U;
        input_state =
            beam_broken ? LP_INPUT_ACTIVE : LP_INPUT_INACTIVE;
        if (beam_broken && cooldown_samples == 0U) {
          ++event_counter;
          if (event_counter == 0U) {
            counter_overflowed = 1U;
          }
          cooldown_samples =
              (lp_u16_decode(active_sensor_config.cooldown_ms) +
               SAMPLE_PERIOD_MS - 1U) /
              SAMPLE_PERIOD_MS;
        }
      }
    } else if (!role_is_button()) {
      pending_state = beam_broken;
      pending_samples = 0U;
    }

    uint16_t status_flags = 0U;
    if (config_valid) {
      status_flags |= LP_STATUS_CONFIG_VALID | LP_STATUS_COMMISSIONED;
    }
    if (operating_mode == LP_MODE_GAME) {
      status_flags |= LP_STATUS_GAME_MODE;
    }
    if ((role_is_button() && button_is_pressed()) ||
        (!role_is_button() && beam_broken)) {
      status_flags |= LP_STATUS_INPUT_ACTIVE;
    }
    if (input_state == LP_INPUT_UNSTABLE) {
      status_flags |= LP_STATUS_INPUT_UNSTABLE;
    }
    if (cooldown_samples > 0U) {
      status_flags |= LP_STATUS_COOLDOWN_ACTIVE;
    }
    if (counter_overflowed) {
      status_flags |= LP_STATUS_COUNTER_OVERFLOWED;
    }
    if (staged_sensor_valid) {
      status_flags |= LP_STATUS_CONFIG_STAGED;
    }
    if (staged_identity_valid) {
      status_flags |= LP_STATUS_IDENTITY_STAGED;
    }
    if (last_result != LP_RESULT_OK) {
      status_flags |= LP_STATUS_LAST_OPERATION_ERROR;
    }

    publish_registers(boot_counter, event_counter, status_flags, raw_adc,
                      filtered_adc, input_state, cooldown_samples);
  }
}
