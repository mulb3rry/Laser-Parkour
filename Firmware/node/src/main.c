#include "laser_protocol.h"
#include "usi_i2c_target.h"

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

#if F_CPU != 1000000UL
#error "The node firmware currently expects a 1 MHz ATtiny85 system clock"
#endif

enum {
  ADC_SAMPLE_TICKS = 156U,
  BROKEN_THRESHOLD = 240U,
  CLEAR_THRESHOLD = 224U,
  STABLE_SAMPLES = 3U,
  COOLDOWN_SAMPLES = 50U,
  SAMPLE_PERIOD_MS = 10U,
};

static uint16_t EEMEM persisted_boot_counter;

static lp_identity_register_t identity = {
    .protocol_major = LP_PROTOCOL_MAJOR,
    .protocol_minor = LP_PROTOCOL_MINOR,
    .firmware_major = 0U,
    .firmware_minor = 1U,
    .firmware_patch = 0U,
    .role = LP_ROLE_UNCONFIGURED,
    .address = LP_ADDRESS_COMMISSIONING,
    .capabilities = {.low = (uint8_t)LP_CAPABILITY_STATUS_LED, .high = 0U},
    .config_format_version = LP_CONFIG_FORMAT_VERSION,
    .crc8 = 0U,
};

static lp_fast_status_register_t fast_status;
static lp_diagnostics_register_t diagnostics;

static const usi_i2c_register_t register_table[] = {
    {LP_REGISTER_IDENTITY, (const uint8_t *)&identity, sizeof(identity)},
    {LP_REGISTER_FAST_STATUS, (const uint8_t *)&fast_status,
     sizeof(fast_status)},
    {LP_REGISTER_DIAGNOSTICS, (const uint8_t *)&diagnostics,
     sizeof(diagnostics)},
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
  TCCR1 = _BV(PWM1A) | _BV(COM1A1) | _BV(CS13) | _BV(CS12) | _BV(CS10);
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

static void publish_registers(uint16_t boot_counter, uint16_t event_counter,
                              uint16_t status_flags, uint16_t raw_adc,
                              uint16_t filtered_adc, uint8_t input_state,
                              uint8_t cooldown_samples) {
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
      .operating_mode = LP_MODE_SETUP,
      .last_result = LP_RESULT_OK,
      .cooldown_remaining_ms =
          lp_u16_encode((uint16_t)cooldown_samples * SAMPLE_PERIOD_MS),
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
  status_led_init();
  adc_init();
  TCCR0A = 0U;
  TCCR0B = _BV(CS01) | _BV(CS00);  // /64: 64 us per Timer0 tick.

  const uint16_t boot_counter = increment_boot_counter();
  uint16_t event_counter = 0U;
  uint16_t raw_adc = adc_read();
  uint16_t filtered_adc = raw_adc;
  uint8_t beam_broken = filtered_adc >= BROKEN_THRESHOLD;
  uint8_t pending_state = beam_broken;
  uint8_t pending_samples = 0U;
  uint8_t cooldown_samples = 0U;
  uint8_t counter_overflowed = 0U;

  identity.crc8 = lp_register_crc8(
      LP_REGISTER_IDENTITY, (const uint8_t *)&identity,
      sizeof(identity) - sizeof(identity.crc8));
  publish_registers(boot_counter, event_counter, 0U, raw_adc, filtered_adc,
                    beam_broken ? LP_INPUT_ACTIVE : LP_INPUT_INACTIVE,
                    cooldown_samples);
  usi_i2c_target_init(LP_ADDRESS_COMMISSIONING, register_table,
                      sizeof(register_table) / sizeof(register_table[0]));
  sei();

  uint8_t last_sample_tick = TCNT0;
  for (;;) {
    const uint8_t now = TCNT0;
    if ((uint8_t)(now - last_sample_tick) < ADC_SAMPLE_TICKS) {
      continue;
    }
    last_sample_tick = (uint8_t)(last_sample_tick + ADC_SAMPLE_TICKS);

    if (cooldown_samples > 0U) {
      --cooldown_samples;
    }

    raw_adc = adc_read();
    filtered_adc = (uint16_t)((filtered_adc * 15UL + raw_adc) / 16UL);

    uint8_t candidate = beam_broken;
    if (!beam_broken && filtered_adc >= BROKEN_THRESHOLD) {
      candidate = 1U;
    } else if (beam_broken && filtered_adc <= CLEAR_THRESHOLD) {
      candidate = 0U;
    }

    uint8_t input_state =
        beam_broken ? LP_INPUT_ACTIVE : LP_INPUT_INACTIVE;
    if (candidate != beam_broken) {
      if (candidate != pending_state) {
        pending_state = candidate;
        pending_samples = 1U;
      } else if (pending_samples < STABLE_SAMPLES) {
        ++pending_samples;
      }
      input_state = LP_INPUT_UNSTABLE;

      if (pending_samples >= STABLE_SAMPLES) {
        beam_broken = candidate;
        pending_samples = 0U;
        input_state =
            beam_broken ? LP_INPUT_ACTIVE : LP_INPUT_INACTIVE;
        if (beam_broken && cooldown_samples == 0U) {
          ++event_counter;
          if (event_counter == 0U) {
            counter_overflowed = 1U;
          }
          cooldown_samples = COOLDOWN_SAMPLES;
        }
      }
    } else {
      pending_state = beam_broken;
      pending_samples = 0U;
    }

    uint16_t status_flags = 0U;
    if (beam_broken) {
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

    publish_registers(boot_counter, event_counter, status_flags, raw_adc,
                      filtered_adc, input_state, cooldown_samples);
  }
}
