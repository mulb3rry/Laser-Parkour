// Combined node hardware smoke test.
#include <avr/io.h>

#ifndef F_CPU
#error "F_CPU must be defined by PlatformIO"
#endif

#if F_CPU != 1000000UL
#error "The node smoke test expects a 1 MHz ATtiny85 system clock"
#endif

enum {
  LDR_BROKEN_THRESHOLD = 240,
  LDR_CLEAR_THRESHOLD = 224,
};

static void led_timer_init(void) {
  // PB1 (physical pin 6) is the Timer/Counter1 OC1A output.
  DDRB |= _BV(PB1);

  TCCR1 = 0;
  GTCCR = _BV(PSR1);
  TCNT1 = 0;

  // 1 MHz / 4096 / (60 + 1) is approximately 4 Hz, with 50% duty cycle.
  OCR1C = 60;
  OCR1A = 30;

  GTCCR = 0;
  TCCR1 = _BV(PWM1A) | _BV(CS13) | _BV(CS12) | _BV(CS10);
}

static void led_set_clear(void) {
  // Disconnect OC1A from PB1 and leave the LED steadily on.
  TCCR1 &= (uint8_t)~(_BV(COM1A1) | _BV(COM1A0));
  PORTB |= _BV(PB1);
}

static void led_set_broken(void) {
  // Timer1 drives the LED; no interrupt or software blinking is involved.
  TCCR1 = (TCCR1 & (uint8_t)~_BV(COM1A0)) | _BV(COM1A1);
}

static void adc_init(void) {
  // ADC3 is PB3 (physical pin 2). Use VCC as the voltage reference and
  // disable its digital input buffer. /8 gives a 125 kHz ADC clock.
  ADMUX = _BV(MUX1) | _BV(MUX0);
  DIDR0 = _BV(ADC3D);
  ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0);
}

static uint16_t adc_read(void) {
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC)) {
  }
  return ADC;
}

int main(void) {
  led_timer_init();
  adc_init();

  uint16_t filtered = adc_read();
  uint8_t beam_broken = filtered >= LDR_BROKEN_THRESHOLD;

  if (beam_broken) {
    led_set_broken();
  } else {
    led_set_clear();
  }

  for (;;) {
    // 1/16 new sample and 15/16 previous value, matching the specification.
    filtered = (uint16_t)((filtered * 15UL + adc_read()) / 16UL);

    if (!beam_broken && filtered >= LDR_BROKEN_THRESHOLD) {
      beam_broken = 1;
      led_set_broken();
    } else if (beam_broken && filtered <= LDR_CLEAR_THRESHOLD) {
      beam_broken = 0;
      led_set_clear();
    }
  }
}
