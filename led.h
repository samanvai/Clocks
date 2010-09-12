/*
 * Trivial LED driver for an ATMEGA328.
 *
 * (C)opyright Peter Gammie, peteg42 at gmail dot com
 * Commenced September 2010.
 *
 */

#ifndef _LED_H_
#define _LED_H_

#define LED (_BV(PB6))

static inline void
led_init(void)
{
  DDRB |= LED;
}

static inline void
led_off(void)
{
  PORTB &= ~LED;
}

static inline void
led_on(void)
{
  PORTB |= LED;
}

// FIXME halt and flash the led.
static inline void
led_flash(unsigned int d)
{
  led_init();

  while(1) {
    led_on();
    _delay_ms(1000);

    led_off();
    _delay_ms(d * 100);
  }
}

#endif /* _LED_H_ */
