/*
 * Simple SPO256 driver.
 *
 * (C)opyright 2010, 2011 Peter Gammie, peteg42 at gmail dot com. All rights reserved.
 * Commenced September 2010.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

FIXME shuffle all allophones into program memory.
This causes issues with the derived arrays.

 */

#ifndef _SPO256_H_
#define _SPO256_H_

#include <stdint.h>

#include <util/delay.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#include "allophones.h"

/* **************************************** */

/*

FIXME document.

Trivial spo256 driver. See below for port connections.

So use Port C instead of PortB? Interrupts are where? Want an
interrupt for the SBY line? PCINT is a blunderbuss. What are the
options?

 */

#define SPO256_ENABLE (_BV(PB1))
#define SPO256_ALD    (_BV(PB0))
#define SPO256_RESET  (_BV(PB7))
#define SPO256_SBY    (_BV(PB6))

#define SPO256_CTRL     PORTB
#define SPO256_CTRL_IN  PINB
#define SPO256_CTRL_DDR DDRB

#define SPO256_DATA     PORTD
#define SPO256_DATA_DDR DDRD

static inline void
spo256_turn_on(void)
{
  /* Turn the SPO256 on. */
  SPO256_CTRL |= SPO256_ENABLE;
  _delay_ms(1);

  /* Reset must be held low for at least 100ns, here 1ms, overkill. */
  PORTB |= SPO256_RESET;
  _delay_ms(1);
  PORTB &= ~SPO256_RESET;
  _delay_ms(1);
  // _delay_loop_1(); FIXME 3 cycles per loop.
  PORTB |= SPO256_RESET;
}

static inline void
spo256_turn_off(void)
{
  SPO256_CTRL &= ~SPO256_ENABLE;
}

static inline void
spo256_init(void)
{
  // Port D:
  //   D2 - D7 data
  SPO256_DATA = 0x0;
  SPO256_DATA_DDR = 0xFC;

  // Port B:
  //   B0: ALD, active-low (address load)
  //   B1: power switch
  //   B6: SBY, active high (standby, chip idle, input)
  //   B7: RESET, active low.
  SPO256_CTRL = 0x0;
  SPO256_CTRL_DDR &= SPO256_SBY;
  SPO256_CTRL_DDR |= SPO256_ALD | SPO256_RESET | SPO256_ENABLE;
}

static inline void
speak_allophone(allophone_t allophone)
{
  /* Load allophone, holding ALD high. */
  /* FIXME adhoc shift here, abstract that too. */
  SPO256_DATA = (allophone & 0x3F) << 2;
  SPO256_CTRL |= SPO256_ALD;

  // Take ALD low for at least 1.1us, overkill.
  // _delay_ms(1);
  SPO256_CTRL &= ~SPO256_ALD;
  _delay_ms(1);
  // _delay_loop_1(); FIXME 3 cycles per loop.
  SPO256_CTRL |= SPO256_ALD;

  // Wait for the chip to finish.
  // FIXME an interrupt might be the go here.
  while(! (SPO256_CTRL_IN & SPO256_SBY)) {
  }
}

/* All allophone lists live in program memory. */
static inline
void speak_P(const prog_allophone_t allophones[])
{
  unsigned int i = 0;
  uint8_t a = pgm_read_byte(&allophones[i]);

  while(a != aEND) {
    speak_allophone(a);
    i++;
    a = pgm_read_byte(&allophones[i]);
  }

  // FIXME 50ms pause on the end.
  speak_allophone(0x02);
}

/* **************************************** */
/* Letters. */

static const
prog_allophone_t char_G[] = { 0x0A, aIY, aEND };

/* **************************************** */
/* Words and phrases. */

static const
prog_allophone_t talking_computer[] = {
  aTT2, aAO, aAO, 0x02, 0x2A, aIH, 0x2C, 0x04,
  0x04, 0x2A, 0x0F, 0x10, 0x00, 0x31, 0x16, aTT2,
  0x33, 0x04, aEND
};

static const
prog_allophone_t talking_clock[] = {
  aTT2, aAO, aAO, 0x02, 0x2A, aIH, 0x2C, 0x04,
  0x04, 0x2A, 0x2D, aAA, aAA, 0x02, 0x29, aEND
};

static const
prog_allophone_t and[] = { aAE, aNN1, aDD1, aEND };

static const
prog_allophone_t clown[] = { 0x2A, 0x2D, 0x20, aNN1, aEND };

static const
prog_allophone_t degrees[] = { 0x21, aIY, 0x24, 0x0E, aIY, 0x2B, aEND }; // DD2 IY GG1 RR1 IY ZZ

static const
prog_allophone_t hours[] = { 0x20, 0x33, 0x2B, aEND }; // AW ER1

static const
prog_allophone_t is[] = { aIH, 0x2B, aEND };

static const
prog_allophone_t it[] = { aIH, 0x03, aTT1, aEND };

static const
prog_allophone_t minutes[] = { aMM, aIH, aNN1, aIH, 0x02, aTT2, 0x2B, aEND };

static const
prog_allophone_t no[] = { 0x38, 0x35, aEND };

static const
prog_allophone_t point[] = { 0x09, 0x05, aNN1, aTT1, aEND };

static const
prog_allophone_t seconds[] = { 0x37, 0x37, 0x07, 0x02, 0x2A, aIH, aNN1, 0x01, aDD1, 0x2B, aEND }; // SS SS EH PA3 KK1 IH NN1 PA2 DD1 ZZ

static const
prog_allophone_t sensors[] = { 0x37, 0x37, 0x07, 0x07, aNN1, 0x37, 0x37, 0x33, 0x2B, aEND }; // SS SS EH EH NN1 SS SS ER1 ZZ

static const
prog_allophone_t the[] = { 0x12, aIY, aEND };

static const
prog_allophone_t time[] = { aTT2, 0x06, 0x10, aEND };

/* **************************************** */
/* Numbers */

static const
prog_allophone_t minus[] = { aMM, aAY, aNN1, aIH, 0x02, 0x37, aEND };

static const
prog_allophone_t zero[] = { 0x2B, 0x3C, 0x35, aEND }; // ZZ YR OW

static const
prog_allophone_t one[] = { 0x30, 0x0F, aNN1, aEND }; // WW SX AX NN1

static const
prog_allophone_t two[] = { aTT2, 0x1F, aEND };

static const
prog_allophone_t three[] = { 0x36, 0x27, aIY, aEND };

static const
prog_allophone_t four[] = { 0x28, aAO, aAO, 0x27, aEND };

static const
prog_allophone_t five[] = { 0x28, 0x06, 0x23, aEND };

static const
prog_allophone_t six[] = { 0x37, aIH, 0x29, 0x37, aEND };

static const
prog_allophone_t seven[] = { 0x37, 0x37, 0x07, 0x07, 0x23, aIH, aNN1, aEND };

static const
prog_allophone_t eight[] = { aEY, aTT1, aEND };

static const
prog_allophone_t nine[] = { 0x38, 0x06, aNN1, aEND };

static const
prog_allophone_t ten[] = { aTT2, 0x07, 0x07, aNN1, aEND };

static const
prog_allophone_t eleven[] = { aIY, 0x2D, 0x07, 0x23, 0x34, aNN1, aEND };

static const
prog_allophone_t twelve[] = { aTT2, 0x2E, 0x07, 0x3E, 0x01, 0x23, aEND };

static const
prog_allophone_t thirteen[] = { 0x1D, 0x33, aTT2, aIY, aNN1, aEND };

static const
prog_allophone_t fourteen[] = { 0x28, aAO, 0x27, aTT2, aIY, aNN1, aEND };

static const
prog_allophone_t fifteen[] = { 0x28, aIH, 0x28, aTT2, aIY, aNN1, aEND };

static const
prog_allophone_t sixteen[] = { 0x37, aIH, 0x29, 0x37, aTT2, aIY, aNN1, aEND };

static const
prog_allophone_t seventeen[] = { 0x37, 0x37, 0x07, 0x07, 0x23, aIH, aNN1, aTT2, aIY, aNN1, aEND };

static const
prog_allophone_t eighteen[] = { aEY, aTT1, aTT2, aIY, aNN1, aEND };

static const
prog_allophone_t nineteen[] = { 0x38, 0x06, aNN1, aTT2, aIY, aNN1, aEND };

static const
prog_allophone_t twenty[] = { aTT2, 0x2E, 0x07, aNN1, aTT2, aIY, aEND };

static const
prog_allophone_t thirty[] = { 0x1D, 0x33, aTT2, aIY, aEND };

static const
prog_allophone_t forty[] = { 0x28, aAO, 0x27, aTT2, aIY, aEND };

static const
prog_allophone_t fifty[] = { 0x28, aIH, 0x28, aTT2, aIY, aEND };

static const
prog_allophone_t sixty[] = { 0x37, aIH, 0x29, 0x37, aTT2, aIY, aEND };

static const
prog_allophone_t seventy[] = { 0x37, 0x37, 0x07, 0x07, 0x23, aIH, aNN1, aTT2, aIY, aEND };

static const
prog_allophone_t eighty[] = { aEY, aTT1, aTT2, aIY, aEND };

static const
prog_allophone_t ninety[] = { 0x38, 0x06, aNN1, aTT2, aIY, aEND };

static const
prog_allophone_t hundred[] = { 0x39, 0x0F, 0x0F, aNN1, 0x01, 0x21, 0x27, aIH, aIH, 0x00, aDD1, aEND };

static const
prog_allophone_t thousand[] = { 0x1D, 0x20, 0x2B, aAE, aNN1, aDD1, aEND };

static const
prog_allophone_t *numbers[] PROGMEM = {
  zero, one, two, three, four, five,
  six, seven, eight, nine, ten,
  eleven, twelve, thirteen, fourteen, fifteen,
  sixteen, seventeen, eighteen, nineteen
};

static const
prog_allophone_t *decades[] PROGMEM = {
  twenty, thirty, forty, fifty, sixty, seventy, eighty, ninety
};

/* FIXME need to pepper this with ptr reads (arrays only?). */
static
void speak_number(int16_t n)
{
  /* Tail recursive, but don't expect GCC to realise that. */
  while(1) {
    if(n < 0) {
      speak_P(minus);
      n = -n;
    } else if(n >= 1000) {
      uint8_t t = n / 1000;
      n = n % 1000;

      speak_number(t);
      speak_P(thousand);
      if(n == 0) {
        break;
      } else if(n < 100) {
        speak_P(and);
      }
    } else if(n >= 100) {
      uint8_t h = n / 100;
      n = n % 100;

      speak_number(h);
      speak_P(hundred);
      if(n == 0) {
        break;
      } else {
        speak_P(and);
      }
    } else if(n >= 20) {
      uint8_t d = n / 10;
      n = n % 10;

      speak_P((prog_allophone_t *)pgm_read_word(&(decades[d - 2])));
      if(n == 0) {
        break;
      }
    } else {
      speak_P((prog_allophone_t *)pgm_read_word(&numbers[n]));
      break;
    }
  }
}

#endif /* _SPO256_H_ */
