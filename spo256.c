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
 */

#include <stdint.h>

// FIXME cli/sei experiment
#include <avr/interrupt.h>

#include <util/delay.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#include "spo256.h"

void
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

void
speak_P(const prog_allophone_t allophones[])
{
  unsigned int i = 0;
  uint8_t a = pgm_read_byte(&allophones[i]);

  // FIXME do interrupts cause us to glitch?
  cli();

  while(a != aEND) {
    speak_allophone(a);
    i++;
    a = pgm_read_byte(&allophones[i]);
  }

  // FIXME 50ms pause on the end.
  speak_allophone(aPA3);

  sei();
}

void
speak_number(int16_t n)
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

/* FIXME should auto-generate all this + a header from a text file. */

/* **************************************** */
/* Letters. */

static const
prog_allophone_t char_G[] = { 0x0A, aIY, aEND };

/* **************************************** */
/* Words and phrases. */

const prog_allophone_t
talking_computer[] = {
  aTT2, aAO, aAO, aPA3, 0x2A, aIH, 0x2C, 0x04,
  0x04, 0x2A, aAX, 0x10, 0x00, 0x31, 0x16, aTT2,
  0x33, 0x04, aEND
};

const prog_allophone_t
talking_clock[] = {
  aTT2, aAO, aAO, aPA3, 0x2A, aIH, 0x2C, 0x04,
  0x04, 0x2A, 0x2D, aAA, aAA, aPA3, 0x29, aEND
};

const prog_allophone_t
and[] = { aAE, aNN1, aDD1, aEND };

const prog_allophone_t
clown[] = { 0x2A, 0x2D, aAW, aNN1, aEND };

const prog_allophone_t
degrees[] = { 0x21, aIY, 0x24, 0x0E, aIY, aZZ, aEND }; // DD2 IY GG1 RR1 IY ZZ

const prog_allophone_t
hours[] = { aAW, 0x33, aZZ, aEND }; // AW ER1

const prog_allophone_t
is[] = { aIH, aZZ, aEND };

const prog_allophone_t
it[] = { aIH, 0x03, aTT1, aEND };

const prog_allophone_t
minutes[] = { aMM, aIH, aNN1, aIH, aPA3, aTT2, aZZ, aEND };

const prog_allophone_t
no[] = { 0x38, aOW, aEND };

const prog_allophone_t
point[] = { 0x09, 0x05, aNN1, aTT1, aEND };

const prog_allophone_t
seconds[] = { aSS, aSS, 0x07, aPA3, 0x2A, aIH, aNN1, 0x01, aDD1, aZZ, aEND }; // SS SS EH PA3 KK1 IH NN1 PA2 DD1 ZZ

const prog_allophone_t
sensors[] = { aSS, aSS, 0x07, 0x07, aNN1, aSS, aSS, 0x33, aZZ, aEND }; // SS SS EH EH NN1 SS SS ER1 ZZ

const prog_allophone_t
the[] = { 0x12, aIY, aEND };

const prog_allophone_t
time[] = { aTT2, 0x06, 0x10, aEND };

/* **************************************** */
/* FIXME directions, rough */

const prog_allophone_t
left[] = { aLL, aEH, aFF, aTT1, aEND };

const prog_allophone_t
right[] = { aRR1, aIH, aTT1, aEND };

const prog_allophone_t
up[] = { aAX, aPP, aEND };

const prog_allophone_t
down[] = { aDD2, aAW, aNN1, aEND };

/* **************************************** */
/* Numbers */

const prog_allophone_t
minus[] = { aMM, aAY, aNN1, aIH, aSS, aEND };

static const
prog_allophone_t zero[] = { aZZ, aYR, aOW, aEND };

static const
prog_allophone_t one[] = { aWH, aAX, aNN1, aEND };

static const
prog_allophone_t two[] = { aTT2, 0x1F, aEND };

static const
prog_allophone_t three[] = { 0x36, 0x27, aIY, aEND };

static const
prog_allophone_t four[] = { 0x28, aAO, aAO, 0x27, aEND };

static const
prog_allophone_t five[] = { 0x28, 0x06, 0x23, aEND };

static const
prog_allophone_t six[] = { aSS, aIH, 0x29, aSS, aEND };

static const
prog_allophone_t seven[] = { aSS, aSS, 0x07, 0x07, 0x23, aIH, aNN1, aEND };

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
prog_allophone_t thirteen[] = { aTH, 0x33, aTT2, aIY, aNN1, aEND };

static const
prog_allophone_t fourteen[] = { 0x28, aAO, 0x27, aTT2, aIY, aNN1, aEND };

static const
prog_allophone_t fifteen[] = { 0x28, aIH, 0x28, aTT2, aIY, aNN1, aEND };

static const
prog_allophone_t sixteen[] = { aSS, aIH, 0x29, aSS, aTT2, aIY, aNN1, aEND };

static const
prog_allophone_t seventeen[] = { aSS, aSS, 0x07, 0x07, 0x23, aIH, aNN1, aTT2, aIY, aNN1, aEND };

static const
prog_allophone_t eighteen[] = { aEY, aTT1, aTT2, aIY, aNN1, aEND };

static const
prog_allophone_t nineteen[] = { 0x38, 0x06, aNN1, aTT2, aIY, aNN1, aEND };

static const
prog_allophone_t twenty[] = { aTT2, 0x2E, 0x07, aNN1, aTT2, aIY, aEND };

static const
prog_allophone_t thirty[] = { aTH, 0x33, aTT2, aIY, aEND };

static const
prog_allophone_t forty[] = { 0x28, aAO, 0x27, aTT2, aIY, aEND };

static const
prog_allophone_t fifty[] = { 0x28, aIH, 0x28, aTT2, aIY, aEND };

static const
prog_allophone_t sixty[] = { aSS, aIH, 0x29, aSS, aTT2, aIY, aEND };

static const
prog_allophone_t seventy[] = { aSS, aSS, 0x07, 0x07, 0x23, aIH, aNN1, aTT2, aIY, aEND };

static const
prog_allophone_t eighty[] = { aEY, aTT1, aTT2, aIY, aEND };

static const
prog_allophone_t ninety[] = { 0x38, 0x06, aNN1, aTT2, aIY, aEND };

const prog_allophone_t
hundred[] = { aHH2, aAX, aAX, aNN1, 0x01, 0x21, 0x27, aIH, aIH, 0x00, aDD1, aEND };

const prog_allophone_t
thousand[] = { aTH, aAW, aZZ, aAE, aNN1, aDD1, aEND };

const prog_allophone_t *
numbers[] PROGMEM = {
  zero, one, two, three, four, five,
  six, seven, eight, nine, ten,
  eleven, twelve, thirteen, fourteen, fifteen,
  sixteen, seventeen, eighteen, nineteen
};

const prog_allophone_t *
decades[] PROGMEM = {
  twenty, thirty, forty, fifty, sixty, seventy, eighty, ninety
};
