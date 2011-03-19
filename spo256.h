/*
 * Simple SPO256 driver.
 *
 * (C)opyright Peter Gammie, peteg42 at gmail dot com
 * Commenced September 2010.
 * Synchronous, no interrupts in sight.
 *
 * (C)opyright 2010 Peter Gammie, peteg42 at gmail dot com. All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY <COPYRIGHT HOLDER> ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Commenced September 2010.
 *
 * Allophone arrays use aEND as a terminator.
 */

#ifndef _SPO256_H_
#define _SPO256_H_

#include <stdint.h>

#include <util/delay.h>
#include <avr/io.h>

#include "allophones.h"

/* **************************************** */

/*

FIXME document.

Trivial spo256 driver. See below for port connections.

So use Port C instead of PortB? Interrupts are where? Want an
interrupt for the SBY line? PCINT is a blunderbuss. What are the
options?

 */

#define ALD (_BV(PB0))

#define RESET (_BV(PB7))
#define SBY (_BV(PB6))

static inline void
spo256_init(void)
{
  // Port D:
  //   D2 - D7 data
  PORTD = 0x0;
  DDRD = 0xFC;

  // Port B:
  //   B0: ALD, active-low (address load)
  //   B6: SBY, active high (standby, chip idle, input)
  //   B7: RESET, active low.
  PORTB = 0x0;
  DDRB &= SBY;
  DDRB |= ALD;
  DDRB |= RESET;

  // Reset must be held low for at least 100ns, here 1ms, overkill.
  PORTB |= RESET;
  _delay_ms(1);
  PORTB &= ~RESET;
  _delay_ms(1);
  // _delay_loop_1(); FIXME 3 cycles per loop.
  PORTB |= RESET;
}

static inline void
load_allophone(uint8_t allophone)
{
  // Load allophone, holding ALD high.
  PORTD = (allophone & 0x3F) << 2;
  PORTB |= ALD;

  // Take ALD low for at least 1.1us, overkill.
  // _delay_ms(1);
  PORTB &= ~ALD;
  _delay_ms(1);
  // _delay_loop_1(); FIXME 3 cycles per loop.
  PORTB |= ALD;

  // Wait for the chip to finish.
  // FIXME an interrupt might be the go here.
  while(! (PINB & SBY)) {
  }
}

static inline
void speak(const uint8_t allophones[])
{
  unsigned int i = 0;
  while(allophones[i] != aEND) {
    load_allophone(allophones[i]);
    i++;
  }

  // FIXME 50ms pause on the end.
  load_allophone(0x02);
}

/* **************************************** */
/* Letters. */

static const
uint8_t char_G[] = { 0x0A, 0x13, aEND };

/* **************************************** */
/* Words and phrases. */

static const
uint8_t talking_computer[] = {
  aTT2, 0x17, 0x17, 0x02, 0x2A, aIH, 0x2C, 0x04,
  0x04, 0x2A, 0x0F, 0x10, 0x00, 0x31, 0x16, aTT2,
  0x33, 0x04, aEND
};

static const
uint8_t talking_clock[] = {
  aTT2, 0x17, 0x17, 0x02, 0x2A, aIH, 0x2C, 0x04,
  0x04, 0x2A, 0x2D, 0x18, 0x18, 0x02, 0x29, aEND
};

static const
uint8_t and[] = { 0x1A, 0x0B, 0x15, aEND };

static const
uint8_t clown[] = { 0x2A, 0x2D, 0x20, 0x0B, aEND };

static const
uint8_t degrees[] = { 0x21, 0x13, 0x24, 0x0E, 0x13, 0x2B, aEND }; // DD2 IY GG1 RR1 IY ZZ

static const
uint8_t hours[] = { 0x20, 0x33, 0x2B, aEND }; // AW ER1

static const
uint8_t is[] = { aIH, 0x2B, aEND };

static const
uint8_t it[] = { aIH, 0x03, aTT1, aEND };

static const
uint8_t minutes[] = { aMM, aIH, aNN1, aIH, 0x02, aTT2, 0x2B, aEND };

static const
uint8_t no[] = { 0x38, 0x35, aEND };

static const
uint8_t point[] = { 0x09, 0x05, 0x0B, aTT1, aEND }; // FIXME

static const
uint8_t seconds[] = { 0x37, 0x37, 0x07, 0x02, 0x2A, aIH, 0x0B, 0x01, 0x15, 0x2B, aEND }; // SS SS EH PA3 KK1 IH NN1 PA2 DD1 ZZ

static const
uint8_t sensors[] = { 0x37, 0x37, 0x07, 0x07, 0x0B, 0x37, 0x37, 0x33, 0x2B, aEND }; // SS SS EH EH NN1 SS SS ER1 ZZ

static const
uint8_t the[] = { 0x12, 0x13, aEND };

static const
uint8_t time[] = { aTT2, 0x06, 0x10, aEND };

/* **************************************** */
/* Numbers */

static const
uint8_t minus[] = { aMM, aAY, 0x0B, aIH, 0x02, 0x37, aEND };

static const
uint8_t zero[] = { 0x2B, 0x3C, 0x35, aEND }; // ZZ YR OW

static const
uint8_t one[] = { 0x30, 0x0F, 0x0B, aEND }; // WW SX AX NN1

static const
uint8_t two[] = { aTT2, 0x1F, aEND };

static const
uint8_t three[] = { 0x36, 0x27, 0x13, aEND };

static const
uint8_t four[] = { 0x28, 0x17, 0x17, 0x27, aEND };

static const
uint8_t five[] = { 0x28, 0x06, 0x23, aEND };

static const
uint8_t six[] = { 0x37, aIH, 0x29, 0x37, aEND };

static const
uint8_t seven[] = { 0x37, 0x37, 0x07, 0x07, 0x23, aIH, 0x0B, aEND };

static const
uint8_t eight[] = { aEY, aTT1, aEND };

static const
uint8_t nine[] = { 0x38, 0x06, 0x0B, aEND };

static const
uint8_t ten[] = { aTT2, 0x07, 0x07, 0x0B, aEND };

static const
uint8_t eleven[] = { 0x13, 0x2D, 0x07, 0x23, 0x34, 0x0B, aEND };

static const
uint8_t twelve[] = { aTT2, 0x2E, 0x07, 0x3E, 0x01, 0x23, aEND };

static const
uint8_t thirteen[] = { 0x1D, 0x33, aTT2, 0x13, 0x0B, aEND };

static const
uint8_t fourteen[] = { 0x28, 0x17, 0x27, aTT2, 0x13, 0x0B, aEND };

static const
uint8_t fifteen[] = { 0x28, aIH, 0x28, aTT2, 0x13, 0x0B, aEND };

static const
uint8_t sixteen[] = { 0x37, aIH, 0x29, 0x37, aTT2, 0x13, 0x0B, aEND };

static const
uint8_t seventeen[] = { 0x37, 0x37, 0x07, 0x07, 0x23, aIH, 0x0B, aTT2, 0x13, 0x0B, aEND };

static const
uint8_t eighteen[] = { aEY, aTT1, aTT2, 0x13, 0x0B, aEND };

static const
uint8_t nineteen[] = { 0x38, 0x06, 0x0B, aTT2, 0x13, 0x0B, aEND };

static const
uint8_t twenty[] = { aTT2, 0x2E, 0x07, 0x0B, aTT2, 0x13, aEND };

static const
uint8_t thirty[] = { 0x1D, 0x33, aTT2, 0x13, aEND };

static const
uint8_t forty[] = { 0x28, 0x17, 0x27, aTT2, 0x13, aEND };

static const
uint8_t fifty[] = { 0x28, aIH, 0x28, aTT2, 0x13, aEND };

static const
uint8_t sixty[] = { 0x37, aIH, 0x29, 0x37, aTT2, 0x13, aEND };

static const
uint8_t seventy[] = { 0x37, 0x37, 0x07, 0x07, 0x23, aIH, 0x0B, aTT2, 0x13, aEND };

static const
uint8_t eighty[] = { aEY, aTT1, aTT2, 0x13, aEND };

static const
uint8_t ninety[] = { 0x38, 0x06, 0x0B, aTT2, 0x13, aEND };

static const
uint8_t hundred[] = { 0x39, 0x0F, 0x0F, 0x0B, 0x01, 0x21, 0x27, aIH, aIH, 0x00, 0x15, aEND };

static const
uint8_t thousand[] = { 0x1D, 0x20, 0x2B, 0x1A, 0x0B, 0x15, aEND };

static const
uint8_t *numbers[] = {
  zero, one, two, three, four, five,
  six, seven, eight, nine, ten,
  eleven, twelve, thirteen, fourteen, fifteen,
  sixteen, seventeen, eighteen, nineteen
};

static const
uint8_t *decades[] = {
  twenty, thirty, forty, fifty, sixty, seventy, eighty, ninety
};

static
void speak_number(int16_t n)
{
  /* Tail recursive, but don't expect GCC to realise that. */
  while(1) {
    if(n < 0) {
      speak(minus);
      n = -n;
    } else if(n >= 1000) {
      uint8_t t = n / 1000;
      n = n % 1000;

      speak_number(t);
      speak(thousand);
      if(n == 0) {
        break;
      } else if(n < 100) {
        speak(and);
      }
    } else if(n >= 100) {
      uint8_t h = n / 100;
      n = n % 100;

      speak_number(h);
      speak(hundred);
      if(n == 0) {
        break;
      } else {
        speak(and);
      }
    } else if(n >= 20) {
      uint8_t d = n / 10;
      n = n % 10;

      speak(decades[d - 2]);
      if(n == 0) {
        break;
      }
    } else {
      speak(numbers[n]);
      break;
    }
  }
}

#endif /* _SPO256_H_ */
