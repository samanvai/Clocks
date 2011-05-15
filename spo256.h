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

#ifndef _SPO256_H_
#define _SPO256_H_

#include <stdint.h>

#include <util/delay.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#include "allophones.h"

/* **************************************** */
/* Port connections. */

#define SPO256_ENABLE (_BV(PB1))
#define SPO256_ALD    (_BV(PB0))
#define SPO256_RESET  (_BV(PB7))
#define SPO256_SBY    (_BV(PB6))

#define SPO256_CTRL     PORTB
#define SPO256_CTRL_IN  PINB
#define SPO256_CTRL_DDR DDRB

#define SPO256_DATA     PORTD
#define SPO256_DATA_DDR DDRD

/* **************************************** */

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

void speak_allophone(allophone_t allophone);

/* All allophone arrays live in program memory. */
void speak_P(const prog_allophone_t allophones[]);

void speak_number(int16_t n);

/* **************************************** */
/* Allophone arrays. */

extern const prog_allophone_t talking_clock[];

extern const prog_allophone_t *decades[];
extern const prog_allophone_t *numbers[];

extern const prog_allophone_t and[];
extern const prog_allophone_t clown[];
extern const prog_allophone_t hours[];
extern const prog_allophone_t hundred[];
extern const prog_allophone_t hundreds[];
extern const prog_allophone_t is[];
extern const prog_allophone_t minus[];
extern const prog_allophone_t minutes[];
extern const prog_allophone_t seconds[];
extern const prog_allophone_t sensors[];
extern const prog_allophone_t the[];
extern const prog_allophone_t thousand[];
extern const prog_allophone_t time[];

#endif /* _SPO256_H_ */
