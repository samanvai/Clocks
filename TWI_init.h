/*
 * TWI driver for an ATMEGA328 (really any AVR with TWI hardware).
 *
 * Initialisation routines.
 *
 * Synchronous, no interrupts in sight. Some ideas from Peter Fleury's
 * code:
 *
 *    http://code.google.com/p/freecockpit/source/browse/software_avr/hwmaster_mega8/twimaster.c
 *
 * (C)opyright 2010 Peter Gammie, peteg42 at gmail dot com. All rights reserved.
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

#ifndef _TWI_INIT_H_
#define _TWI_INIT_H_

#include <avr/io.h>
#include <util/twi.h>

#ifndef F_CPU
#error "Please define the cpu frequency F_CPU"
#endif

/* TWI clock in Hz. 400kHz is the limit. With a 1MHz clock try ~ 10kHz. */
#ifndef SCL_CLOCK
#error "Please define the TWI clock frequency SCL_CLOCK"
#endif

static inline void
TWI_init(void)
{
  /* Fire up the TWI module. */
  power_twi_enable();

  /* TWI timing: prescaler: 1. */
  TWSR = 0;

  // FIXME for a 1MHz AVR clock / 400kHz TWI clock:
  //    TWI.h:157: warning: large integer implicitly truncated to unsigned type
  // FIXME SCL frequency = 1159200 / (16 + 2 * 47 * 1) = 98.743 khz
#if (F_CPU / SCL_CLOCK - 16) / 2 <= 10
#error (F_CPU / SCL_CLOCK - 16) / 2 should be > 10 for stable operation
#endif
  TWBR = (F_CPU / SCL_CLOCK - 16) / 2;

  /* Don't bother setting the TWAR - slave address register. */
}

/* FIXME useful? */
static inline void
TWI_turn_off(void)
{
  power_twi_disable();
}

#endif /* _TWI_INIT_H_ */
