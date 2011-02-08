/*
 * DS1307 RTC driver for an ATMEGA328 (really any AVR with TWI hardware).
 *
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
 */

#ifndef _ds1307_H_
#define _ds1307_H_

#include <stdbool.h>

// FIXME 10kHz I2C clock.
// #define SCL_CLOCK  400000L
#define SCL_CLOCK  10000L

#include "TWI.h"

// FIXME
#include "led.h"

/* **************************************** */

/* DS1307-specifics: twi address 0b1101000. Note: shifted left 1. */
#define DS1307_ADDR  0xD0

/* reg0: Turns the clock oscillator on/off. */
#define CLOCK_HALT   _BV(7)

/* reg2: if TWELVE_HOUR
 *         then 12hr mode with AMPM indictor,
 *         otherwise 24hr mode. */
#define AMPM         _BV(5)
#define TWELVE_HOUR  _BV(6)

/* Unsigned 8-bit BCD operations. */
#define fromBCD(x) (((x) >> 4) * 10 + ((x) & 0xF))
#define toBCD(x)   ((((x) / 10) << 4) | ((x) % 10))

struct ds1307_time_t {
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
};

/* **************************************** */

static inline bool
ds1307_read(struct ds1307_time_t *time_data)
{
  uint8_t twsr;
  uint8_t data;

  /* Tell the DS1307 we want to read starting at address 0. */
  if(!TWI_start(DS1307_ADDR, &twsr, WRITE)) goto error;
  if(!TWI_write(&twsr, 0x0)) goto error;
  /* Commence the read. */
  if(!TWI_rep_start(DS1307_ADDR, &twsr, READ)) goto error;

  /* Read the seconds register. */
  if(!TWI_read(&twsr, &data, true)) goto error;
  time_data->seconds = fromBCD(data & ~CLOCK_HALT);

  /* Read the minutes register. */
  if(!TWI_read(&twsr, &data, true)) goto error;
  time_data->minutes = fromBCD(data);

  /* Read the hours register. */
  /* FIXME last read, send NACK. */
  if(!TWI_read(&twsr, &data, false)) goto error;
  if(data & TWELVE_HOUR) {
    uint8_t hours = fromBCD(data & 0x1F);
    if(data & AMPM) {
      hours += 12;
    }
    time_data->hours = hours;
  } else {
    time_data->hours = fromBCD(data);
  }

  TWI_send_stop(&twsr);
  return true;

 error:

  TWI_send_stop(&twsr);
  return false;
}

/* FIXME Assumes the time is sane and is a bit hardwired. */
static inline bool
ds1307_write(struct ds1307_time_t *time_data)
{
  uint8_t twsr;

  /* Tell the DS1307 we want to start writing at address 0. */
  if(!TWI_start(DS1307_ADDR, &twsr, WRITE)) goto error;
  if(!TWI_write(&twsr, 0x0)) goto error;
  /* Keep the oscillator running. */
  if(!TWI_write(&twsr, toBCD(time_data->seconds) & ~CLOCK_HALT)) goto error;
  if(!TWI_write(&twsr, toBCD(time_data->minutes))) goto error;
  /* FIXME 24hr time. */
  if(!TWI_write(&twsr, toBCD(time_data->hours))) goto error;

  TWI_send_stop(&twsr);
  return true;

 error:

  TWI_send_stop(&twsr);
  return false;
}

/* Initialise the TWI interface. */
static inline bool
ds1307_init(void)
{
  TWI_init();

  /* Tell the DS1307 to start the oscillator (turn off CLOCK HALT) in
   * case it has lost power. */
  uint8_t twsr;
  uint8_t reg0;

  /* Address, say read from memory slot 0, then read data. */
  if(!TWI_start(DS1307_ADDR, &twsr, WRITE)) goto error;
  if(!TWI_write(&twsr, 0x0)) goto error;
  if(!TWI_rep_start(DS1307_ADDR, &twsr, READ)) goto error;
  if(!TWI_read(&twsr, &reg0, true)) goto error;

  if(reg0 & CLOCK_HALT) {
    if(!TWI_rep_start(DS1307_ADDR, &twsr, WRITE)) goto error;
    if(!TWI_write(&twsr, 0x0)) goto error;
    if(!TWI_write(&twsr, reg0 & ~CLOCK_HALT)) goto error;
  }

  // TWI_send_stop(&twsr);

  return true;

 error:

  TWI_send_stop(&twsr);
  return false;
}

#endif /* _ds1307_H_ */
