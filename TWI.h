/*
 * TWI driver for an ATMEGA328 (really any AVR with TWI hardware).
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
 */

#ifndef _TWI_H_
#define _TWI_H_

#include <avr/io.h>
#include <util/twi.h>

#ifndef F_CPU
#error "Please define the cpu frequency F_CPU"
#endif

/* TWI clock in Hz. 400kHz is the limit. With a 1MHz clock try ~ 10kHz. */
#ifndef SCL_CLOCK
#error "Please define the TWI clock frequency SCL_CLOCK"
#endif

typedef enum { READ, WRITE } rw_t;

/* **************************************** */

static inline void
TWI_wait_int(uint8_t *twsr) {
  while(!(TWCR & _BV(TWINT)))
    ;
  *twsr = TW_STATUS & TW_STATUS_MASK;
}

/* Send a START condition to the bus and wait for TWINT to be set. */
static inline bool
TWI_send_start(uint8_t *twsr)
{
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
  TWI_wait_int(twsr);

  /* START or REPEATED_START are both OK. */
  return *twsr == TW_START || *twsr == TW_REP_START;
}

static inline void
TWI_send_stop(uint8_t *twsr)
{
  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
  while(TWCR & _BV(TWSTO))
    ;
}

/* **************************************** */

/* Read data from the previously addressed device. */
static inline bool
TWI_read(uint8_t *twsr, uint8_t *data, bool ack)
{
  TWCR = _BV(TWINT) | _BV(TWEN) | (ack ? _BV(TWEA) : 0);
  TWI_wait_int(twsr);
  *data = TWDR;

  return *twsr == (ack ? TW_MR_DATA_ACK : TW_MR_DATA_NACK);
}

/* Send data to the previously addressed device. */
static inline bool
TWI_write(uint8_t *twsr, uint8_t data)
{
  TWDR = data;
  TWCR = _BV(TWINT) | _BV(TWEN);
  TWI_wait_int(twsr);

  return *twsr == TW_MT_DATA_ACK;
}

static inline bool
TWI_start(uint8_t slave_addr, uint8_t *twsr, rw_t rw)
{
  if(TWI_send_start(twsr)) {
    /* Send slave (chip) address. */
    TWDR = slave_addr | (rw == READ ? TW_READ : TW_WRITE);
    TWCR = _BV(TWINT) | _BV(TWEN);
    TWI_wait_int(twsr);

    return *twsr == TW_MR_SLA_ACK || *twsr == TW_MT_SLA_ACK;
  }

  return false;
}

static inline bool
TWI_start_wait(uint8_t slave_addr, rw_t rw)
{
  while(1) {
  //for(uint8_t i = 0; i < MAX_TRIES; i++) {
    uint8_t twsr;

    if(TWI_start(slave_addr, &twsr, rw)) {
    	return twsr == TW_MR_SLA_ACK || twsr == TW_MT_SLA_ACK;
    } else if(twsr == TW_MT_SLA_NACK || twsr == TW_MR_DATA_NACK) {
      /* The device is busy, send stop condition to terminate the operation. */
      TWI_send_stop(&twsr);
    }
  }

  return false;
}

static inline bool
TWI_rep_start(uint8_t slave_addr, uint8_t *twsr, rw_t rw)
{
  return TWI_start(slave_addr, twsr, rw);
}

static inline void
TWI_init(void)
{
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

#endif /* _TWI_H_ */
