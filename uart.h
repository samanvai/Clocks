/*
 * UART driver for an ATMEGA328. Note the constant names are specific
 * to this chip (and perhaps other closely related ones).
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

FIXME add TX queue.

 */

#ifndef _UART_H_
#define _UART_H_

#include <stdbool.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/pgmspace.h>

#ifndef F_CPU
#error "Please define the cpu frequency F_CPU"
#endif

#ifndef BAUD
#error "Please define the baud rate BAUD"
#endif

/* **************************************** */
/* Primitive synchronous blocking read. */

/* static inline uint8_t */
/* uart_read(void) */
/* { */
/*   while(! (UCSR0A & _BV(RXC0))) */
/*       ; */
/*   return UDR0; */
/* } */

/* **************************************** */
/* Interrupts and FIFO buffers, asynchronous non-blocking read/write. */

#define RX_FIFO_LEN 8
#define TX_FIFO_LEN 8

static volatile uint8_t rx_fifo[RX_FIFO_LEN];
static volatile uint8_t rx_fifo_head, rx_fifo_tail; /* implicitly initialized to 0 */

/* The UART completely received something. */

  /* FIXME ignore the issue of overflow. This is racey: Drop old data
     when we overflow. Easiest to drop new data on the floor... but
     last-data wins is probably nicer semantics. Maybe just flushing
     on every overflow is not so bad either.

     I think this code does this last thing.

  */

ISR(USART_RX_vect)
{
  /* Clears the RXC0 flag. */
  rx_fifo[rx_fifo_tail] = UDR0;
  rx_fifo_tail = (rx_fifo_tail + 1) % RX_FIFO_LEN;

  /* if(rx_fifo_tail == rx_fifo_head) { */
  /*   rx_fifo_head = (rx_fifo_head + 1) % RX_FIFO_LEN; */
  /* } */
}

static inline bool
uart_rx(uint8_t *v)
{
  if(rx_fifo_tail != rx_fifo_head) {
    *v = rx_fifo[rx_fifo_head];
    rx_fifo_head = (rx_fifo_head + 1) % RX_FIFO_LEN;
    return true;
  } else {
    return false;
  }
}

static inline void
uart_tx(uint8_t c)
{
  while(! (UCSR0A & _BV(UDRE0)))
      ;
  UDR0 = c;
}

/* **************************************** */

static inline void
uart_init(void)
{
  /* Fire up the UART module. */
  power_usart0_enable();

#include <util/setbaud.h>

  /* Set the baud rate. */
  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;

  /* 8 N 1 */
  UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);

  /* Turn on the transmission and reception circuitry, Receive Complete Interrupt Enable. */
  UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);
  /* Note the USART Data Register Empty Interrupt Enable gets set by the tx routines. */

#if USE_2X
  UCSR0A |= (1 << U2X0);
#else
  UCSR0A &= ~(1 << U2X0);
#endif
}

/* **************************************** */

static void
uart_tx_nl(void)
{
  uart_tx('\n');
  uart_tx('\r');
}

/* Print out a string stored in FLASH. */
static void
uart_putstringP(const prog_char *str, bool nl)
{
  uint8_t i = 0;

  while(1) {
    char c = pgm_read_byte(&str[i]);
    if(c != '\0') {
      uart_tx(c);
      i++;
    } else {
      break;
    }
  }

  if(nl) {
    uart_tx_nl();
  }
}

static void
uart_putw_dec(uint16_t w)
{
  uint16_t num = 10000;
  uint8_t started = 0;

  while(num > 0)
    {
      uint8_t b = w / num;
      if(b > 0 || started || num == 1)
	{
	  uart_tx('0' + b);
	  started = 1;
	}
      w -= b * num;

      num /= 10;
    }
}

// FIXME
#define IF_DEBUG(x)  if(DEBUG) { x; }
#define uart_debug_putstringP(x) IF_DEBUG(uart_putstringP(x, true))

// by default we stick strings in ROM to save RAM
// #define putstring(x) ROM_putstring(PSTR(x), 0)
// #define putstring_nl(x) ROM_putstring(PSTR(x), 1)

#endif /* _UART_H_ */
