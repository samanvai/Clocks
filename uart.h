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
 */

#ifndef _UART_H_
#define _UART_H_

#include <stdbool.h>
#include <stdint.h>

#include <avr/io.h>

#ifndef F_CPU
#error "Please define the cpu frequency F_CPU"
#endif

#ifndef BAUD
#error "Please define the baud rate BAUD"
#endif

static inline uint8_t
uart_read(void)
{
  while(! (UCSR0A & _BV(RXC0)))
      ;
  return UDR0;
}

static inline void
uart_write(uint8_t c)
{
  while(! (UCSR0A & _BV(UDRE0)))
      ;
  UDR0 = c;
}

static inline void
uart_init(void)
{
  /* Fire up the UART module. */
  power_usart0_enable();

#include <util/setbaud.h>

  /* Set the baud rate */
  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;

  /* Turn on the transmission and reception circuitry: 8 N 1. */
  UCSR0B = _BV(RXEN0) | _BV(TXEN0);
  UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);

  /* FIXME Receive Complete Interrupt Enable */
  UCSR0B |= _BV(RXCIE0);

#if USE_2X
  UCSR0A |= (1 << U2X0);
#else
  UCSR0A &= ~(1 << U2X0);
#endif
}

/* Print out a string stored in FLASH. */
static void
uart_putstring(const char *str, bool nl)
{
  uint8_t i = 0;

  while(1) {
    // char c = pgm_read_byte(&str[i]);
    uint8_t c = str[i];
    if(c) {
      uart_write(c);
      i++;
    } else {
      break;
    }
  }

  if(nl) {
    uart_write('\n');
    uart_write('\r');
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
	  uart_write('0' + b);
	  started = 1;
	}
      w -= b * num;

      num /= 10;
    }
}

// FIXME
#define DEBUGGING 1
#define DEBUG(x)  if(DEBUGGING) { x; }
#define uart_debug_putstring(x) DEBUG(uart_putstring(x, true))

// by default we stick strings in ROM to save RAM
// #define putstring(x) ROM_putstring(PSTR(x), 0)
// #define putstring_nl(x) ROM_putstring(PSTR(x), 1)

#endif /* _UART_H_ */
