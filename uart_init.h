/*
 * UART driver for an ATMEGA328. Note the constant names are specific
 * to this chip (and perhaps other closely related ones).
 *
 * Initialisation routines.
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

#ifndef _UART_INIT_H_
#define _UART_INIT_H_

#ifndef F_CPU
#error "Please define the cpu frequency F_CPU"
#endif

#ifndef BAUD
#error "Please define the baud rate BAUD"
#endif

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

#endif /* _UART_INIT_H_ */
