/*
 * Simple talking clock / thermometer.
 *
 * Synchronous, no interrupts in sight.
 *
 * See the various headers for the port connections.
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

/*

FIXME probably makes more sense to use Port C (only 6 bits) as I don't
need the ADC.

Where does the I2C hook into? PC4-5, so no Port C for me...

So use Port C instead of PortB? Interrupts are where? Want an
interrupt for the SBY line? PCINT is a blunderbuss. What are the
options?

 */

#ifndef F_CPU
#error "Please define the cpu frequency F_CPU"
#endif

#include <stdint.h>

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>

#define BAUD 9600
#include "uart.h"

#include "ds1307.h"
#include "spo256.h"

/* **************************************** */

volatile bool do_speak_the_time = true;

/* FIXME which PC int? Any?
 - UART activity - PCINT16 - PCI2
 - SPO completion - PCINT6 - PCI0
 */
ISR(PCINT0_vect)
{
  // uart_putstring("PCINT0", true);
}

// 1Hz tick from the RTC - PC1 - PCINT11 - PCI1.
ISR(PCINT1_vect)
{
  // uart_putstring("PCINT1", true);
}

// FIXME UART activity: need to wake up for a while before we get RX interrupts.
ISR(PCINT2_vect)
{
  uart_putstring("PCINT2", true);
  do_speak_the_time = true;
}

ISR(USART_RX_vect)
{
  uint8_t c;
  c = UDR0;
  sei();
  uart_putstring("USART_RX ", false);
  uart_write(c);
  uart_putstring("", true);
  do_speak_the_time = true;
}

static void
speak_the_time(void)
{
  struct ds1307_time_t t;

  if(ds1307_read(&t)) {
    uart_putstring("The time is ", false);
    uart_putw_dec(t.hours);
    uart_putstring(" hours ", false);
    uart_putw_dec(t.minutes);
    uart_putstring(" minutes ", false);
    uart_putw_dec(t.seconds);
    uart_putstring(" seconds", true);

    speak(the);
    speak(time);
    speak(is);
    // FIXME pluralisation
    speak_number(t.hours);
    speak(hours);
    speak_number(t.minutes);
    speak(minutes);
    speak_number(t.seconds);
    speak(seconds);
  } else {
    uart_debug_putstring("** ds1307 read failure");
    speak(no);
  }
}

int
main(void)
{
  // FIXME default all IO pins to inputs, no pull-ups.
  DDRB = 0x0;
  DDRC = 0x0;
  DDRD = 0x0;

  PORTB = 0x0;
  PORTC = 0x0;
  PORTD = 0x0;

  // FIXME pull-up the RTC interrupt pin.
  PORTC |= _BV(PC3);

  /* Initialise the power management register. */
  power_all_disable();
  power_twi_enable();
  power_usart0_enable();

  uart_init();
  uart_putstring("Talking clock.", true);

  uart_debug_putstring("Initialising the RTC (ds1307)...");
  ds1307_init();
  uart_debug_putstring("The RTC (ds1307) is initialised.");

  uart_debug_putstring("Initialising the SPO256...");
  spo256_init();
  uart_debug_putstring("The SPO256 is initialised.");

  /* Enable interrupts after initialising everything. */
  /* Enable the pin-change interrupts on all pins. FIXME refine. */
  PCMSK2 = _BV(PCINT16); // Only when the UART receives something (and not when it sends something).
  PCMSK1 = _BV(PCINT11); // Just the RTC interrupt, not the TWI bus.
  PCMSK0 = 0xFF;
  PCICR |= _BV(PCIE2) | _BV(PCIE1) | _BV(PCIE0);
  sei();

  speak(talking_clock);

  /* for(int i = 0; i < 256; i++) { */
  /*   speak_number(i); */
  /* } */

  /* { */
  /*   struct ds1307_time_t t; */
  /*   t.seconds = 13; */
  /*   t.minutes = 27; */
  /*   t.hours = 17; */
  /*   ds1307_write(&t); */
  /* } */

  while(1) {
    speak_the_time();

    do_speak_the_time = false;
    while(!do_speak_the_time) {
      uart_debug_putstring("wake up");
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);
      sleep_enable();
      sleep_cpu();
      sleep_disable();
    }
  }
}
