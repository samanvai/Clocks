/*
 * Simple talking clock / thermometer.
 *
 * See the various headers for the port connections.
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

#include "spo256.h"

// FIXME 10kHz I2C clock.
// #define SCL_CLOCK  400000L
#define SCL_CLOCK  10000L

#include "TWI.h"
#include "ds1307.h"
#include "mma7660fc.h"

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

// 1Hz tick from the RTC - PC2 - PCINT10 - PCI1.
// accelerometer event - PC3 - PCINT11 - PCI1.
ISR(PCINT1_vect)
{
  uart_putstring("PCINT1", true);
  // mma7660fc_clear_interrupt();
  // do_speak_the_time = true;
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
    speak(time);
    speak(clown);
  }
}

static void
speak_acc_reading(void)
{
  int8_t x, y, z;

  if(mma7660fc_read_axes(&x, &y, &z)) {
    uart_putstring("Acc read successful.", true);
    uart_putw_dec(x);
    uart_putstring("", true);
    uart_putw_dec(y);
    uart_putstring("", true);
    uart_putw_dec(z);
    uart_putstring("", true);

    speak(sensors);
    speak_number(x);
    speak_number(y);
    speak_number(z);
  } else {
    uart_putstring("*** Acc read failed.", true);
    speak(sensors);
    speak(clown);
  }
}

static void
dump_acc_registers(void)
{
  uint8_t twsr;
  uint8_t t;

  TWI_start(MMA7660FC_ADDR, &twsr, WRITE);
  TWI_write(&twsr, 0x0);
  TWI_rep_start(MMA7660FC_ADDR, &twsr, READ);

  for(int i = 0; i <= 0x0A; i++) {
    TWI_read(&twsr, &t, true);
    uart_putw_dec(t);
    uart_putstring("", true);
  }

  TWI_read(&twsr, &t, false);
  TWI_send_stop(&twsr);
}

int
main(void)
{
  /* Default all IO pins to inputs, no pull-ups. */
  DDRB = 0x0;
  DDRC = 0x0;
  DDRD = 0x0;

  PORTB = 0x0;
  PORTC = 0x0;
  PORTD = 0x0;

  /* By default power all the sub-modules off. */
  power_all_disable();

  /* Disable all external interrupts. */
  PCMSK0 = 0x0;
  PCMSK1 = 0x0;
  PCMSK2 = 0x0;

  PCICR = 0x0;

  uart_init();
  uart_putstring("Talking clock.", true);

  /* Interrupt when the U(S)ART receives something (and not when it sends something). */
  PCMSK2 = _BV(PCINT16);
  PCICR |= _BV(PCIE2);

  TWI_init();

  uart_debug_putstring("Initialising the RTC (ds1307)...");
  if(ds1307_init(true)) {
    uart_debug_putstring("The RTC (ds1307) is initialised.");

    /* Pull-up the RTC interrupt pin and listen for Pin Change interrupts. */
    PORTC |= _BV(PC2);
    PCMSK1 |= _BV(PCINT10);
    PCICR |= _BV(PCIE1);
  } else {
    uart_debug_putstring("** The RTC (ds1307) failed to initialise.");
  }

  uart_debug_putstring("Initialising the accelerometer (mma7660)...");
  if(mma7660fc_init()) {
    uart_debug_putstring("The accelerometer (mma7660) is initialised.");

    /* Listen for accelerometer events. */
    PCMSK1 |= _BV(PCINT11);
    PCICR |= _BV(PCIE1);
  } else {
    uart_debug_putstring("** The accelerometer (mma7660) failed to initialise.");
  }

  uart_debug_putstring("Initialising the SPO256...");
  spo256_init();
  uart_debug_putstring("The SPO256 is initialised.");

  /* Enable interrupts after initialising everything. */
  sei();

  speak(talking_clock);

  while(1) {
    speak_the_time();
    speak_acc_reading();

    /* Wait for an interrupt */
    do_speak_the_time = false;
    while(!do_speak_the_time) {
      uart_debug_putstring("go to sleep");
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);
      sleep_enable();
      sleep_cpu();
      sleep_disable();
      uart_debug_putstring("woke up");
      dump_acc_registers();
    }
  }
}
