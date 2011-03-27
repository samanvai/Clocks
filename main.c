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

/*

FIXME use the watch dog timer instead, say 2s.
 - if we're talking on the USART, reset the WDT
  - somehow say "if we're not lively, the WDT ISR should put the CPU to sleep"
 - otherwise just let it time out
 - check the alarm on timeout, then go back to sleep

FIXME reset from software.

FIXME slow down the CPU - 500kHz? TWI / UART might be the limit.

 */

#ifndef F_CPU
#error "Please define the cpu frequency F_CPU"
#endif

#include <stdint.h>

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

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
/* The Esterel controller defines these. */

extern void CONTROLLER_I_accelerometer_interrupt(void);
extern void CONTROLLER_I_rtc_interrupt(void);
extern void CONTROLLER_I_uart_interrupt(void);

void CONTROLLER_reset(void);
void CONTROLLER(void);

/* **************************************** */
/* Interrupt handlers */

/* SPO completion - PCINT6 - PCI0 */
ISR(PCINT0_vect)
{
  // uart_putstring(PSTR("PCINT0"), true);
}

/* accelerometer event - PC3 - PCINT11 - PCI1 */
ISR(PCINT1_vect)
{
  uart_putstring(PSTR("PCINT1"), true);
  CONTROLLER_I_accelerometer_interrupt();

  // mma7660fc_clear_interrupt();
  // do_speak_the_time = true;
}

/* U(S)ART receive activity - PCINT16 - PCI2 */
ISR(PCINT2_vect)
{
  uart_putstring(PSTR("PCINT2"), true);
  /* FIXME: if we're being talked to on the serial port, stay awake for a while. */
  wdt_reset();
}

ISR(WDT_vect) {
  uart_putstring(PSTR("WATCH DOG"), true);
  wdt_reset();
}

/* **************************************** */

static void
speak_the_time(void)
{
  struct ds1307_time_t t;

  if(ds1307_read(&t)) {
    uart_putstring(PSTR("The time is "), false);
    uart_putw_dec(t.hours);
    uart_putstring(PSTR(" hours "), false);
    uart_putw_dec(t.minutes);
    uart_putstring(PSTR(" minutes "), false);
    uart_putw_dec(t.seconds);
    uart_putstring(PSTR(" seconds"), true);

    speak_P(the);
    speak_P(time);
    speak_P(is);
    // FIXME pluralisation
    speak_number(t.hours);
    speak_P(hours);
    speak_number(t.minutes);
    speak_P(minutes);
    speak_number(t.seconds);
    speak_P(seconds);
  } else {
    uart_debug_putstring(PSTR("** ds1307 read failure"));
    speak_P(time);
    speak_P(clown);
  }
}

static void
speak_acc_reading(void)
{
  int8_t x, y, z;

  if(mma7660fc_read_axes(&x, &y, &z)) {
    uart_putstring(PSTR("Acc read successful."), true);
    uart_putw_dec(x);
    uart_putstring(PSTR(""), true);
    uart_putw_dec(y);
    uart_putstring(PSTR(""), true);
    uart_putw_dec(z);
    uart_putstring(PSTR(""), true);

    speak_P(sensors);
    speak_number(x);
    speak_number(y);
    speak_number(z);
  } else {
    uart_putstring(PSTR("*** Acc read failed."), true);
    speak_P(sensors);
    speak_P(clown);
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
    uart_putstring(PSTR(""), true);
  }

  TWI_read(&twsr, &t, false);
  TWI_send_stop(&twsr);
}

/* **************************************** */
/* Called by the Esterel code. */

void
check_alarm(void)
{
}

void
handle_accelerometer_interrupt(void)
{
}

void
sleep(void)
{
  uart_debug_putstring(PSTR("going to sleep"));
  TWI_turn_off();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();

  /* ... and when we come back ... */

  sleep_disable();
  uart_debug_putstring(PSTR("woke up"));
  TWI_init();
}

/* **************************************** */

int
main(void)
{
  /* No interruptions, thanks. */
  cli();

  /* Clear a watch-dog timer reset. */
  WDTCSR &= ~_BV(WDRF) | ~_BV(WDE);

  /* Default all IO pins to inputs, no pull-ups. */
  DDRB = 0x0;
  DDRC = 0x0;
  DDRD = 0x0;

  PORTB = 0x0;
  PORTC = 0x0;
  PORTD = 0x0;

  /* Turn off the stuff we don't use: analog comparator and ADC. */
  ACSR |= _BV(7);
  ADCSRA &= _BV(7);

  /* By default power all the sub-modules off. */
  power_all_disable();

  /* Disable all external interrupts. */
  PCMSK0 = 0x0;
  PCMSK1 = 0x0;
  PCMSK2 = 0x0;

  PCICR = 0x0;

  /* Set up the watch-dog timer: interrupt, not system reset, 8s timeout. */
  wdt_reset();
  MCUCR &= ~_BV(WDRF);
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR =  _BV(WDIE) | _BV(WDP0) | _BV(WDP3);

  uart_init(true);
  uart_putstring(PSTR("Talking clock."), true);

  /* Interrupt when the U(S)ART receives something (and not when it sends something). */
  PCMSK2 = _BV(PCINT16);
  PCICR |= _BV(PCIE2);

  /* 2-wire bus (TWI) device initialisation. */

  TWI_init();

  uart_debug_putstring(PSTR("Initialising the RTC (ds1307)..."));
  if(ds1307_init(false)) {
    uart_debug_putstring(PSTR("The RTC (ds1307) is initialised."));
  } else {
    uart_debug_putstring(PSTR("** The RTC (ds1307) failed to initialise."));
  }

  uart_debug_putstring(PSTR("Initialising the accelerometer (mma7660)..."));
  if(mma7660fc_init()) {
    uart_debug_putstring(PSTR("The accelerometer (mma7660) is initialised."));

    /* Listen for accelerometer events. */
    PCMSK1 |= _BV(PCINT11);
    PCICR |= _BV(PCIE1);
  } else {
    uart_debug_putstring(PSTR("** The accelerometer (mma7660) failed to initialise."));
  }

  uart_debug_putstring(PSTR("Initialising the SPO256..."));
  spo256_init();
  uart_debug_putstring(PSTR("The SPO256 is initialised."));

  /* Enable interrupts after initialising everything. */
  sei();

  spo256_turn_on();
  speak_P(talking_clock);
  spo256_turn_off();

  CONTROLLER_reset();

  /* FIXME this needs major structural adjustment. */
  while(1) {
    /* FIXME debugging for the moment. */
    spo256_turn_on();
    speak_the_time();
    // speak_acc_reading();
    spo256_turn_off();

    sleep();

    /* uart_debug_putstring(PSTR("Entering the Esterel controller...")); */

    /* /\* Wait for an interrupt *\/ */
    /* while(1) { */
    /*   CONTROLLER(); */
    /* } */
  }
}
