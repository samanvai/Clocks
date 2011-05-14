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

#include <stdbool.h>
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

extern void CONTROLLER_I_wdt_event(void);
extern void CONTROLLER_I_accelerometer_event(void);
extern void CONTROLLER_I_uart_event(void);

void CONTROLLER_reset(void);
void CONTROLLER(void);

/* **************************************** */
/* FIXME double-buffer events. */

struct events_t {
  bool event_accelerometer:1;
  bool event_uart:1;
  bool event_wdt:1;
};

static struct events_t events;

/* **************************************** */
/* Interrupt handlers */

/* SPO completion - PCINT6 - PCI0 */
ISR(PCINT0_vect)
{
  // uart_debug_putstringP(PSTR("PCINT0"));
}

/* accelerometer event - PC3 - PCINT11 - PCI1 */
ISR(PCINT1_vect)
{
  uart_debug_putstringP(PSTR("PCINT1"));
  events.event_accelerometer = true;

  // mma7660fc_clear_interrupt();
  // do_speak_the_time = true;
}

/* U(S)ART receive activity - PCINT16 - PCI2 */
ISR(PCINT2_vect)
{
  uart_debug_putstringP(PSTR("PCINT2"));
  /* FIXME: if we're being talked to on the serial port, stay awake for a while. */
  wdt_reset();
  events.event_uart = true;
}

/* Watch-dog timeout. */
ISR(WDT_vect) {
  uart_debug_putstringP(PSTR("WATCH DOG"));
  wdt_reset();
  events.event_wdt = true;
}

/* **************************************** */
/* Debugging */

static void
dump_acc_registers(void)
{
  uint8_t twsr;
  uint8_t t;

  TWI_start(MMA7660FC_ADDR, &twsr, WRITE);
  TWI_write(&twsr, 0x0);
  TWI_rep_start(MMA7660FC_ADDR, &twsr, READ);

  for(uint8_t i = 0; i <= 0x0A; i++) {
    TWI_read(&twsr, &t, true);
    uart_putw_dec(t);
    uart_tx_nl();
  }

  TWI_read(&twsr, &t, false);
  TWI_send_stop(&twsr);
}

/* **************************************** */
/* Esterel call backs. These are guaranteed not to be re-entered, so
   cheesy global state will do. */

#define BUF_SIZE 4
char buf[BUF_SIZE];
uint8_t buf_index = 0;

void
handle_uart_reset(void)
{
  uart_debug_putstringP(PSTR("handle_uart_reset()"));
  buf_index = 0;
}

void
handle_uart_event(void)
{
  uint8_t c;

  uart_debug_putstringP(PSTR("handle_uart_event()"));

  /* Wait for a character to be received. */
  while(!uart_rx(&c))
    ;

  buf[buf_index] = c;
  buf_index++;

  uart_debug_putstringP(PSTR("handle_uart_event(): got character"));

  if(buf_index == BUF_SIZE) {
    uart_debug_putstringP(PSTR("handle_uart_event(): dumping buffer"));

    for(int i = 0; i < BUF_SIZE; i++) {
      uart_tx(buf[i]);
    }
    uart_tx_nl();

    buf_index = 0;

    dump_acc_registers();
  }

  uart_debug_putstringP(PSTR("handle_uart_event() finished"));
}

void
handle_accelerometer_event(void)
{
  uart_debug_putstringP(PSTR("handle_accelerometer_event()"));
}

void
check_alarm(void)
{
  uart_debug_putstringP(PSTR("check_alarm()"));
}

void
speak_the_time(void)
{
  struct ds1307_time_t t;

  if(ds1307_read(&t)) {
    uart_putstringP(PSTR("The time is "), false);
    uart_putw_dec(t.hours);
    uart_putstringP(PSTR(" hours "), false);
    uart_putw_dec(t.minutes);
    uart_putstringP(PSTR(" minutes "), false);
    uart_putw_dec(t.seconds);
    uart_putstringP(PSTR(" seconds"), true);

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
    uart_debug_putstringP(PSTR("** ds1307 read failure"));
    speak_P(time);
    speak_P(clown);
  }
}

static void
speak_acc_reading(void)
{
  int8_t x, y, z;

  if(mma7660fc_read_axes(&x, &y, &z)) {
    uart_putstringP(PSTR("Acc read successful."), true);
    uart_putw_dec(x);
    uart_putstringP(PSTR(""), true);
    uart_putw_dec(y);
    uart_putstringP(PSTR(""), true);
    uart_putw_dec(z);
    uart_putstringP(PSTR(""), true);

    speak_P(sensors);
    speak_number(x);
    speak_number(y);
    speak_number(z);
  } else {
    uart_putstringP(PSTR("*** Acc read failed."), true);
    speak_P(sensors);
    speak_P(clown);
  }
}

/* **************************************** */

void
sleep(void)
{
  uart_debug_putstringP(PSTR("going to sleep"));
  TWI_turn_off();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();

  /* ... and when we come back ... */

  sleep_disable();
  uart_debug_putstringP(PSTR("woke up"));
  TWI_init();
}

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

  /* Set up the watch-dog timer: interrupt (do not reset the system), 8s timeout. */
  wdt_reset();
  MCUCR &= ~_BV(WDRF);
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR =  _BV(WDIE) | _BV(WDP0) | _BV(WDP3);

  uart_init();
  uart_putstringP(PSTR("Talking clock."), true);

  /* Interrupt when the U(S)ART receives something (and not when it sends something). */
  PCMSK2 = _BV(PCINT16);
  PCICR |= _BV(PCIE2);

  /* 2-wire bus (TWI) device initialisation. */

  TWI_init();

  uart_debug_putstringP(PSTR("Initialising the RTC (ds1307)..."));
  if(ds1307_init(false)) {
    uart_debug_putstringP(PSTR("The RTC (ds1307) is initialised."));
  } else {
    uart_debug_putstringP(PSTR("** The RTC (ds1307) failed to initialise."));
  }

  uart_debug_putstringP(PSTR("Initialising the accelerometer (mma7660)..."));
  if(mma7660fc_init()) {
    uart_debug_putstringP(PSTR("The accelerometer (mma7660) is initialised."));

    /* Listen for accelerometer events. */
    PCMSK1 |= _BV(PCINT11);
    PCICR |= _BV(PCIE1);
  } else {
    uart_debug_putstringP(PSTR("** The accelerometer (mma7660) failed to initialise."));
  }

  uart_debug_putstringP(PSTR("Initialising the SPO256..."));
  spo256_init();
  uart_debug_putstringP(PSTR("The SPO256 is initialised."));

  /* Enable interrupts after initialising everything. */
  sei();

  spo256_turn_on();
  speak_P(talking_clock);
  spo256_turn_off();

  uart_debug_putstringP(PSTR("Resetting the Esterel controller."));
  CONTROLLER_reset();

  while(1) {
    sleep();

    /*

       The Esterel controller cannot cope with new events occurring
       while it is processing a reaction.

       One non-solution: disable interrupts while it is running. This
       also disables interrupts while we do what the controller tells
       us to do, which might limit implementation options there.

       So double-buffering it must be. Marshall the events here.

    */

    if(events.event_accelerometer) {
      CONTROLLER_I_accelerometer_event();
      events.event_accelerometer = 0;
    }
    if(events.event_uart) {
      CONTROLLER_I_uart_event();
      events.event_uart = 0;
    }
    if(events.event_wdt) {
      CONTROLLER_I_wdt_event();
      events.event_wdt = 0;
    }

    uart_debug_putstringP(PSTR("Entering the Esterel controller."));
    CONTROLLER();
    uart_debug_putstringP(PSTR("Exiting the Esterel controller."));
  }

    /* FIXME debugging for the moment. */
    // spo256_turn_on();
    // speak_the_time();
    // speak_acc_reading();
    // spo256_turn_off();
}
