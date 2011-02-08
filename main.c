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

#define BAUD 9600
#include "uart.h"

#include "ds1307.h"
#include "ds18x20.h"
#include "spo256.h"

// FIXME abstract from 1-wire.
#include "onewire.h"

// FIXME maximum number of ds18x20 sensors.
#define MAXSENSORS 1
uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];

static
uint8_t search_sensors(void)
{
  uint8_t id[OW_ROMCODE_SIZE];
  uint8_t diff, nSensors;

  ow_reset();

  nSensors = 0;

  diff = OW_SEARCH_FIRST;
  while( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS) {
    DS18X20_find_sensor(&diff, id);

    if( diff == OW_PRESENCE_ERR ) {
      speak(no);
      speak(clown);
      break;
    }

    if( diff == OW_DATA_ERR ) {
      speak(clown);
      break;
    }

    for(uint8_t i = 0; i < OW_ROMCODE_SIZE; i++) {
      gSensorIDs[nSensors][i] = id[i];
    }

    nSensors++;
  }

  return nSensors;
}

static
void ds18x20_init(void)
{
  uint8_t nSensors = search_sensors();
  speak_number(nSensors);
  speak(sensors);
}

static
void ds18x20_read(void)
{
  // range from -550:-55.0°C to 1250:+125.0°C -> min. 6+1 chars
  int16_t decicelsius;

  DS18X20_start_meas(DS18X20_POWER_PARASITE, NULL);
  _delay_ms(DS18B20_TCONV_12BIT);
  DS18X20_read_decicelsius_single(gSensorIDs[0][0], &decicelsius);  // family-code for conversion-routine

  int8_t d = decicelsius / 10;
  uint8_t r = decicelsius % 10;

  speak(it);
  speak(is);
  speak_number(d);
  if(r > 0) {
    speak(point);
    speak_number(r);
  }
  speak(degrees);
}

/* **************************************** */

int main(void)
{
  // FIXME default all IO pins to inputs, no pull-ups.
  DDRB = 0x0;
  DDRC = 0x0;
  DDRD = 0x0;

  PORTB = 0x0;
  PORTC = 0x0;
  PORTD = 0x0;

  uart_init();
  uart_putstring("Speaking clock.", true);

  uart_debug_putstring("Initialising the RTC (ds1307)...");
  ds1307_init();
  uart_debug_putstring("The RTC (ds1307) is initialised.");

  uart_debug_putstring("Initialising the SPO256...");
  spo256_init();
  uart_debug_putstring("The SPO256 is initialised.");

  speak(talking_computer);

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

    _delay_ms(5000);
  }

  // ds18x20_read();

  // ds18x20_init();
}
