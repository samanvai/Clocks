#ifndef _DEBUG_H_
#define _DEBUG_H_

/* FIXME

Assume this isn't used too heavily, so inlining is OK.
Probably not true given the state of the code base though.

*/

#include "uart.h"

#include <stdbool.h>

#include <avr/pgmspace.h>

/* Print out a string stored in FLASH. */
static inline void
ROM_putstring(const char *str, bool nl)
{
  uint8_t i = 0;

  while(1) {
    char c = pgm_read_byte(&str[i]);
    if(c) {
      UART_write(c);
      i++;
    } else {
      break;
    }
  }

  if(nl) {
    UART_write('\n');
    // FIXME uart_putchar('\r');
  }
}

static inline void
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
// by default we stick strings in ROM to save RAM
#define putstring(x) ROM_putstring(PSTR(x), 0)
#define putstring_nl(x) ROM_putstring(PSTR(x), 1)

#define DEBUGGING 0
#define DEBUG(x)  if (DEBUGGING) { x; }
#define DEBUGP(x) DEBUG(ROM_putstring(x, true))

#endif /* _DEBUG_H_ */
