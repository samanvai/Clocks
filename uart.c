/* FIXME

*/

#include "uart.h"

#include <stdbool.h>

#include <avr/pgmspace.h>

/* Print out a string stored in FLASH. */
void
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

void
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
