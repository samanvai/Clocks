#include "ds18x20.h"
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
