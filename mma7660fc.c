/*
 * MMA7660FC accelerometer driver for an ATMEGA328 (really any AVR with TWI hardware).
 *
 * (C)opyright 2011 Peter Gammie, peteg42 at gmail dot com. All rights reserved.
 * Commenced March 2011.
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
 * Based on Freescale's Application notes:
 *   http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA7660FC&tab=Documentation_Tab&pspll=1&SelectedAsset=Documentation&ProdMetaId=PID/DC/MMA7660FC&fromPSP=true&assetLockedForNavigation=true&componentId=2&leftNavCode=1&pageSize=25&Documentation=Documentation/00210KscRcb``Application%20Notes&fpsp=1&linkline=Application%20Notes
 * The datasheet is a bit lacking.
 */

#include <stdbool.h>
#include <stdint.h>

#include "mma7660fc.h"
#include "TWI.h"

/* **************************************** */
/* Read the tilt status. */

bool
mma7660fc_read_tilt(mma7660fc_tilt_orientation_t *o, mma7660fc_tilt_back_front_t *bf)
{
  uint8_t twsr;
  uint8_t t;

  if(!TWI_start(MMA7660FC_ADDR, &twsr, WRITE)) goto error;
  if(!TWI_write(&twsr, MMA7660FC_TILT_REG)) goto error;
  if(!TWI_rep_start(MMA7660FC_ADDR, &twsr, READ)) goto error;
  TWI_read(&twsr, &t, false);
  TWI_send_stop(&twsr);

  // FIXME abstract
  *o = (t >> 2) & 0x7;
  *bf = t & 0x3;

  return true;

 error:
  TWI_send_stop(&twsr);

  return false;
}

/* **************************************** */
/* Ask the accelerometer for the axes readings. */

static inline bool
mma7660fc_read_axis(int8_t *v, bool final)
{
  uint8_t twsr;

  // Thanks: http://graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend
  struct {signed int f:6;} t6;
  uint8_t t;

  /* If the alert bit is set a register update by the device has
   * interfered with our read. */
  if(TWI_read(&twsr, &t, final) && !(t & _BV(MMA7660FC__OUT_ALERT))) {
    // FIXME casts??
    *v = t6.f = t;
    return true;
  } else {
    return false;
  }
}

bool
mma7660fc_read_axes(int8_t *x, int8_t *y, int8_t *z)
{
  uint8_t twsr;

  for(int i = 0; i < READ_AXIS_ATTEMPTS; i++) {
    if(!TWI_start(MMA7660FC_ADDR, &twsr, WRITE)) goto error;
    if(!TWI_write(&twsr, MMA7660FC_XOUT_REG)) goto error;
    if(!TWI_rep_start(MMA7660FC_ADDR, &twsr, READ)) goto error;

    if(   mma7660fc_read_axis(x, true)
       && mma7660fc_read_axis(y, true)
       && mma7660fc_read_axis(z, false)) {
      TWI_send_stop(&twsr);

      return true;
    }

    TWI_send_stop(&twsr);
  }

 error:
  TWI_send_stop(&twsr);

  return false;
}

/* **************************************** */

static inline bool
mma7660fc_set_mode(MMA7660FC_MODE_t mode)
{
  uint8_t twsr;

  if(!TWI_start(MMA7660FC_ADDR, &twsr, WRITE)) goto error;
  if(!TWI_write(&twsr, MMA7660FC_MODE_REG)) goto error;
  if(!TWI_write(&twsr, mode)) goto error;

  TWI_send_stop(&twsr);

  return true;

 error:
  return false;
}

/* **************************************** */

/* FIXME clear the interrupt just by talking to the accelerometer. Unnecessary when we do it for real. */
bool
mma7660fc_clear_interrupt(void)
{
  uint8_t twsr;
  uint8_t t;

  if(!TWI_start(MMA7660FC_ADDR, &twsr, WRITE)) goto error;
  if(!TWI_write(&twsr, MMA7660FC_XOUT_REG)) goto error;
  if(!TWI_rep_start(MMA7660FC_ADDR, &twsr, READ)) goto error;
  if(!TWI_read(&twsr, &t, false)) goto error;
  TWI_send_stop(&twsr);

  return true;

 error:
  TWI_send_stop(&twsr);

  return false;
}

/* Initialise the MMA7660. Assumes the TWI interface is already initialised.
 * FIXME tries to set up the tap detection stuff but it doesn't seem to work.
 */
bool
mma7660fc_init_tap(void)
{
  uint8_t twsr;

  /* Device must be placed in standby mode before we can change its registers. */
  if(!mma7660fc_set_mode(MMA7660FC_MODE_STANDBY)) goto error;

  /* No sleep count. */
  if(!TWI_start(MMA7660FC_ADDR, &twsr, WRITE)) goto error;
  if(!TWI_write(&twsr, MMA7660FC_SPCNT_REG)) goto error;
  if(!TWI_write(&twsr, 0x0)) goto error;
  TWI_send_stop(&twsr);

  /* Configure tap detection interrupt. */
  if(!TWI_start(MMA7660FC_ADDR, &twsr, WRITE)) goto error;
  if(!TWI_write(&twsr, MMA7660FC_INTSU_REG)) goto error;
  if(!TWI_write(&twsr, _BV(MMA7660FC_INTSU_PDINT))) goto error;
  TWI_send_stop(&twsr);

  /* Only Z axis tap detection on, threshold +/-12 counts */
  if(!TWI_start(MMA7660FC_ADDR, &twsr, WRITE)) goto error;
  if(!TWI_write(&twsr, MMA7660FC_PDET_REG)) goto error;
  if(!TWI_write(&twsr, _BV(MMA7660FC_PDET_XDA) | _BV(MMA7660FC_PDET_YDA) | _BV(MMA7660FC_PDET_ZDA) | MMA7660FC_PDET_PDTH(12))) goto error;
  TWI_send_stop(&twsr);

  /* 120 samples/s. */
  if(!TWI_start(MMA7660FC_ADDR, &twsr, WRITE)) goto error;
  if(!TWI_write(&twsr, MMA7660FC_SR_REG)) goto error;
  if(!TWI_write(&twsr, MMA7660FC_SR_AMPD)) goto error;
  TWI_send_stop(&twsr);

  /* Tap detection debounce count = 9. */
  if(!TWI_start(MMA7660FC_ADDR, &twsr, WRITE)) goto error;
  if(!TWI_write(&twsr, MMA7660FC_PD_REG)) goto error;
  if(!TWI_write(&twsr, MMA7660FC_PD_REG_VAL(9))) goto error;
  TWI_send_stop(&twsr);

  if(!mma7660fc_set_mode(MMA7660FC_MODE_ACTIVE | _BV(MMA7660FC_MODE_IPP))) goto error;

  return true;

 error:
  TWI_send_stop(&twsr);

  return false;
}

/* Initialise the MMA7660. Assumes the TWI interface is already initialised.
 * Try out the orientation detection.
 */
bool
mma7660fc_init_Bryan(void)
{
  uint8_t twsr;

  /* Device must be placed in standby mode before we can change its registers. */
  if(!mma7660fc_set_mode(MMA7660FC_MODE_STANDBY)) goto error;

  /* No sleep count. */
  if(!TWI_start(MMA7660FC_ADDR, &twsr, WRITE)) goto error;
  if(!TWI_write(&twsr, MMA7660FC_SPCNT_REG)) goto error;
  if(!TWI_write(&twsr, 0x0)) goto error;
  TWI_send_stop(&twsr);

  /* Configure front/back and up/down/right/left interrupts */
  if(!TWI_start(MMA7660FC_ADDR, &twsr, WRITE)) goto error;
  if(!TWI_write(&twsr, MMA7660FC_INTSU_REG)) goto error;
  if(!TWI_write(&twsr, _BV(MMA7660FC_INTSU_FBINT) | _BV(MMA7660FC_INTSU_PLINT))) goto error;
  TWI_send_stop(&twsr);

  /* No tap detection. FIXME suspicious */
  if(!TWI_start(MMA7660FC_ADDR, &twsr, WRITE)) goto error;
  if(!TWI_write(&twsr, MMA7660FC_PDET_REG)) goto error;
  if(!TWI_write(&twsr, 0xE0)) goto error;
  TWI_send_stop(&twsr);

  /* 8 samples/s, TILT debounce filter = 2. FIXME */
  if(!TWI_start(MMA7660FC_ADDR, &twsr, WRITE)) goto error;
  if(!TWI_write(&twsr, MMA7660FC_SR_REG)) goto error;
  if(!TWI_write(&twsr, 0x34)) goto error;
  TWI_send_stop(&twsr);

  /* Tap detection debounce count = 0. */
  if(!TWI_start(MMA7660FC_ADDR, &twsr, WRITE)) goto error;
  if(!TWI_write(&twsr, MMA7660FC_PD_REG)) goto error;
  if(!TWI_write(&twsr, MMA7660FC_PD_REG_VAL(0))) goto error;
  TWI_send_stop(&twsr);

  if(!mma7660fc_set_mode(MMA7660FC_MODE_ACTIVE | _BV(MMA7660FC_MODE_IPP))) goto error;

  return true;

 error:
  TWI_send_stop(&twsr);

  return false;
}
