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

#ifndef _mma7660fc_H_
#define _mma7660fc_H_

#include <stdbool.h>
#include <stdint.h>

#include <avr/io.h>

/* **************************************** */

#define READ_AXIS_ATTEMPTS 3

/* 7-bit twi address 0b1001100. Note: shifted left 1, as the LSB (8th) is the read/write bit. */
#define MMA7660FC_ADDR  0x98

/* Registers */
#define MMA7660FC_XOUT_REG  0x00
#define MMA7660FC_YOUT_REG  0x01
#define MMA7660FC_ZOUT_REG  0x02

#define MMA7660FC__OUT_ALERT   6

/* Tilt status */
#define MMA7660FC_TILT_REG  0x03

#define MMA7660FC_TILT_BaFro   0
#define MMA7660FC_TILT_PoLA    2
#define MMA7660FC_TILT_TAP     5
#define MMA7660FC_TILT_ALERT   6
#define MMA7660FC_TILT_SHAKE   7

/* Sampling rate status */
#define MMA7660FC_SRST_REG  0x04

/* Sleep count */
#define MMA7660FC_SPCNT_REG 0x05

/* Interrupts */
#define MMA7660FC_INTSU_REG 0x06

#define MMA7660FC_INTSU_FBINT  0
#define MMA7660FC_INTSU_PLINT  1
#define MMA7660FC_INTSU_PDINT  2
#define MMA7660FC_INTSU_ASINT  3
#define MMA7660FC_INTSU_GINT   4
#define MMA7660FC_INTSU_SHINTZ 5
#define MMA7660FC_INTSU_SHINTY 6
#define MMA7660FC_INTSU_SHINTX 7

/* Mode register. */
#define MMA7660FC_MODE_REG  0x07

/* Modes bits, use the mma7660fc_mode_t enum below. */
#define MMA7660FC_MODE_MODE  0
#define MMA7660FC_MODE_TON   2
/* Auto-Wake disabled (0) or enabled (1) */
#define MMA7660FC_MODE_AWE   3
/* Auto-Sleep disabled (0) or enabled (1) */
#define MMA7660FC_MODE_ASE   4
/* FIXME clock scaling */
#define MMA7660FC_MODE_SCPS  5
/* Interrupt is open-drain (0) or push-pull (1) */
#define MMA7660FC_MODE_IPP   6
/* Interrupt is active-low (0) or active-high (1) */
#define MMA7660FC_MODE_IAH   7

typedef enum {
  MMA7660FC_MODE_STANDBY = 0x0,
  MMA7660FC_MODE_TEST    = _BV(MMA7660FC_MODE_TON),
  MMA7660FC_MODE_ACTIVE  = _BV(MMA7660FC_MODE_MODE)
} MMA7660FC_MODE_t;

/* Auto-wake/sleep, samples, debounce filter register. */
#define MMA7660FC_SR_REG    0x08

typedef enum {
  /*
Tap Detection Mode and 120 Samples/Second Active and Auto-Sleep Mode
Tap Detection Sampling Rate: The device takes readings continually at
a rate of nominally 3846 g-cell measurements a second. It then filters
these high speed measurements by maintaining continuous rolling
averages of the current and last g-cell measurements. The averages are
updated every 260 μs to track fast moving accelerations. Tap
detection: itself compares the two filtered axis responses (fast and
slow) described above for each axis. The absolute (unsigned)
difference between the fast and slow axis responses is compared
against the tap detection delta threshold value PDTH[4:0] in the PDET
(0x09) register. For portrait/landscape detection: The device takes
and averages 32 g-cell measurements every 8.36 ms in Active Mode and
Auto-Sleep. The update rate is 120 samples per second. These
measurements update the XOUT (0x00), YOUT (0x01), and ZOUT (0x02)
registers also.
  */
  MMA7660FC_SR_AMPD = 0x0,
  /*
8 Samples/Second Auto-Wake Mode For portrait/landscape detection: The
device takes and averages 32 g-cell measurements every 125 ms in
Auto-Wake. The update rate is 8 samples per second. These measurements
update the XOUT (0x00), YOUT (0x01), and ZOUT (0x02) registers also.
   */
  MMA7660FC_SR_AM8 = _BV(4),
  /*
4 measurement samples at the rate set by AMSR[2:0] or AWSR[1:0] have
to match before the device updates portrait/ landscape data in TILT
(0x03) register.
   */
  MMA7660FC_SR_FILT4 = _BV(6) | _BV(5),
} mma7660fc_sr_t;

/* Tap detection register. */
#define MMA7660FC_PDET_REG  0x09

/* bits 0-4 are the count register */
#define MMA7660FC_PDET_PDTH(x) ((x) & 0x1F)
#define MMA7660FC_PDET_XDA  5
#define MMA7660FC_PDET_YDA  6
#define MMA7660FC_PDET_ZDA  7

/* Tap debounce count. */
#define MMA7660FC_PD_REG    0x0A

#define MMA7660FC_PD_REG_VAL(x) ((x) <= 2 ? 1 : (x) - 1)

/* **************************************** */

bool mma7660fc_read_axes(int8_t *x, int8_t *y, int8_t *z);

/* FIXME clear the interrupt just by talking to the accelerometer. Unnecessary when we do it for real. */
bool mma7660fc_clear_interrupt(void);

/* FIXME clean up */
typedef enum {
  MMA7660FC_tilt_left = 1,
  MMA7660FC_tilt_right = 2,
  MMA7660FC_tilt_down = 5,
  MMA7660FC_tilt_up = 6,
} mma7660fc_tilt_orientation_t;

/* FIXME clean up */
typedef enum {
  MMA7660FC_tilt_front = 1,
  MMA7660FC_tilt_back = 2,
} mma7660fc_tilt_back_front_t;

bool mma7660fc_read_tilt(mma7660fc_tilt_orientation_t *o, mma7660fc_tilt_back_front_t *bf);

/* Initialise the MMA7660. Assumes the TWI interface is already initialised. */
bool mma7660fc_init_tap(void);

bool mma7660fc_init_Bryan(void);

#endif /* _mma7660fc_H_ */
