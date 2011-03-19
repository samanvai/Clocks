/*
 * SPO256 allophones.
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

FIXME note the ones that can be repeated.

 */

#ifndef _ALLOPHONES_H_
#define _ALLOPHONES_H_

typedef enum {
  aPA1 = 0x00, /* 10 ms pause (before BB, DD, GG and JH) */
  aPA2 = 0x01, /* 30 ms pause (before BB, DD, GG and JH) */
  aPA3 = 0x02, /* 50 ms pause (before PP, TT, KK and CH) and between words */
  aPA4 = 0x03, /* 100 ms pause (between clauses and sentences) */
  aPA5 = 0x04, /* 200 ms pause (between clauses and sentences) */

  aOY  = 0x05, /* bOY */
  aAY  = 0x06, /* skY */
  aEH  = 0x07, /* End */
  aKK3 = 0x08, /* Comb */
  aPP  = 0x09, /* Pow */
  aJH  = 0x0A, /* doDGe */
  aNN1 = 0x0B, /* thiN */
  aIH  = 0x0C, /* sIt */
  aTT2 = 0x0D, /* To */
  aRR1 = 0x0E, /* Rural */
  aAX  = 0x0F, /* sUcceed */
  aMM  = 0x10, /* Milk */
  aTT1 = 0x11, /* parT */
  aDH1 = 0x12, /* THey */
  aIY  = 0x13, /* sEE */
  aEY  = 0x14, /* bEIge */
  aDD1 = 0x15, /* coulD */
  aUW1 = 0x16, /* tO */
  aAO  = 0x17, /* AUght */
  aAA  = 0x18, /* hOt */
  aYY2 = 0x19, /* Yes */
  aAE  = 0x1A, /* hAt */
  aHH1 = 0x1B, /* He */
  aBB1 = 0x1C, /* Business */
  aTH  = 0x1D, /* THin */
  aUH  = 0x1E, /* bOOk */
  aUW2 = 0x1F, /* fOOd */
  aAW  = 0x20, /* OUt */
  aDD2 = 0x21, /* Do */
  aGG3 = 0x22, /* wiG */
  aVV  = 0x23, /* Vest */
  aGG1 = 0x24, /* Got */
  aSH  = 0x25, /* SHip */
  aZH  = 0x26, /* aZure */
  aRR2 = 0x27, /* bRain */
  aFF  = 0x28, /* Food */
  aKK2 = 0x29, /* sKy */
  aKK1 = 0x2A, /* Can't */
  aZZ  = 0x2B, /* Zoo */
  aNG  = 0x2C, /* aNchor */
  aLL  = 0x2D, /* Lake */
  aWW  = 0x2E, /* Wool */
  aXR  = 0x2F, /* repaiR */
  aWH  = 0x30, /* WHig */
  aYY1 = 0x31, /* Yes */
  aCH  = 0x32, /* CHurCH */
  aER1 = 0x33, /* fIR */
  aER2 = 0x34, /* fIR */
  aOW  = 0x35, /* bEAU */
  aDH2 = 0x36, /* THey */
  aSS  = 0x37, /* veSt */
  aNN2 = 0x38, /* No */
  aHH2 = 0x39, /* Hoe */
  aOR  = 0x3A, /* stORe */
  aAR  = 0x3B, /* alARm */
  aYR  = 0x3C, /* cleaR */
  aGG2 = 0x3D, /* Guest */
  aEL  = 0x3E, /* saddLE */
  aBB2 = 0x3F, /* Business */

  aEND = 0xFF, /* Sentinel for arrays of allophones. */
} allophone_t;

#endif /* _ALLOPHONES_H_ */
