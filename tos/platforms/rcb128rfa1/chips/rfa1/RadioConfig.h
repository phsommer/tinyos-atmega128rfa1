/*
 * Copyright (c) 2007, Vanderbilt University
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE VANDERBILT UNIVERSITY BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE VANDERBILT
 * UNIVERSITY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE VANDERBILT UNIVERSITY SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE VANDERBILT UNIVERSITY HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 *
 * Author: Miklos Maroti
 */

#ifndef __RADIOCONFIG_H__
#define __RADIOCONFIG_H__

#include <MicaTimer.h>
#include <RFA1DriverLayer.h>
#include "HplAtmRfa1Timer.h"
#include <util/crc16.h>

enum
{
	
	/**
	 * This is the default value of the CCA_MODE field in the PHY_CC_CCA register
	 * which is used to configure the default mode of the clear channel assessment
	 */
	RFA1_CCA_MODE_VALUE = CCA_CS_AND_ED,

	/**
	 * This is the value of the CCA_THRES register that controls the
	 * energy levels used for clear channel assessment
	 */
	RFA1_CCA_THRES_VALUE = 0xC7,
};

/* This is the default value of the TX_PWR field of the PHY_TX_PWR register. */
#ifndef RFA1_DEF_RFPOWER
#define RFA1_DEF_RFPOWER	0
#endif

/* This is the default value of the CHANNEL field of the PHY_CC_CCA register. */
#ifndef RFA1_DEF_CHANNEL
#define RFA1_DEF_CHANNEL	26
#endif

/* The number of microseconds a sending mote will wait for an acknowledgement */
#ifndef SOFTWAREACK_TIMEOUT
#define SOFTWAREACK_TIMEOUT	5000
#endif

/*
 * This is the command used to calculate the CRC for the RFA1 chip. 
 * TODO: Check why the default crcByte implementation is in a different endianness
 */
inline uint16_t RFA1_CRCBYTE_COMMAND(uint16_t crc, uint8_t data)
{
	return _crc_ccitt_update(crc, data);
}

/**
 * This is the timer type of the radio alarm interface
 */
typedef T62khz TRadio;

/**
 * The number of radio alarm ticks per one microsecond
 */
#define RADIO_ALARM_MICROSEC	62500UL / 1000000UL

/**
 * The base two logarithm of the number of radio alarm ticks per one millisecond
 */
#define RADIO_ALARM_MILLI_EXP	4

/**
 * Make PACKET_LINK automaticaly enabled for Ieee154MessageC
 */
#if !defined(TFRAMES_ENABLED) && !defined(PACKET_LINK)
#define PACKET_LINK
#endif

#endif//__RADIOCONFIG_H__
