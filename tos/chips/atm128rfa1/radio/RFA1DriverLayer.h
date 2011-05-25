/*
 * Copyright (c) 2007, Vanderbilt University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holder nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Miklos Maroti
 * Author: Andras Biro
 */

#ifndef __RFA1DRIVERLAYER_H__
#define __RFA1DRIVERLAYER_H__

typedef nx_struct rfa1_header_t
{
	nxle_uint8_t length;
} rfa1_header_t;

typedef struct rfa1_metadata_t
{
	uint8_t lqi;
	union
	{
		uint8_t power;
		uint8_t rssi;
	};
} rfa1_metadata_t;


enum rfa1_trx_status_enums
{
	RFA1_CCA_DONE = 1 << 7,
	RFA1_CCA_STATUS = 1 << 6,
	RFA1_TRX_STATUS_MASK = 0x1F,
	RFA1_P_ON = 0,
	RFA1_BUSY_RX = 1,
	RFA1_BUSY_TX = 2,
	RFA1_RX_ON = 6,
	RFA1_TRX_OFF = 8,
	RFA1_PLL_ON = 9,
	RFA1_SLEEP = 15,
	RFA1_BUSY_RX_AACK = 17,
	RFA1_BUSR_TX_ARET = 18,
	RFA1_RX_AACK_ON = 22,
	RFA1_TX_ARET_ON = 25,
	RFA1_RX_ON_NOCLK = 28,
	RFA1_AACK_ON_NOCLK = 29,
	RFA1_BUSY_RX_AACK_NOCLK = 30,
	RFA1_STATE_TRANSITION_IN_PROGRESS = 31,
	RFA1_TRAC_STATUS_MASK = 0xE0,
	RFA1_TRAC_SUCCESS = 0,
	RFA1_TRAC_SUCCESS_DATA_PENDING = 1 << 5,
	RFA1_TRAC_CHANNEL_ACCESS_FAILURE = 3 << 5,
	RFA1_TRAC_NO_ACK = 5 << 5,
	RFA1_TRAC_INVALID = 7 << 5,
	RFA1_TRX_CMD_MASK = 0x1F,
	RFA1_NOP = 0,
	RFA1_TX_START = 2,
	RFA1_FORCE_TRX_OFF = 3,
};

enum rfa1_phy_register_enums
{
	RFA1_TX_PWR_MASK = 0x0F,
	RFA1_TX_AUTO_CRC_ON = 1 << 5,
	RFA1_RSSI_MASK = 0x1F,
	RFA1_CCA_REQUEST = 1 << 7,
	RFA1_CCA_MODE_0 = 0 << 5,
	RFA1_CCA_MODE_1 = 1 << 5,
	RFA1_CCA_MODE_2 = 2 << 5,
	RFA1_CCA_MODE_3 = 3 << 5,
	RFA1_CHANNEL_DEFAULT = 11,
	RFA1_CHANNEL_MASK = 0x1F,
	RFA1_CCA_CS_THRES_SHIFT = 4,
	RFA1_CCA_ED_THRES_SHIFT = 0,
};

enum rfa1_control_register_enums
{
	RFA1_AVREG_EXT = 1 << 7,
	RFA1_AVDD_OK = 1 << 6,
	RFA1_DVREG_EXT = 1 << 3,
	RFA1_DVDD_OK = 1 << 2,
	RFA1_BATMON_OK = 1 << 5,
	RFA1_BATMON_VHR = 1 << 4,
	RFA1_BATMON_VTH_MASK = 0x0F,
	RFA1_XTAL_MODE_OFF = 0 << 4,
	RFA1_XTAL_MODE_EXTERNAL = 4 << 4,
	RFA1_XTAL_MODE_INTERNAL = 15 << 4,
};

enum rfa1_pll_register_enums
{
	RFA1_PLL_CF_START = 1 << 7,
	RFA1_PLL_DCU_START = 1 << 7,
};

#endif//__RFA1DRIVERLAYER_H__
