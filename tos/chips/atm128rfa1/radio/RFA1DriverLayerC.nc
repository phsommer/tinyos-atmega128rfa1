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

#include <RadioConfig.h>
#include <RFA1DriverLayer.h>

configuration RFA1DriverLayerC
{
	provides
	{
		interface RadioState;
		interface RadioSend;
		interface RadioReceive;
		interface RadioCCA;
		interface RadioPacket;

		interface PacketField<uint8_t> as PacketTransmitPower;
		interface PacketField<uint8_t> as PacketRSSI;
		interface PacketField<uint8_t> as PacketTimeSyncOffset;
		interface PacketField<uint8_t> as PacketLinkQuality;
		
		interface LocalTime<TRadio> as LocalTimeRadio;

	}

	uses
	{
		interface RFA1DriverConfig as Config;
		interface PacketTimeStamp<TRadio, uint32_t>;

		interface PacketFlag as TransmitPowerFlag;
		interface PacketFlag as RSSIFlag;
		interface PacketFlag as TimeSyncFlag;
	}
}

implementation
{
	components RFA1DriverLayerP, TaskletC, MainC;

	RadioState = RFA1DriverLayerP;
	RadioSend = RFA1DriverLayerP;
	RadioReceive = RFA1DriverLayerP;
	RadioCCA = RFA1DriverLayerP;
	RadioPacket = RFA1DriverLayerP;

	Config = RFA1DriverLayerP;

	PacketTransmitPower = RFA1DriverLayerP.PacketTransmitPower;
	TransmitPowerFlag = RFA1DriverLayerP.TransmitPowerFlag;

	PacketRSSI = RFA1DriverLayerP.PacketRSSI;
	RSSIFlag = RFA1DriverLayerP.RSSIFlag;

	PacketTimeSyncOffset = RFA1DriverLayerP.PacketTimeSyncOffset;
	TimeSyncFlag = RFA1DriverLayerP.TimeSyncFlag;

	PacketLinkQuality = RFA1DriverLayerP.PacketLinkQuality;
	PacketTimeStamp = RFA1DriverLayerP.PacketTimeStamp;

	components LocalTime62khzC;
	RFA1DriverLayerP.LocalTime -> LocalTime62khzC;
	LocalTimeRadio = LocalTime62khzC;

#ifdef RFA1_TIMESTAMP_RTC
	components HplAtmRfa1TimerMacC;
	RFA1DriverLayerP.SfdCapture -> HplAtmRfa1TimerMacC.SfdCapture;
#endif

#ifdef RFA1_TIMESTAMP_MCU
        components HplAtmRfa1Timer1C;
	RFA1DriverLayerP.SfdCapture -> HplAtmRfa1Timer1C.Capture;
#endif

	RFA1DriverLayerP.Tasklet -> TaskletC;

#ifdef RADIO_DEBUG
	components DiagMsgC;
	RFA1DriverLayerP.DiagMsg -> DiagMsgC;
#endif

	MainC.SoftwareInit -> RFA1DriverLayerP.SoftwareInit;

	components RealMainP;
	RealMainP.PlatformInit -> RFA1DriverLayerP.PlatformInit;
}
