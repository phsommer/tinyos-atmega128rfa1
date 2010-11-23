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
	}
}

implementation
{
	components RFA1DriverLayerP, BusyWaitMicroC, TaskletC, MainC;

	RadioState = RFA1DriverLayerP;
	RadioSend = RFA1DriverLayerP;
	RadioReceive = RFA1DriverLayerP;
	RadioCCA = RFA1DriverLayerP;
	RadioPacket = RFA1DriverLayerP;

	Config = RFA1DriverLayerP;

	PacketTransmitPower = RFA1DriverLayerP.PacketTransmitPower;
	components new MetadataFlagC() as TransmitPowerFlagC;
	RFA1DriverLayerP.TransmitPowerFlag -> TransmitPowerFlagC;

	PacketRSSI = RFA1DriverLayerP.PacketRSSI;
	components new MetadataFlagC() as RSSIFlagC;
	RFA1DriverLayerP.RSSIFlag -> RSSIFlagC;

	PacketTimeSyncOffset = RFA1DriverLayerP.PacketTimeSyncOffset;
	components new MetadataFlagC() as TimeSyncFlagC;
	RFA1DriverLayerP.TimeSyncFlag -> TimeSyncFlagC;

	PacketLinkQuality = RFA1DriverLayerP.PacketLinkQuality;
	PacketTimeStamp = RFA1DriverLayerP.PacketTimeStamp;

	components LocalTime62khzC;
	RFA1DriverLayerP.LocalTime -> LocalTime62khzC;
	LocalTimeRadio = LocalTime62khzC;

	RFA1DriverLayerP.Tasklet -> TaskletC;
	RFA1DriverLayerP.BusyWait -> BusyWaitMicroC;

#ifdef RADIO_DEBUG
	components DiagMsgC;
	RFA1DriverLayerP.DiagMsg -> DiagMsgC;
#endif

	MainC.SoftwareInit -> RFA1DriverLayerP.SoftwareInit;

	components RealMainP;
	RealMainP.PlatformInit -> RFA1DriverLayerP.PlatformInit;
}
