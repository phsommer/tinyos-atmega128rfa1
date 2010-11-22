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
 * Author: Andras Biro
 * Author: Philipp Sommer
 */

#include <RFA1DriverLayer.h>
#include <Tasklet.h>
#include <RadioAssert.h>
#include <TimeSyncMessageLayer.h>
#include <RadioConfig.h>


/*
*  Preliminary implementation of the RFA1 radio driver
*
*  TOOD: Power reduction settings in PRR1 (PRTRX24 bit)
*  TODO: power management (McuPowerOverride interface)
*/


module RFA1DriverLayerP
{
	provides
	{
		interface Init as PlatformInit @exactlyonce();
		interface Init as SoftwareInit @exactlyonce();

		interface RadioState;
		interface RadioSend;
		interface RadioReceive;
		interface RadioCCA;
		interface RadioPacket;

		interface PacketField<uint8_t> as PacketTransmitPower;
		interface PacketField<uint8_t> as PacketRSSI;
		interface PacketField<uint8_t> as PacketTimeSyncOffset;
		interface PacketField<uint8_t> as PacketLinkQuality;

		interface LocalTime<TRadio>;
	}

	uses
	{
		interface BusyWait<TMicro, uint16_t>;

		interface RFA1DriverConfig as Config;

		interface PacketFlag as TransmitPowerFlag;
		interface PacketFlag as RSSIFlag;
		interface PacketFlag as TimeSyncFlag;

		interface PacketTimeStamp<TRadio, uint32_t>;

		interface Tasklet;
#ifdef RADIO_DEBUG
		interface DiagMsg;
#endif
	}
}

implementation
{
	rfa1_header_t* getHeader(message_t* msg)
	{
		return ((void*)msg) + call Config.headerLength(msg);
	}

	void* getPayload(message_t* msg)
	{
		return ((void*)msg) + call RadioPacket.headerLength(msg);
	}

	rfa1_metadata_t* getMeta(message_t* msg)
	{
		return ((void*)msg) + sizeof(message_t) - call RadioPacket.metadataLength(msg);
	}

/*----------------- STATE -----------------*/

	tasklet_norace uint8_t state;
	enum
	{
		STATE_PLL_ON = 0,
		STATE_SLEEP = 1,
		STATE_SLEEP_2_TRX_OFF = 2,
		STATE_TRX_OFF = 3,
		STATE_TRX_OFF_2_RX_ON = 4,
		STATE_RX_ON = 5,
		STATE_BUSY_TX_2_RX_ON = 6,
		STATE_PLL_ON_2_RX_ON = 7,
	};

	tasklet_norace uint8_t cmd;
	enum
	{
		CMD_NONE = 0,			// the state machine has stopped
		CMD_TURNOFF = 1,		// goto SLEEP state
		CMD_STANDBY = 2,		// goto TRX_OFF state
		CMD_TURNON = 3,			// goto RX_ON state
		CMD_TRANSMIT = 4,		// currently transmitting a message
		CMD_RECEIVE = 5,		// currently receiving a message
		CMD_CCA = 6,			// performing clear channel assessment
		CMD_CHANNEL = 7,		// changing the channel
		CMD_SIGNAL_DONE = 8,	// signal the end of the state transition
		CMD_DOWNLOAD = 9,		// download the received message
	};
	

	norace uint8_t radioIrq;

	tasklet_norace uint8_t txPower;
	tasklet_norace uint8_t channel;

	tasklet_norace message_t* rxMsg;
	message_t rxMsgBuffer;

	tasklet_norace uint8_t rssiClear;
	tasklet_norace uint8_t rssiBusy;


/*----------------- ALARM -----------------*/

	enum
	{
		// TODO: needs to be calibrated
		TX_SFD_DELAY = 0,
		RX_SFD_DELAY = 0,
	};


/*----------------- INIT -----------------*/

	command error_t PlatformInit.init()
	{

		rxMsg = &rxMsgBuffer;

		// these are just good approximates
		rssiClear = 0;
		rssiBusy = 90;

		return SUCCESS;
	}

	command error_t SoftwareInit.init()
	{

		// reset transceiver section
		SET_BIT(TRXPR, TRXRST);

		IRQ_MASK = (1<<PLL_LOCK_EN) | (1<<TX_END_EN) | (1<<RX_START_EN) | (1<<RX_END_EN) | (1<<AWAKE_EN);
		CCA_THRES =  RFA1_CCA_THRES_VALUE;
		PHY_TX_PWR = (RFA1_DEF_RFPOWER & RFA1_TX_PWR_MASK) | (PA_BUF_LT_6US<<PA_BUF_LT0) | (PA_LT_2US<<PA_LT0);

		txPower = RFA1_DEF_RFPOWER & RFA1_TX_PWR_MASK;
		channel = RFA1_DEF_CHANNEL & RFA1_CHANNEL_MASK;
		
		PHY_CC_CCA = (RFA1_CCA_MODE_VALUE<<CCA_MODE0) | channel;

		// enable MAC layer timestamping with 32khz RTC
		SCCR0 = 1<<SCEN | 1<<SCTSE | 1<<SCCKSEL;

		// enter sleep mode
		SET_BIT(TRXPR, SLPTR);
	    state = STATE_SLEEP;

		return SUCCESS;
	}


/*----------------- CHANNEL -----------------*/

	tasklet_async command uint8_t RadioState.getChannel()
	{
		return channel;
	}

	tasklet_async command error_t RadioState.setChannel(uint8_t c)
	{
		c &= RFA1_CHANNEL_MASK;

		if( cmd != CMD_NONE )
			return EBUSY;
		else if( channel == c )
			return EALREADY;

		channel = c;
		cmd = CMD_CHANNEL;
		call Tasklet.schedule();

		return SUCCESS;
	}

	inline void changeChannel()
	{
		ASSERT( cmd == CMD_CHANNEL );
		ASSERT( state == STATE_SLEEP || state == STATE_TRX_OFF || state == STATE_RX_ON );

		if (state==STATE_RX_ON || state==STATE_PLL_ON) {
			PHY_CC_CCA = (RFA1_CCA_MODE_VALUE<<CCA_MODE0) | channel;	
			state = STATE_TRX_OFF_2_RX_ON;
		}
		
	}

/*----------------- TURN ON/OFF -----------------*/

	inline void changeState()
	{
		
		if( (cmd == CMD_STANDBY || cmd == CMD_TURNON)
			&& state == STATE_SLEEP)
		{
			CLR_BIT(TRXPR, SLPTR);
			state = STATE_SLEEP_2_TRX_OFF;
			
		}
		else if( cmd == CMD_TURNON && state == STATE_TRX_OFF)
		{
			ASSERT( !radioIrq );
			
			// setChannel was ignored in SLEEP so do it here
            PHY_CC_CCA = (RFA1_CCA_MODE_VALUE<<CCA_MODE0) | channel;
			TRX_STATE = CMD_RX_ON;
			state = STATE_TRX_OFF_2_RX_ON;

		}
		else if( (cmd == CMD_TURNOFF || cmd == CMD_STANDBY) 
			&& state == STATE_RX_ON )
		{
			TRX_STATE = CMD_TRX_OFF;
			radioIrq = 0;

			state = STATE_TRX_OFF;
		}

		if( cmd == CMD_TURNOFF && state == STATE_TRX_OFF )
		{
			SET_BIT(TRXPR, SLPTR);
			state = STATE_SLEEP;
			cmd = CMD_SIGNAL_DONE;
		}
		else if( cmd == CMD_STANDBY && state == STATE_TRX_OFF )
			cmd = CMD_SIGNAL_DONE;
	}

	tasklet_async command error_t RadioState.turnOff()
	{

		if( cmd != CMD_NONE)
			return EBUSY;
		else if( state == STATE_TRX_OFF )
			return EALREADY;

		cmd = CMD_TURNOFF;
		
		call Tasklet.schedule();
		
		return SUCCESS;

 	}
	
	tasklet_async command error_t RadioState.standby()
	{
		if( cmd != CMD_NONE || (state == STATE_SLEEP) )
			return EBUSY;
		else if( state == STATE_TRX_OFF )
			return EALREADY;

		cmd = CMD_STANDBY;
		call Tasklet.schedule();

		return SUCCESS;
	}

	tasklet_async command error_t RadioState.turnOn()
	{
		if( cmd != CMD_NONE)
			return EBUSY;
		else if( state == STATE_RX_ON )
			return EALREADY;

		cmd = CMD_TURNON;
		call Tasklet.schedule();

		return SUCCESS;
	}

	default tasklet_async event void RadioState.done() { }

/*----------------- TRANSMIT -----------------*/

	tasklet_async command error_t RadioSend.send(message_t* msg)
	{

		uint32_t time;
		uint8_t length;
		uint8_t* data;
		uint8_t header;
		void* timesync;
		

		if( cmd != CMD_NONE || state != STATE_RX_ON || radioIrq )
			return EBUSY;

		length = (call PacketTransmitPower.isSet(msg) ?
			call PacketTransmitPower.get(msg) : RFA1_DEF_RFPOWER);

		if( length != txPower )
		{
			txPower = length;
			PHY_TX_PWR = (txPower & RFA1_TX_PWR_MASK) | (PA_BUF_LT_6US<<PA_BUF_LT0) | (PA_LT_2US<<PA_LT0);
		}

		if( call Config.requiresRssiCca(msg) 
				&& (PHY_RSSI & RFA1_RSSI_MASK) > ((rssiClear + rssiBusy) >> 3) )
			return EBUSY;

		TRX_STATE = CMD_PLL_ON;

		// do something useful, just to wait a little
		timesync = call PacketTimeSyncOffset.isSet(msg) ? ((void*)msg) + call PacketTimeSyncOffset.get(msg) : 0;

		// we have missed an incoming message in this short amount of time
		if( (TRX_STATUS & RFA1_TRX_STATUS_MASK) != RFA1_PLL_ON )
		{
			ASSERT( (TRX_STATUS & RFA1_TRX_STATUS_MASK) == RFA1_BUSY_RX );

			state = STATE_PLL_ON_2_RX_ON;
			return EBUSY;
		}


		ASSERT( ! radioIrq );

		data = getPayload(msg);
		length = getHeader(msg)->length;
		
		// length | data[0] ... data[length-3] | automatically generated FCS
		TRXFBST = length;
		

		// the FCS is automatically generated (2 bytes)
		length -= 2;

		header = call Config.headerPreloadLength();
		if( header > length )
			header = length;

		length -= header;
				
		// upload header
		memcpy((void*)(&TRXFBST+1), data, header);
		
		atomic
		{
			uint8_t lsb;
			
			TRX_STATE = CMD_TX_START;
			// get timestamp
			// reading the LSB captures current symbol counter value
			lsb = SCCNTLL;
			time = SCCNTHH;
			time = time<<8 | SCCNTHL;
			time = time<<8 | SCCNTLH;
			time = time<<8 | lsb;
			time = time + TX_SFD_DELAY;
		}


		if( timesync != 0 )
			*(timesync_relative_t*)timesync = (*(timesync_absolute_t*)timesync) - time;

		// copy payload
		memcpy((void*)(&TRXFBST+1+header), data + header, length);

		/*
		 * There is a very small window (~1 microsecond) when the RFA1 went 
		 * into PLL_ON state but was somehow not properly initialized because 
		 * of an incoming message and could not go into BUSY_TX. I think the
		 * radio can even receive a message, and generate a TRX_UR interrupt
		 * because of concurrent access, but that message probably cannot be
		 * recovered.
		 *
		 * TODO: this needs to be verified, and make sure that the chip is 
		 * not locked up in this case.
		 */

		// go back to RX_ON state when finished
		// wait for the TRX_END interrupt
		
		state = STATE_BUSY_TX_2_RX_ON;
		cmd = CMD_TRANSMIT;
		
		TRX_STATE = CMD_RX_ON;
		
		if( timesync != 0 )
			*(timesync_absolute_t*)timesync = time;

		call PacketTimeStamp.set(msg, time);

#ifdef RADIO_DEBUG_MESSAGES
		if( call DiagMsg.record() )
		{
			length = getHeader(msg)->length;

			call DiagMsg.chr('t');
			call DiagMsg.uint32(call PacketTimeStamp.isValid(rxMsg) ? call PacketTimeStamp.timestamp(rxMsg) : 0);
			call DiagMsg.uint16(call RadioAlarm.getNow());
			call DiagMsg.int8(length);
			call DiagMsg.hex8s(getPayload(msg), length - 2);
			call DiagMsg.send();
		}
#endif

		return SUCCESS;
	}

	default tasklet_async event void RadioSend.sendDone(error_t error) { }
	default tasklet_async event void RadioSend.ready() { }

/*----------------- CCA -----------------*/

	tasklet_async command error_t RadioCCA.request()
	{
		if( cmd != CMD_NONE || state != STATE_RX_ON)
			return EBUSY;

		// TODO: see Errata 38.5.5 of the datasheet
		cmd = CMD_CCA;
		PHY_CC_CCA = 1<<CCA_REQUEST | (RFA1_CCA_MODE_VALUE<<CCA_MODE0) | channel;

		return SUCCESS;
	}

	default tasklet_async event void RadioCCA.done(error_t error) { }

/*----------------- RECEIVE -----------------*/

	inline void downloadMessage()
	{
		uint8_t length;
		bool signalReceive = TRUE;

		// read the length byte
		length = TST_RX_LENGTH;

				
		// if correct length
		if( length >= 3 && length <= call RadioPacket.maxPayloadLength() + 2 )
		{
			uint8_t read;
			uint8_t* data;

			data = getPayload(rxMsg);
			getHeader(rxMsg)->length = length;
			
			// we do not store the CRC field
			length -= 2;

			read = call Config.headerPreloadLength();
			if( length < read )
				read = length;

			length -= read;

	        memcpy(data, (void*)(&TRXFBST), read);
	        data += read;

			if( signal RadioReceive.header(rxMsg) )
			{
				
				memcpy(data, (void*)(&TRXFBST+read), length);
        
        		call PacketLinkQuality.set(rxMsg, (uint16_t)(&TRXFBST+TST_RX_LENGTH));
			}
			else
				signalReceive = FALSE;
		}
		else
			signalReceive = FALSE;

		// release frame buffer
		CLR_BIT(TRX_CTRL_2, RX_SAFE_MODE);
		// and protect it again
		SET_BIT(TRX_CTRL_2, RX_SAFE_MODE);

		state = STATE_RX_ON;

#ifdef RADIO_DEBUG_MESSAGES
		if( call DiagMsg.record() )
		{
			length = getHeader(rxMsg)->length;

			call DiagMsg.chr('r');
			call DiagMsg.uint32(call PacketTimeStamp.isValid(rxMsg) ? call PacketTimeStamp.timestamp(rxMsg) : 0);
			call DiagMsg.uint16(call RadioAlarm.getNow());
			call DiagMsg.int8(crc == 0 ? length : -length);
			call DiagMsg.hex8s(getPayload(rxMsg), length - 2);
			call DiagMsg.int8(call PacketRSSI.isSet(rxMsg) ? call PacketRSSI.get(rxMsg) : -1);
			call DiagMsg.uint8(call PacketLinkQuality.isSet(rxMsg) ? call PacketLinkQuality.get(rxMsg) : 0);
			call DiagMsg.send();
		}
#endif
		
		cmd = CMD_NONE;

		// signal only if it has passed the CRC check
		if (signalReceive)
			rxMsg = signal RadioReceive.receive(rxMsg);
	}





	void serviceRadio()
	{

			uint8_t irq;
			uint8_t temp;
			
			atomic {
			  irq = radioIrq;
			  // TODO: this will reset all other pending interrupts
			  radioIrq = 0;
			}
			
#ifdef RFA1_RSSI_ENERGY
			if( irq & RFA1_IRQ_TRX_END )
			{
				if( irq == RFA1_IRQ_TRX_END || 
					(irq == (RFA1_IRQ_RX_START | RFA1_IRQ_TRX_END) && cmd == CMD_NONE) )
					call PacketRSSI.set(rxMsg, PHY_ED_LEVEL);
				else
					call PacketRSSI.clear(rxMsg);
			}
#endif

			if( irq & RFA1_IRQ_PLL_LOCK )
			{
				if( cmd == CMD_TURNON || cmd == CMD_CHANNEL )
				{
					ASSERT( state == STATE_TRX_OFF_2_RX_ON );

					state = STATE_RX_ON;
					// enable frame buffer protection
					SET_BIT(TRX_CTRL_2, RX_SAFE_MODE);
					cmd = CMD_SIGNAL_DONE;
				}
				else if( cmd == CMD_TRANSMIT )
				{
					ASSERT( state == STATE_BUSY_TX_2_RX_ON );
				}
				else
					ASSERT(FALSE);
			}

			if( irq & RFA1_IRQ_RX_START )
			{
				if( cmd == CMD_CCA )
				{
					signal RadioCCA.done(FAIL);
					cmd = CMD_NONE;
				}

				if( cmd == CMD_NONE )
				{
					ASSERT( state == STATE_RX_ON || state == STATE_PLL_ON_2_RX_ON );

					// the most likely place for busy channel, with no TRX_END interrupt
					if( irq == RFA1_IRQ_RX_START )
					{
						temp = PHY_RSSI & RFA1_RSSI_MASK;
						rssiBusy += temp - (rssiBusy >> 2);
#ifndef RFA1_RSSI_ENERGY
						call PacketRSSI.set(rxMsg, temp);
					}
					else
					{
						call PacketRSSI.clear(rxMsg);
#endif
					}

					/*
					 * The timestamp corresponds to the first event which could not
					 * have been a PLL_LOCK because then cmd != CMD_NONE, so we must
					 * have received a message (and could also have received the 
					 * TRX_END interrupt in the mean time, but that is fine. Also,
					 * we could not be after a transmission, because then cmd = 
					 * CMD_TRANSMIT.
					 */
					if( irq == RFA1_IRQ_RX_START ) // just to be cautious
					{
						
						uint32_t time =  SCTSRHH;
						time = time<<8 | SCTSRHL;
						time = time<<8 | SCTSRLH;
						time = time<<8 | SCTSRLL;
						
						//printf("Time: %lu\n", time);
						call PacketTimeStamp.set(rxMsg, time - RX_SFD_DELAY);
					}
					else {
						call PacketTimeStamp.clear(rxMsg);
					}

					cmd = CMD_RECEIVE;
				}
				else
					ASSERT( cmd == CMD_TURNOFF );
			}

			if( irq & RFA1_IRQ_TRX_END )
			{
				
				if( cmd == CMD_TRANSMIT )
				{
					ASSERT( state == STATE_BUSY_TX_2_RX_ON );

					state = STATE_RX_ON;
					cmd = CMD_NONE;
					
					signal RadioSend.sendDone(SUCCESS);

					// TODO: we could have missed a received message
					ASSERT( ! (irq & RFA1_IRQ_RX_START) );
				}
				else if( cmd == CMD_RECEIVE )
				{
					ASSERT( state == STATE_RX_ON || state == STATE_PLL_ON_2_RX_ON );

					if( state == STATE_PLL_ON_2_RX_ON )
					{
						ASSERT( (TRX_STATUS & RFA1_TRX_STATUS_MASK) == RFA1_PLL_ON );

						TRX_STATE = CMD_RX_ON;
						state = STATE_RX_ON;
					}
					else
					{
						// the most likely place for clear channel (hope to avoid acks)
						rssiClear += (PHY_RSSI & RFA1_RSSI_MASK) - (rssiClear >> 2);
					}

					cmd = CMD_DOWNLOAD;
				}
				else
					ASSERT(FALSE);
			}
	}

	default tasklet_async event bool RadioReceive.header(message_t* msg)
	{
		return TRUE;
	}

	default tasklet_async event message_t* RadioReceive.receive(message_t* msg)
	{
		return msg;
	}

/*----------------- TASKLET -----------------*/


	tasklet_async event void Tasklet.run()
	{
		//printf("Tasklet run: cmd = %u, state = %u\n", cmd, state);
		
		if( radioIrq )
			serviceRadio();

		if( cmd != CMD_NONE )
		{
			if( cmd == CMD_DOWNLOAD )
				downloadMessage();
			else if( CMD_TURNOFF <= cmd && cmd <= CMD_TURNON )
				changeState();
			else if( cmd == CMD_CHANNEL )
				changeChannel();
			else if( cmd == CMD_CCA ) {
				cmd = CMD_NONE;
				ASSERT( (TRX_STATUS & RFA1_TRX_STATUS_MASK) == RX_ON );
				signal RadioCCA.done((TRX_STATUS & RFA1_CCA_STATUS) ? SUCCESS : EBUSY);
			}
			
			if( cmd == CMD_SIGNAL_DONE )
			{
				cmd = CMD_NONE;
				signal RadioState.done();
			}
		}

		if( cmd == CMD_NONE && state == STATE_RX_ON && ! radioIrq )
			signal RadioSend.ready();
	}

/*----------------- RadioPacket -----------------*/
	
	async command uint8_t RadioPacket.headerLength(message_t* msg)
	{
		return call Config.headerLength(msg) + sizeof(rfa1_header_t);
	}

	async command uint8_t RadioPacket.payloadLength(message_t* msg)
	{
		return getHeader(msg)->length - 2;
	}

	async command void RadioPacket.setPayloadLength(message_t* msg, uint8_t length)
	{
		ASSERT( 1 <= length && length <= 125 );
		ASSERT( call RadioPacket.headerLength(msg) + length + call RadioPacket.metadataLength(msg) <= sizeof(message_t) );

		// we add the length of the CRC, which is automatically generated
		getHeader(msg)->length = length + 2;
	}

	async command uint8_t RadioPacket.maxPayloadLength()
	{
		ASSERT( call Config.maxPayloadLength() - sizeof(rfa1_header_t) <= 125 );

		return call Config.maxPayloadLength() - sizeof(rfa1_header_t);
	}

	async command uint8_t RadioPacket.metadataLength(message_t* msg)
	{
		return call Config.metadataLength(msg) + sizeof(rfa1_metadata_t);
	}

	async command void RadioPacket.clear(message_t* msg)
	{
		// all flags are automatically cleared
	}

/*----------------- PacketTransmitPower -----------------*/

	async command bool PacketTransmitPower.isSet(message_t* msg)
	{
		return call TransmitPowerFlag.get(msg);
	}

	async command uint8_t PacketTransmitPower.get(message_t* msg)
	{
		return getMeta(msg)->power;
	}

	async command void PacketTransmitPower.clear(message_t* msg)
	{
		call TransmitPowerFlag.clear(msg);
	}

	async command void PacketTransmitPower.set(message_t* msg, uint8_t value)
	{
		call TransmitPowerFlag.set(msg);
		getMeta(msg)->power = value;
	}

/*----------------- PacketRSSI -----------------*/

	async command bool PacketRSSI.isSet(message_t* msg)
	{
		return call RSSIFlag.get(msg);
	}

	async command uint8_t PacketRSSI.get(message_t* msg)
	{
		return getMeta(msg)->rssi;
	}

	async command void PacketRSSI.clear(message_t* msg)
	{
		call RSSIFlag.clear(msg);
	}

	async command void PacketRSSI.set(message_t* msg, uint8_t value)
	{
		// just to be safe if the user fails to clear the packet
		call TransmitPowerFlag.clear(msg);

		call RSSIFlag.set(msg);
		getMeta(msg)->rssi = value;
	}

/*----------------- PacketTimeSyncOffset -----------------*/

	async command bool PacketTimeSyncOffset.isSet(message_t* msg)
	{
		return call TimeSyncFlag.get(msg);
	}

	async command uint8_t PacketTimeSyncOffset.get(message_t* msg)
	{
		return call RadioPacket.headerLength(msg) + call RadioPacket.payloadLength(msg) - sizeof(timesync_absolute_t);
	}

	async command void PacketTimeSyncOffset.clear(message_t* msg)
	{
		call TimeSyncFlag.clear(msg);
	}

	async command void PacketTimeSyncOffset.set(message_t* msg, uint8_t value)
	{
		// we do not store the value, the time sync field is always the last 4 bytes
		ASSERT( call PacketTimeSyncOffset.get(msg) == value );

		call TimeSyncFlag.set(msg);
	}

/*----------------- PacketLinkQuality -----------------*/

	async command bool PacketLinkQuality.isSet(message_t* msg)
	{
		return TRUE;
	}

	async command uint8_t PacketLinkQuality.get(message_t* msg)
	{
		return getMeta(msg)->lqi;
	}

	async command void PacketLinkQuality.clear(message_t* msg)
	{
	}

	async command void PacketLinkQuality.set(message_t* msg, uint8_t value)
	{
		getMeta(msg)->lqi = value;
	}


/*----------------- LocalTime -----------------*/

	async command uint32_t LocalTime.get() {
		
		uint32_t time;		
		atomic {
			// reading the LSB captures the current symbol counter
			uint8_t lsb = SCCNTLL;
		    time = SCCNTHH;
		    time = time<<8 | SCCNTHL;
		    time = time<<8 | SCCNTLH;
		    time = time<<8 | lsb;
		}
		return time;
	}
	
	/*----------------- INTERRUPTS -----------------*/


	AVR_ATOMIC_HANDLER(TRX24_RX_START_vect) {
		
		ASSERT( ! radioIrq );

		radioIrq |= RFA1_IRQ_RX_START;
		call Tasklet.schedule();
	}

	AVR_ATOMIC_HANDLER(TRX24_RX_END_vect) {

		ASSERT( ! radioIrq );
		radioIrq |= RFA1_IRQ_TRX_END;
		call Tasklet.schedule();
	}

	AVR_ATOMIC_HANDLER(TRX24_TX_END_vect) {
		ASSERT( ! radioIrq );
		radioIrq |= RFA1_IRQ_TRX_END;
		call Tasklet.schedule();
	}

	AVR_ATOMIC_HANDLER(TRX24_PLL_LOCK_vect) {

		if (state == STATE_TRX_OFF_2_RX_ON) {
			// state change to PLL_ON/RX_ON
			state = STATE_RX_ON;
			cmd = CMD_SIGNAL_DONE;
		    call Tasklet.schedule();
		}
		
	}

	AVR_ATOMIC_HANDLER(TRX24_AWAKE_vect) {
		// TRX_OFF state reached
		if (state==STATE_SLEEP_2_TRX_OFF) {
		  	state = STATE_TRX_OFF;
			call Tasklet.schedule();
		}
	}


	AVR_ATOMIC_HANDLER(TRX24_CCA_ED_DONE_vect) {
		// CCA completed 
		// TODO: check for Errata 38.5.5
		if( cmd == CMD_CCA && (TRX_STATUS & RFA1_CCA_DONE))
		{
			call Tasklet.schedule();
		}		


	}
	
	
}
