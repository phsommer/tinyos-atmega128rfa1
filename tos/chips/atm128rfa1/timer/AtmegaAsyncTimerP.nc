/*
 * Copyright (c) 2010-2011, University of Szeged
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
 */

generic module AtmegaAsyncTimerP(typedef precision_tag, uint8_t mode, uint8_t mindt)
{
	provides
	{
		interface Init @exactlyonce();
		interface Counter<precision_tag, uint16_t>;
		interface Alarm<precision_tag, uint16_t>;
	}

	uses
	{
		interface AtmegaCounter<uint8_t>;
		interface AtmegaCompare<uint8_t>;
	}
}

implementation
{
	command error_t Init.init()
	{
		call AtmegaCounter.setMode(mode);
		// we do not enable the overflow interrupt, see below

		call AtmegaCompare.setMode(0);	// normal mode
		call AtmegaCompare.start();

		return SUCCESS;
	}

	typedef union time_t
	{
		uint16_t full;
		struct 
		{
			uint8_t low;
			uint8_t high;
		};
	} time_t;

	struct flags_t
	{
		unsigned int suspendOverflow : 1;
		unsigned int compareTrampoline : 1;
		unsigned int alarmRunning : 1; 
	} flags;

/**
 * The async counter overflow flag is set one 32KHz tick after it has changed 
 * from 0xFF to 0x00. So if the prescaler is disabled, then we have two
 * posible sequence of events for the counter and the overflow flag
 *
 *	CNT=0, OVF=0, CNT=0, OVF=1, CNT=1
 *	CNT=0, OVF=0, CNT=1, OVF=1, CNT=1
 *
 * However, if the prescaler is enabled (divide 8 or more), then the overflow
 * interrupt comes while the counter is still 0 (because of the prescaler it 
 * will advance to 1 much later). So we have only one sequence of events in 
 * this case:
 *
 *	CNT=0, OVF=0, CNT=0, OVF=1, CNT=0
 *
 * In this case the overflow interrupt is usually executed when the counter is
 * still zero, so from (CNT=0, TST=0) we cannot decide if we are before the 
 * interrupt or after. In this case we can busy wait for 1/32768 sec to be 
 * deffinitely after the interrupt, but that is inefficient.
 *
 * The output compare register works differently. It is always signaled one
 * prescalled clock tick after the counter becomes equal to the compare 
 * register, regardless of the prescaler. This way the compare flag is always 
 * set on the clock tick boundary. If the compare register is 00, then we
 * have two possible sequences:
 *
 *	CNT=0, CMP=0, CNT=0, CMP=1, CNT=1
 *	CNT=0, CMP=0, CNT=1, CMP=1, CNT=1
 *
 * and it works the same way for all other compare register values. This 
 * makes the compare interrupt predictable.
 *
 * Therefore, we do not use the overflow interrupt (and never enable it) to 
 * cut the number of interrupts into half and to minimize the amount of time 
 * the MCU is awake.
 */

	async event void AtmegaCounter.overflow()
	{
	}

/*
 * The low 8-bit of the current time is always AtmegaCounter.get()-1. The
 * higher bits are advanced when a compare match is signalled, so there is
 * a pending interrupt, then we have to increment it here as well. We have
 * to read AtmegaCounter.get before AtmegaCompare.test, otherwise we cannot
 * distingush two cases (interupt before the two calls, or the interrupt is
 * already serviced). There is still a race, if the interrupt happens between
 * the AtmegaCounter.get and AtmegaCompate.test call, but this can be 
 * separated. The other case when we have to increment the higher bits is
 * when AtmegaCounter.get() is lower than AtmegaCompare.get() + 1 (the 
 * interrupt comes one tick after the match). Both events can happen
 * simultaneously, e.g. when an interrupt is delayed by more than 2 ticks,
 * AtmegaCompare.get() = 0xFE, AtmegaCounter.get() == 0x00, and
 * AtmegaCompare.test() == TRUE.
 */

	uint8_t currentHigh;
	uint8_t alarmHigh;

	async command uint16_t Counter.get()
	{
		time_t time;
		bool test;
		uint8_t comp;

		atomic
		{
			time.low = call AtmegaCounter.get();
			test = call AtmegaCompare.test();
			comp = call AtmegaCompare.get();
			time.high = currentHigh;
		}

		// pending interrupt before time.low was read
		if( test && time.low != comp )
			time.high += 1;

		// interrupt comes one tick after the match
		time.low -= 1;	

		// overflowed
		if( time.low < comp )
			time.high += 1;

		return time.full;
	}

	async command bool Counter.isOverflowPending()
	{
		if( currentHigh < 0xFE || flags.suspendOverflow )
			return FALSE;

		return (int16_t)(call Counter.get()) >= 0;
	}

	async command void Counter.clearOverflow()
	{
		if( call Counter.isOverflowPending() )
			flags.suspendOverflow = 1;
	}

/**
 * We signal the counter.overflow event VERY late, when the next compare 
 * event is executed. This does not cause problem, since that event does not 
 * have to be very precise, however, we report the overflow pending flag 
 * correctly. 
 *
 * Setting the compare value cannot be close to the counter, so we do a 
 * trampoline, where we just advance the compare value twice by 0x80. We
 * do the second part here, if necessary.
 */

	// called in atomic context
	async event void AtmegaCompare.fired()
	{
		if( flags.compareTrampoline )
		{
			uint8_t comp = call AtmegaCompare.get();
			call AtmegaCompare.set(comp + 0x80);

			// if a new interrupt will come in the same cycle
			if( comp < 0x80 )
				return;
		}

		if( ++currentHigh == 0 )
		{
			if( flags.suspendOverflow )
				flags.suspendOverflow = 0;
			else
				signal Counter.overflow();
		}

		if( flags.compareTrampoline )
			flags.compareTrampoline = 0;
		else if( currentHigh == alarmHigh )
			;
	}

/**
 * The compare is always enabled, so we have to emulate enabling/
 * disabling the alarm. The lower 8-bit of the alarm is always the
 * compare register value.
 */

	async command uint16_t Alarm.getNow()
	{
		return call Counter.get();
	}

	async command bool Alarm.isRunning()
	{
		return flags.alarmRunning;
	}

	async command void Alarm.stop()
	{
		flags.alarmRunning = 0;
	}

	async command uint16_t Alarm.getAlarm()
	{
		time_t time;
		bool tramp;

		atomic
		{
			time.high = alarmHigh;
			time.low = call AtmegaCompare.get();
			tramp = flags.compareTrampoline;
		}

		if( tramp )
			time.low += 0x80;

		return time.full;
	}

/*
	// called in atomic context
	async event void AtmegaCompare.fired()
	{
		uint8_t comp;

		comp = call AtmegaCompare.get();
		comp += 17;

		call AtmegaCompare.set(comp);

		if( (uint8_t)(comp+1) < 17 )
		{
			if( ++currentHigh == 0 )
				signal Counter.overflow();
		}
	}
*/

	default async event void Alarm.fired() { }

	// callers make sure that time is always in the future
	void setAlarm(uint16_t time)
	{
		call AtmegaCompare.set(time);
		call AtmegaCompare.reset();
		call AtmegaCompare.start();
	}

	async command void Alarm.startAt(uint16_t nt0, uint16_t ndt)
	{
		atomic
		{
			// current time + time needed to set alarm
			uint16_t n = call Counter.get() + mindt;

			// if alarm is set in the future, where n - nt0 
			// is the time passed since nt0
			if( (uint16_t)(n - nt0) < ndt )
				n = nt0 + ndt;

			setAlarm(n);
		}
	}

	async command void Alarm.start(uint16_t ndt)
	{
		atomic
		{
			uint16_t n = call Counter.get();

			// calculate the next alarm
			n += (mindt > ndt) ? mindt : ndt;

			setAlarm(n);
		}
	}
}
