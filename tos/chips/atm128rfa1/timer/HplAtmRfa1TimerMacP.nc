/*
 * Copyright (c) 2010, University of Szeged
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

#include "HplAtmRfa1Timer.h"

module HplAtmRfa1TimerMacP @safe()
{
	provides
	{
		interface AtmegaCounter<uint32_t> as Counter;
		interface AtmegaCompare<uint32_t> as CompareA;
		interface McuPowerOverride;
	}

	uses
	{
		interface McuPowerState;
	}
}

implementation
{
	typedef union reg32_t
	{
		uint32_t full;
		struct 
		{
			uint8_t ll;
			uint8_t lh;
			uint8_t hl;
			uint8_t hh;
		};
	} reg32_t;

// ----- COUNTER: symbol counter register (SCCNT)

	async command uint32_t Counter.get()
	{
		reg32_t time;

		atomic
		{
			time.ll = SCCNTLL;
			time.lh = SCCNTLH;
			time.hl = SCCNTHL;
			time.hh	= SCCNTHH;
		}

		return time.full;
	}

	async command void Counter.set(uint32_t value)
	{
		reg32_t time;
		
		time.full = value;

		atomic
		{
			SCCNTHH = time.hh;
			SCCNTHL = time.hl;
			SCCNTLH = time.lh;
			SCCNTLL = time.ll;
		}

		while( SCSR & (1 << SCBSY) )
			;
	}

// ----- COUNTER: symbol counter interrupt status register (SCIRQS), overflow flag (IRQSOF)

	default async event void Counter.overflow() { }

	AVR_ATOMIC_HANDLER(SCNT_OVFL_vect) { signal Counter.overflow(); }

	async command bool Counter.test() { return SCIRQS & (1 << IRQSOF); }

	async command void Counter.reset() { SCIRQS = 1 << IRQSOF; }

// ----- COUNTER: symbol counter interrupt mask register (SCIRQM), overflow interrupt enable (IRQMOF)

	async command void Counter.start()
	{
		SET_BIT(SCIRQM, IRQMOF);
	}

	async command void Counter.stop()
	{
		CLR_BIT(SCIRQM, IRQMOF);
	}

	async command bool Counter.isOn() { return SCIRQM & (1 << IRQMOF); }

// ----- COUNTER: symbol counter control register (SCCR), counter enable (SCEN) and clock select (SCCKSEL)

	async command void Counter.setMode(uint8_t mode)
	{
		mode = ((mode & ATMRFA1_SCCK_ENABLE) ? (1 << SCEN) : 0)
			| ((mode & ATMRFA1_SCCK_RTC) ? (1 << SCCKSEL) : 0);

		atomic SCCR0 = (SCCR0 & ~((1 << SCEN) | (1 << SCCKSEL))) | mode;

		call McuPowerState.update();
	}

	async command uint8_t Counter.getMode()
	{
		return SCCR0 & ((1 << SCEN) | (1 << SCCKSEL));
	}


// ----- COMPARE A: symbol counter output compare register (SCOCR)

	async command uint32_t CompareA.get()
	{
		reg32_t time;

		atomic
		{
			time.ll = SCOCR1LL;
			time.lh = SCOCR1LH;
			time.hl = SCOCR1HL;
			time.hh	= SCOCR1HH;
		}

		return time.full;
	}

	async command void CompareA.set(uint32_t value)
	{
		reg32_t time;
		
		time.full = value;

		atomic
		{
			SCOCR1HH = time.hh;
			SCOCR1HL = time.hl;
			SCOCR1LH = time.lh;
			SCOCR1LL = time.ll;
		}
	}

// ----- COMPARE A: symbol counter interrupt status register (SCIRQS), comare match flag (IRQSCP)

	default async event void CompareA.fired() { }

	AVR_ATOMIC_HANDLER(SCNT_CMP1_vect) { signal CompareA.fired(); }

	async command bool CompareA.test() { return SCIRQS & (1 << IRQSCP1); }

	async command void CompareA.reset() { SCIRQS = 1 << IRQSCP1; }

// ----- COMPARE A: symbol counter interrupt mask register (SCIRQM), compare interrupt enable (IRQMCP)

	async command void CompareA.start()
	{
		SET_BIT(SCIRQM, IRQMCP1);

		call McuPowerState.update();
	}

	async command void CompareA.stop()
	{
		CLR_BIT(SCIRQM, IRQMCP1);

		call McuPowerState.update();
	}

	async command bool CompareA.isOn() { return SCIRQM & (1 << IRQMCP1); }

// ----- COMPARE A: symbol counter control register (SCCR), compare mode (SCCMP)

	async command void CompareA.setMode(uint8_t mode)
	{
		atomic
		{
			SCCR0 = (SCCR0 & ~(1 << SCCMP1)) 
				| (mode & 0x1) << SCCMP1;
		}
	}

	async command uint8_t CompareA.getMode()
	{
		return (SCCR0 >> SCCMP1) & 0x1;
	}

// ----- COMPARE A: ignore force for the symbol counter

	async command void CompareA.force()
	{
	}

// ----- MCUPOWER

	async command mcu_power_t McuPowerOverride.lowestState()
	{
		// TODO: go down to ATM128_POWER_DOWN and think about DEEP SLEEP
		return ATM128_POWER_SAVE;
	}
}
