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

module HplAtmRfa1Timer1P @safe()
{
	provides
	{
		interface AtmegaCounter<uint16_t> as Timer;
		interface AtmegaCompare<uint16_t> as CompareA;
//		interface AtmegaCompare<uint16_t> as CompareB;
//		interface AtmegaCompare<uint16_t> as CompareC;
		interface McuPowerOverride;
	}

	uses
	{
		interface McuPowerState;
	}
}

implementation
{
// ----- TIMER: timer counter register (TCNT)

	async command uint16_t Timer.get()
	{
		atomic return TCNT1;
	}

	async command void Timer.set(uint16_t value)
	{
		atomic TCNT1 = value;
	}

// ----- TIMER: timer interrupt flag register (TIFR), timer overflow flag (TOV)

	default async event void Timer.overflow() { }

	AVR_ATOMIC_HANDLER(TIMER1_OVF_vect) { signal Timer.overflow(); }

	async command bool Timer.test() { return TIFR1 & (1 << TOV1); }

	async command void Timer.reset() { TIFR1 = 1 << TOV1; }

// ----- TIMER: timer interrupt mask register (TIMSK), timer overflow interrupt enable (TOIE)

	async command void Timer.start()
	{
		SET_BIT(TIMSK1, TOIE1);
	}

	async command void Timer.stop()
	{
		CLR_BIT(TIMSK1, TOIE1);
	}

	async command bool Timer.isOn() { return TIMSK1 & (1 << TOIE1); }

// ----- TIMER: timer control register (TCCR), clock select (CS) and waveform generation mode (WGM) bits

	async command void Timer.setMode(uint8_t mode)
	{
		atomic
		{
			TCCR1A = (TCCR1A & ~(0x3 << WGM10))
				| ((mode >> 3) & 0x3) << WGM10;

			TCCR1B = (TCCR1B & ~(0x3 << WGM12 | 0x7 << CS10))
				| ((mode >> 5) & 0x3) << WGM12
				| ((mode >> 0) & 0x7) << CS10;
		}
	}

	async command uint8_t Timer.getMode()
	{
		uint8_t a, b;

		atomic
		{
			a = TCCR1A;
			b = TCCR1B;
		}

		return ((a >> WGM10) & 0x3) << 3
			| ((b >> WGM12) & 0x3) << 5
			| ((b >> CS10) & 0x7) << 0;
	}

// ----- COMPAREA: output compare register (OCR)

	async command uint16_t CompareA.get()
	{
		atomic return OCR1A;
	}

	async command void CompareA.set(uint16_t value)
	{
		atomic OCR1A = value;
	}

// ----- COMPAREA: timer interrupt flag register (TIFR), output comare match flag (OCF)

	default async event void CompareA.fired() { }

	AVR_ATOMIC_HANDLER(TIMER1_COMPA_vect) { signal CompareA.fired(); }

	async command bool CompareA.test() { return TIFR1 & (1 << OCF1A); }

	async command void CompareA.reset() { TIFR1 = 1 << OCF1A; }

// ----- COMPAREA: timer interrupt mask register (TIMSK), output compare interrupt enable (OCIE)

	async command void CompareA.start()
	{
		SET_BIT(TIMSK1, OCIE1A);

		call McuPowerState.update();
	}

	async command void CompareA.stop()
	{
		CLR_BIT(TIMSK1, OCIE1A);

		call McuPowerState.update();
	}

	async command bool CompareA.isOn() { return TIMSK1 & (1 << OCIE1A); }

// ----- COMPAREA: timer control register (TCCR), compare output mode (COM)

	async command void CompareA.setMode(uint8_t mode)
	{
		atomic
		{
			TCCR1A = (TCCR1A & ~(0x3 << COM1A0))
				| (mode & 0x3) << COM1A0;
		}
	}

	async command uint8_t CompareA.getMode()
	{
		return (TCCR1A >> COM1A0) & 0x3;
	}

// ----- COMPAREA: timer control register (TCCR), force output compare (FOC)

	async command void CompareA.force()
	{
		SET_BIT(TCCR1B, FOC1A);
	}

// ----- MCUPOWER

	async command mcu_power_t McuPowerOverride.lowestState()
	{
		// if we need to wake up by this clock
		if( TIMSK1 & (1 << TOIE1 | 1 << OCIE1A | 1 << OCIE1B) )
			return ATM128_POWER_SAVE;
		else
			return ATM128_POWER_DOWN;
	}
}
