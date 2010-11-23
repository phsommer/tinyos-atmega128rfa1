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

configuration Counter62khz32C
{
	provides
	{
		interface Init;
		interface Counter<T62khz, uint32_t>;

		interface Alarm<T62khz, uint32_t> as Alarm[uint8_t id];
	}
}

implementation
{
	components McuSleepC;

	components HplAtmRfa1TimerMacP;
	HplAtmRfa1TimerMacP.McuPowerOverride <- McuSleepC;
	HplAtmRfa1TimerMacP.McuPowerState -> McuSleepC;

	components new AtmegaCounterP(T62khz, uint32_t, ATMRFA1_SCCK_RTC | ATMRFA1_SCCK_ENABLE);
	Init = AtmegaCounterP;
	Counter = AtmegaCounterP;
	AtmegaCounterP.AtmegaCounter -> HplAtmRfa1TimerMacP;

	components new AtmegaCompareP(T62khz, uint32_t, 0, 2);
	Alarm[0] = AtmegaCompareP;
	AtmegaCompareP.AtmegaCounter -> HplAtmRfa1TimerMacP;
	AtmegaCompareP.AtmegaCompare -> HplAtmRfa1TimerMacP.CompareA;
}
