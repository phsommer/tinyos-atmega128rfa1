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

#ifndef __TIMERCONFIG_H__
#define __TIMERCONFIG_H__

#include "HplAtmRfa1Timer.h"
#include "hardware.h"
#include "Timer.h"

// ------ MCU timer parameters ------

typedef struct T16mhz { } T16mhz;
typedef struct T8mhz { } T8mhz;
typedef struct T4mhz { } T4mhz;
typedef struct T2mhz { } T2mhz;


typedef T2mhz TMcu;

#define MCU_TIMER_MODE		(ATMRFA1_CLK16_DIVIDE_8 | ATMRFA1_WGM16_NORMAL)
#define MCU_TIMER_MHZ_LOG2	1
#define MCU_TIMER_MHZ		(1 << MCU_TIMER_MHZ_LOG2)
#define MCU_TIMER_HZ		(16000000ul / 16 * (1 << MCU_TIMER_MHZ_LOG2))

// selects which 16-bit TimerCounter should be used (1 or 3)
#define MCU_TIMER_NO		1

#define MCU_ALARM_MINDT		100
#define UQ_MCU_ALARM		"UQ_MCU_ALARM"

// ------ RTC timer parameters ------

#define RTC_TIMER_KHZ_LOG2	5

#if RTC_TIMER_KHZ_LOG2 == 5
typedef T32khz TRtc;
#define RTC_TIMER_MODE		(ATMRFA1_CLK8_NORMAL | ATMRFA1_WGM8_NORMAL | ATMRFA1_ASYNC_ON)

#elif RTC_TIMER_KHZ_LOG2 == 2
typedef struct T4khz { } T4khz;
typedef T4khz TRtc;
#define RTC_TIMER_MODE		(ATMRFA1_CLK8_DIVIDE_8 | ATMRFA1_WGM8_NORMAL | ATMRFA1_ASYNC_ON)

#elif RTC_TIMER_KHZ_LOG2 == 0
typedef TMilli TRtc;
#define RTC_TIMER_MODE		(ATMRFA1_CLK8_DIVIDE_32 | ATMRFA1_WGM8_NORMAL | ATMRFA1_ASYNC_ON)

#else
#error "The RTC must be run at 32 KHz, 4 KHz or 1 KHz."
#endif

#define RTC_TIMER_KHZ		(1 << RTC_TIMER_KHZ_LOG2)
#define RTC_TIMER_HZ		(32768ul / 32 * (1 << RTC_TIMER_KHZ_LOG2))
#define RTC_ALARM_MINDT		4
#define UQ_RTC_ALARM		"UQ_RTC_ALARM"

#endif//__TIMERCONFIG_H__
