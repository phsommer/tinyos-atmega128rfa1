// Note: Interrupt handlers have been renamed in "iom128rfa1.h"
// These macros define the vectors for the interrupt handlers in the atm128/atm1281 code. 


/* Timer/Counter0 Compare Match A */
#define SIG_OUTPUT_COMPARE0A _VECTOR(21)
/* Timer/Counter0 Compare Match B */
#define SIG_OUTPUT_COMPARE0B _VECTOR(22)
/* Timer/Counter0 Overflow */
#define SIG_OVERFLOW0 _VECTOR(23)

/* Timer/Counter1 Capture Event */
#define SIG_INPUT_CAPTURE1 _VECTOR(16)
/* Timer/Counter1 Compare Match A */
#define SIG_OUTPUT_COMPARE1A _VECTOR(17)
/* Timer/Counter1 Compare Match B */
#define SIG_OUTPUT_COMPARE1B _VECTOR(18)
/* Timer/Counter1 Compare Match C */
#define SIG_OUTPUT_COMPARE1C _VECTOR(19)
/* Timer/Counter1 Overflow */
#define SIG_OVERFLOW1 _VECTOR(20)

/* Timer/Counter2 Compare Match A */
#define SIG_OUTPUT_COMPARE2A _VECTOR(13)
/* Timer/Counter2 Overflow */
#define SIG_OVERFLOW2 _VECTOR(15)

/* Timer/Counter3 Compare Match A */
#define SIG_OUTPUT_COMPARE3A _VECTOR(32)
/* Timer/Counter3 Compare Match B */
#define SIG_OUTPUT_COMPARE3B _VECTOR(33)
/* Timer/Counter3 Compare Match C */
#define SIG_OUTPUT_COMPARE3C _VECTOR(34)
/* Timer/Counter3 Capture Event */
#define SIG_INPUT_CAPTURE3 _VECTOR(31)
/* Timer/Counter3 Overflow */
#define SIG_OVERFLOW3 _VECTOR(35)

/* USART1, Rx Complete */
#define SIG_USART1_RECV _VECTOR(36)
/* USART1, Tx Complete */
#define SIG_USART1_TRANS _VECTOR(38)

/* ADC Conversion Complete */
#define SIG_ADC _VECTOR(29)


/* disable watchdog timer after reset */
#include <avr/wdt.h>
#define platform_bootstrap() { MCUSR=0; wdt_disable(); }
