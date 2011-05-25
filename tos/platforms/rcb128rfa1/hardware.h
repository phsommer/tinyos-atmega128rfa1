#ifndef HARDWARE_H
#define HARDWARE_H


#include <atm128hardware.h>
#include <Atm128Adc.h>


#define MHZ 16
#define PLATFORM_MHZ 16


// enum so components can override power saving, 
// as per TEP 112. 
enum { 
 TOS_SLEEP_NONE = ATM128_POWER_IDLE, 
}; 
 

#ifndef PLATFORM_BAUDRATE 
#define PLATFORM_BAUDRATE 57600L
#endif

#endif //HARDWARE_H
