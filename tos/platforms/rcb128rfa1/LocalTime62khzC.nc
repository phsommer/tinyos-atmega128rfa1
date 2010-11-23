
#include "Timer.h"
#include "HplAtmRfa1Timer.h"

configuration LocalTime62khzC
{
	provides interface LocalTime<T62khz>;
}

implementation
{
	components Counter62khz32C;
    components new CounterToLocalTimeC(T62khz) as LocalTime62khzC;

    LocalTime62khzC.Counter -> Counter62khz32C;  
	LocalTime = LocalTime62khzC;
}
