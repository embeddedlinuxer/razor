
#include <ti/sysbios/timers/timer64/Timer.h>

void delayTmrIsr(void* arg);

extern Timer_Params   delayTimerarams;
extern Timer_Handle   delayTimerHandle;

Timer_Status            timerStatus;

volatile unsigned int    delayTimerFlag;
volatile unsigned int    delayTimerCount;

void delayTimerSetup(void)
{
    delayTimerCount = 0;
    delayTimerFlag  = 0;
}

void delayTmrIsr(void* arg)
{
    delayTimerCount++;
    delayTimerFlag  = 1;
}

void osalTimerStart(unsigned int ms)
{
    delayTimerCount = 0;
    delayTimerFlag  = 0;

    Timer_setPeriodMicroSecs(delayTimerHandle, (uint32_t)(ms*1000));
    Timer_start(delayTimerHandle);
}

void osalTimerStop()
{
    Timer_stop(delayTimerHandle);

    delayTimerCount = 0;
    delayTimerFlag  = 0;
}

unsigned int osalTimerExpired()
{
    return (delayTimerFlag == 1) ? 1 : 0;
}


// a blocking delay function
void osalTimerDelay(unsigned int ms)
{
    osalTimerStart(ms);
    while (delayTimerFlag == 0);
}
