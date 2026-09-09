#include "ticks.h"
#include "fsl_device_registers.h"

#define TICKS_RELOAD 0x00FFFFFFu

static uint32_t s_cycPerUs = 1u;
static uint32_t s_lastVal;
static uint32_t s_us;
static uint32_t s_carry; /* cycles not yet worth a whole microsecond */

void Ticks_Init(uint32_t coreClockHz)
{
    s_cycPerUs = coreClockHz / 1000000u;
    if (s_cycPerUs == 0u)
    {
        s_cycPerUs = 1u;
    }

    SysTick->CTRL = 0u;
    SysTick->LOAD = TICKS_RELOAD;
    SysTick->VAL  = 0u;
    /* Processor clock, counter enabled, no interrupt. */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    s_lastVal = SysTick->VAL;
    s_us      = 0u;
    s_carry   = 0u;
}

uint32_t Ticks_Us(void)
{
    uint32_t now = SysTick->VAL; /* counts down and reloads at TICKS_RELOAD */
    uint32_t elapsed;

    /* Masking to 24 bits turns the down count into the right answer across a reload. */
    elapsed   = (s_lastVal - now) & TICKS_RELOAD;
    s_lastVal = now;

    s_carry += elapsed;
    s_us    += s_carry / s_cycPerUs;
    s_carry %= s_cycPerUs;

    return s_us;
}

uint32_t Ticks_Ms(void)
{
    return Ticks_Us() / 1000u;
}
