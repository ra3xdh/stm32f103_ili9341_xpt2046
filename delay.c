#include <stdint.h>

#include "delay.h"

void delay_tck(uint32_t d)
{
    for (uint32_t i = 0; i < d; i++) { /* Wait a bit. */
        __asm__( "NOP" );
    }
}

