#include "cc1101.h"
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "TWI_Master.h"
#define slaveaddress    0x04
int main(void) {
    if (!TWIM_Init (100000))
    {
        
        while (1);
    }
    /*
     ** Endless loop
     */
    while (1)
    {
        uint8_t add;
        //Init();
        SpiInit();

        add = SpiReadStatus(CC1101_VERSION);
        
        
        if (!TWIM_Start (slaveaddress, TWIM_WRITE))
        {
            TWIM_Stop ();            
        }
        else
        {
            TWIM_Write(30);
//            TWIM_Write(add);
            TWIM_Stop ();
            _delay_ms (100);
        }
        
    }
}
