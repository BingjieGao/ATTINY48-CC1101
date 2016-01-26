#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "TWI_Master.h"
#include "cc.h"
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
//        CC1101 cc1101;
        byte syncWord = 199;
        byte partum, version,marcstate;
        init();
        setSyncWordbytes(&syncWord, false);
        setCarrierFreq(CFREQ_433);
        disableAddressCheck();
//        add = SpiReadStatus(CC1101_VERSION);
        
        partum = readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER);
        version = readReg(CC1101_VERSION, CC1101_STATUS_REGISTER);
        marcstate = readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f;
        
        if (!TWIM_Start (slaveaddress, TWIM_WRITE))
        {
            TWIM_Stop ();            
        }
        else
        {
            TWIM_Write(30);
            TWIM_Write(version);
            TWIM_Write(partum);
            TWIM_Write(marcstate);
            TWIM_Stop ();
            _delay_ms (100);
        }
        
    }
}
