#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "TWI_Master.h"
#include "cc.h"
#define slaveaddress    0x04
void cc1101signalsInterrupt(void);
void ReadLQI(void);
void ReadRSSI(void);
byte lqi,rssi;
byte packetAvailable = 0x01;
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
        byte partum, version,marcstate,marcstate_after,sentdata;
        struct CCPACKET packet;
        byte spitx;
        init();
        setSyncWordbytes(&syncWord, false);
        setCarrierFreq(CFREQ_433);
        disableAddressCheck();
        partum = readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER);
        version = readReg(CC1101_VERSION, CC1101_STATUS_REGISTER);
        marcstate = readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f;
        
        setRxState();
        
        //wait_GDO0_low();
        //wait_GDO0_high();
        Enable_Pcinterrupt(21, cc1101signalsInterrupt);
        _delay_ms(1000);
        cc1101_Select();
        wait_Miso();
        spitx = spisend(0x3D);
        wait_Miso();
        cc1101_Deselect();
        marcstate = readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f;
        if(readStatusReg(CC1101_RXBYTES) & 0x7F)
            sentdata = 0x40;
        else
            sentdata = 0x00;
//
//        byte n, l, len, *pDataBuf,length,crc;
//        *pDataBuf = 0x00;
//        // Get length byte in packet (safely)
//        n = readStatusReg(CC1101_RXBYTES);
//        do { l = n; n = readStatusReg(CC1101_RXBYTES);} while (n<2 && n!=l);
//        *pDataBuf++ = len =length=readStatusReg(CC1101_RXFIFO);
//        // Copy rest of packet (safely)
//        while (len>1) {
//            n = readStatusReg(CC1101_RXBYTES);
//            do { l = n; n = readStatusReg(CC1101_RXBYTES);; } while (n<2 && n!=l);
//            while (n<1){
//                *pDataBuf++ = len = readStatusReg(CC1101_RXFIFO);
//                len--; n--;
//            }
//        }
//        *pDataBuf++ = readStatusReg(CC1101_RXFIFO);
//        crc = bitRead(*pDataBuf, 7);
        //setIdleState();       // Enter IDLE state
        //flushRxFifo();
       // _delay_ms(2500);
        
//                packet.length = 1;
//                packet.data[0]=0x20;
//                packet.data[1]=0x20;
//                packet.data[2]=0;
//                packet.data[3]=1;
//                packet.data[4]=0;
//        
//                if(sendData(packet))
//                    sentdata = 0x40;
//                else
//                    sentdata=0x00;

        
        if (!TWIM_Start (slaveaddress, TWIM_WRITE))
        {
            TWIM_Stop ();
        }
        else
        {
            TWIM_Write(30);
            TWIM_Write(version);
            TWIM_Write(marcstate);
            TWIM_Write(sentdata);
            TWIM_Write(packetAvailable);
//            TWIM_Write();
//            TWIM_Write(crc);
            TWIM_Stop ();
            _delay_ms (300);
        }
        _delay_ms(4000);

    }
}
void cc1101signalsInterrupt(void){
    // set the flag that a package is available
    packetAvailable = 0x40;
}
void ReadLQI()
{
    byte val=0;
    val=(readReg(CC1101_LQI, CC1101_STATUS_REGISTER));
    lqi = 0x3F - (val & 0x3F);
}
void ReadRSSI()
{
    byte value=0;
    
    value=(readReg(CC1101_RSSI, CC1101_STATUS_REGISTER));
    if (value >= 128)
    {
        rssi = 255 - rssi;
        rssi /= 2;
        rssi += 74;
    }
    else
    {
        rssi = rssi/2;
        rssi += 74;
    }
}
