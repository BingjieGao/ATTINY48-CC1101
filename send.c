//#include <stdio.h>
//#include <avr/io.h>
//#include <avr/interrupt.h>
//#include <util/delay.h>
//#include "TWI_Master.h"
//#include "cc.h"
//#define slaveaddress    0x04
//int main(void) {
//    if (!TWIM_Init (100000))
//    {
//        
//        while (1);
//    }
//    /*
//     ** Endless loop
//     */
//    while (1)
//    {
//        //        CC1101 cc1101;
//        byte syncWord = 199;
//        byte partum, version,marcstate,marcstate_after,sentdata;
//        sentdata = 0x01;
//        init();
//        setSyncWordbytes(&syncWord, false);
//        setCarrierFreq(CFREQ_915);
//        disableAddressCheck();
//        
//        partum = readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER);
//        version = readReg(CC1101_VERSION, CC1101_STATUS_REGISTER);
//        marcstate = readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f;
//        
//        struct CCPACKET data;
//        data.length = 1;
//        data.data[0]="A";
//        data.data[1]="B";
//        data.data[2]="C";
//        data.data[3]="D";
//        data.data[4]="E";
//        rfState = RFSTATE_TX;
//        
//        // Enter RX state
//        //        setRxState();
//        //        marcstate_after =readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f;
//        if(sendData(data)){
//            sentdata = 4;
//        }else{
//            sentdata = 0;
//        }
//        
//        if (!TWIM_Start (slaveaddress, TWIM_WRITE))
//        {
//            TWIM_Stop ();
//        }
//        else
//        {
//            TWIM_Write(30);
//            TWIM_Write(version);
//            TWIM_Write(partum);
//            TWIM_Write(marcstate);
//            TWIM_Write(40);
//            TWIM_Write(sentdata);
//            TWIM_Stop ();
//            _delay_ms (100);
//        }
//        
//    }
//}
