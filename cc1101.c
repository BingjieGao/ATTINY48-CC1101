#include "cc1101.h"
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/EEPROM.h>

//---------------------------------------
#define SCK_PIN			PORTB5			//
#define MISO_PIN		PORTB4			//
#define MOSI_PIN		PORTB3			//
#define SS_PIN			PORTB2			//
#define GDO0			PORTD5			//
#define GDO2			PORTD6			//
//----------------------------------------

//----------------------------------------
#define 	WRITE_BURST     	0x40		//
#define 	READ_SINGLE     	0x80		//
#define 	READ_BURST      	0xC0		//
#define 	BYTES_IN_RXFIFO     0x7F  		//
//-----------------------------------------

byte PaTable[8] = {0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60};

void SpiInit(void)
{
    DDRB = (1<<5)|(1<<3)|(1<<2);
    
    //Enable SPI master mode
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
    
}

void GDO_Set(void)
{
    DDRD = (0<<GDO0)|(0<<GDO2);
}
void Reset(void)
{
    PORTB = 0<<SS_PIN;
    _delay_ms(1);
    PORTB = 1<<SS_PIN;
    _delay_ms(1);
    
    PORTB = 0<<SS_PIN;
    while(!(SPSR & _BV(SPIF)))//while(MISO_PIN);
    SpiTransfer(CC1101_SRES);
    while(!(SPSR & _BV(SPIF)))//while(MISO_PIN);
    PORTB = 1<<SS_PIN;
}

void Init(void)
{
    GDO_Set();
//    PORTB = 1<<SS_PIN;
//    PORTB = 1<<SCK_PIN;
//    PORTB = 0<<MOSI_PIN;
    SpiInit();
    Reset();
    RegConfigSettings();
    //SpiWriteBurstReg(CC1101_PATABLE, PaTable,8);
}

void RegConfigSettings(void)
{
    SpiWriteReg(CC1101_FSCTRL1,  0x08);
    SpiWriteReg(CC1101_FSCTRL0,  0x00);
    SpiWriteReg(CC1101_FREQ2,    0x10);
    SpiWriteReg(CC1101_FREQ1,    0xA7);
    SpiWriteReg(CC1101_FREQ0,    0x62);
    SpiWriteReg(CC1101_MDMCFG4,  0x5B);
    SpiWriteReg(CC1101_MDMCFG3,  0xF8);
    SpiWriteReg(CC1101_MDMCFG2,  0x03);
    SpiWriteReg(CC1101_MDMCFG1,  0x22);
    SpiWriteReg(CC1101_MDMCFG0,  0xF8);
    SpiWriteReg(CC1101_CHANNR,   0x00);
    SpiWriteReg(CC1101_DEVIATN,  0x47);
    SpiWriteReg(CC1101_FREND1,   0xB6);
    SpiWriteReg(CC1101_FREND0,   0x10);
    SpiWriteReg(CC1101_MCSM0 ,   0x18);
    SpiWriteReg(CC1101_FOCCFG,   0x1D);
    SpiWriteReg(CC1101_BSCFG,    0x1C);
    SpiWriteReg(CC1101_AGCCTRL2, 0xC7);
    SpiWriteReg(CC1101_AGCCTRL1, 0x00);
    SpiWriteReg(CC1101_AGCCTRL0, 0xB2);
    SpiWriteReg(CC1101_FSCAL3,   0xEA);
    SpiWriteReg(CC1101_FSCAL2,   0x2A);
    SpiWriteReg(CC1101_FSCAL1,   0x00);
    SpiWriteReg(CC1101_FSCAL0,   0x11);
    SpiWriteReg(CC1101_FSTEST,   0x59);
    SpiWriteReg(CC1101_TEST2,    0x81);
    SpiWriteReg(CC1101_TEST1,    0x35);
    SpiWriteReg(CC1101_TEST0,    0x09);
    SpiWriteReg(CC1101_IOCFG2,   0x0B); 	//serial clock.synchronous to the data in synchronous serial mode
    SpiWriteReg(CC1101_IOCFG0,   0x06);  	//asserts when sync word has been sent/received, and de-asserts at the end of the packet
    SpiWriteReg(CC1101_PKTCTRL1, 0x04);		//two status bytes will be appended to the payload of the packet,including RSSI LQI and CRC OK
    //No address check
    SpiWriteReg(CC1101_PKTCTRL0, 0x05);		//whitening off;CRC Enable£»variable length packets, packet length configured by the first byte after sync word
    SpiWriteReg(CC1101_ADDR,     0x00);		//address used for packet filtration.
    SpiWriteReg(CC1101_PKTLEN,   0x3D); 	//61 bytes max length
}


byte SpiTransfer(byte Data)
{
    SPDR = Data;
    while(!(SPSR & (1<<SPIF)));
    return SPDR;
}

void SpiWriteReg(byte addr, byte data)
{
    PORTB = 0<<SS_PIN;
    while(MISO_PIN);
    SpiTransfer(addr);
    SpiTransfer(data);
    PORTB = 1<<SS_PIN;
}

void SpiWriteBurstReg(byte addr,byte *buffer, byte size)
{
    byte i, temp;
    temp = addr | WRITE_BURST;
    PORTB = 0<<SS_PIN;
    while(MISO_PIN);
    SpiTransfer(temp);
    for(i=0;i<size;i++)
    {
        SpiTransfer(buffer[i]);
    }
    PORTB = 1<<SS_PIN;
}
void SpiStrobe(byte strobe)
{
    PORTB = 0<<SS_PIN;
    while(MISO_PIN);
    SpiTransfer(strobe);
    PORTB = 1<<SS_PIN;
}


byte SpiReadReg(byte addr)
{
    byte temp, data;
    //changed status_register
    temp = addr|0x40;
    PORTB = 0<<SS_PIN;
    while(MISO_PIN);
    SpiTransfer(temp);
    data = SpiTransfer(0);
    PORTB = 1<<SS_PIN;
    return data;
}


void SpiReadBurstReg(byte addr, byte *buffer, byte size)
{
    byte i, temp;
    
    temp = addr|READ_BURST;
    PORTB = 0<<SS_PIN;
    while(MISO_PIN);
    SpiTransfer(temp);
    for(i=0;i<size;i++)
    {
        buffer[i] = SpiTransfer(0);
    }
    
    PORTB = 1<<SS_PIN;
}

byte SpiReadStatus(byte addr)
{
    byte data, temp;
    
    temp = addr|READ_BURST;
    PORTB = 0<<SS_PIN;
    while(MISO_PIN);
    SpiTransfer(temp);
    data = SpiTransfer(0);
    PORTB = 1<<SS_PIN;
    
    PORTC = 0x20;
    _delay_ms(200);
    PORTC=0x00;
    _delay_ms(200);
    
    return data;
}

void SendData(byte *txBuffer, byte size)
{
    SpiWriteReg(CC1101_TXFIFO,size);
    SpiWriteBurstReg(CC1101_TXFIFO,txBuffer,size);
    SpiStrobe(CC1101_STX);
    while(!GDO0);
    while(GDO0);
    SpiStrobe(CC1101_SFTX);
}

void SetReceive(void)
{
    SpiStrobe(CC1101_SRX);
}

byte CheckReceiveFlag(void)
{
    if(GDO0)
    {
        while(GDO0);
        return 1;
    }
    else
        return 0;
}

byte ReceiveData(byte *rxBuffer)
{
    byte size, status[2];
    
    if(SpiReadStatus(CC1101_RXBYTES) & BYTES_IN_RXFIFO)
    {
        size = SpiReadReg(CC1101_RXFIFO);
        SpiReadBurstReg(CC1101_RXFIFO,rxBuffer,size);
        SpiReadBurstReg(CC1101_RXFIFO,status,2);
        SpiStrobe(CC1101_SFRX);
        
        return size;
    }
    else
    {
        SpiStrobe(CC1101_SFRX);
        return 0;
    }
}

void setDefaultRegs(void)
{
    SpiWriteReg(CC1101_IOCFG2,  CC1101_DEFVAL_IOCFG2);
    SpiWriteReg(CC1101_IOCFG1,  CC1101_DEFVAL_IOCFG1);
    SpiWriteReg(CC1101_IOCFG0,  CC1101_DEFVAL_IOCFG0);
    SpiWriteReg(CC1101_FIFOTHR,  CC1101_DEFVAL_FIFOTHR);
    SpiWriteReg(CC1101_PKTLEN,  CC1101_DEFVAL_PKTLEN);
    SpiWriteReg(CC1101_PKTCTRL1,  CC1101_DEFVAL_PKTCTRL1);
    SpiWriteReg(CC1101_PKTCTRL0,  CC1101_DEFVAL_PKTCTRL0);
    
    // Set default synchronization word
    setSyncWord(CC1101_DEFVAL_SYNC1, CC1101_DEFVAL_SYNC0, false);
    
    // Set default device address
//    setDevAddress(CC1101_DEFVAL_ADDR, false);
    // Set default frequency channel
    setChannel(CC1101_DEFVAL_CHANNR, false);
    
    SpiWriteReg(CC1101_FSCTRL1,  CC1101_DEFVAL_FSCTRL1);
    SpiWriteReg(CC1101_FSCTRL0,  CC1101_DEFVAL_FSCTRL0);
    
    // Set default carrier frequency = 868 MHz
    setCarrierFreq(CFREQ_868);
    
    SpiWriteReg(CC1101_MDMCFG4,  CC1101_DEFVAL_MDMCFG4);
    SpiWriteReg(CC1101_MDMCFG3,  CC1101_DEFVAL_MDMCFG3);
    SpiWriteReg(CC1101_MDMCFG2,  CC1101_DEFVAL_MDMCFG2);
    SpiWriteReg(CC1101_MDMCFG1,  CC1101_DEFVAL_MDMCFG1);
    SpiWriteReg(CC1101_MDMCFG0,  CC1101_DEFVAL_MDMCFG0);
    SpiWriteReg(CC1101_DEVIATN,  CC1101_DEFVAL_DEVIATN);
    SpiWriteReg(CC1101_MCSM2,  CC1101_DEFVAL_MCSM2);
    SpiWriteReg(CC1101_MCSM1,  CC1101_DEFVAL_MCSM1);
    SpiWriteReg(CC1101_MCSM0,  CC1101_DEFVAL_MCSM0);
    SpiWriteReg(CC1101_FOCCFG,  CC1101_DEFVAL_FOCCFG);
    SpiWriteReg(CC1101_BSCFG,  CC1101_DEFVAL_BSCFG);
    SpiWriteReg(CC1101_AGCCTRL2,  CC1101_DEFVAL_AGCCTRL2);
    SpiWriteReg(CC1101_AGCCTRL1,  CC1101_DEFVAL_AGCCTRL1);
    SpiWriteReg(CC1101_AGCCTRL0,  CC1101_DEFVAL_AGCCTRL0);
    SpiWriteReg(CC1101_WOREVT1,  CC1101_DEFVAL_WOREVT1);
    SpiWriteReg(CC1101_WOREVT0,  CC1101_DEFVAL_WOREVT0);
    SpiWriteReg(CC1101_WORCTRL,  CC1101_DEFVAL_WORCTRL);
    SpiWriteReg(CC1101_FREND1,  CC1101_DEFVAL_FREND1);
    SpiWriteReg(CC1101_FREND0,  CC1101_DEFVAL_FREND0);
    SpiWriteReg(CC1101_FSCAL3,  CC1101_DEFVAL_FSCAL3);
    SpiWriteReg(CC1101_FSCAL2,  CC1101_DEFVAL_FSCAL2);
    SpiWriteReg(CC1101_FSCAL1,  CC1101_DEFVAL_FSCAL1);
    SpiWriteReg(CC1101_FSCAL0,  CC1101_DEFVAL_FSCAL0);
    SpiWriteReg(CC1101_RCCTRL1,  CC1101_DEFVAL_RCCTRL1);
    SpiWriteReg(CC1101_RCCTRL0,  CC1101_DEFVAL_RCCTRL0);
    SpiWriteReg(CC1101_FSTEST,  CC1101_DEFVAL_FSTEST);
    SpiWriteReg(CC1101_PTEST,  CC1101_DEFVAL_PTEST);
    SpiWriteReg(CC1101_AGCTEST,  CC1101_DEFVAL_AGCTEST);
    SpiWriteReg(CC1101_TEST2,  CC1101_DEFVAL_TEST2);
    SpiWriteReg(CC1101_TEST1,  CC1101_DEFVAL_TEST1);
    SpiWriteReg(CC1101_TEST0,  CC1101_DEFVAL_TEST0);
}












