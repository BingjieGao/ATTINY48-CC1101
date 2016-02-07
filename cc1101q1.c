/**
 * Copyright (c) 2011 panStamp <contact@panstamp.com>
 *
 * This file is part of the panStamp project.
 *
 * panStamp  is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * any later version.
 *
 * panStamp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with panStamp; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301
 * USA
 *
 * Author: Daniel Berenguer
 * Creation date: 03/03/2011
 */

#include "cc.h"
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
#include "external_interrupt.h"
//#include "nvolat.h"

//----------------------------------------------------------------------------------------
#define EXTERNAL_NUM_INTERRUPTS 2
typedef void (*voidFuncPtr)(void);

#ifdef __cplusplus
} // extern "C"
#endif
static volatile voidFuncPtr intFunc[24];
//-----------------------------------------------------------------------------------------

/**
 * PATABLE
 */
//const byte paTable[8] = {0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60};

/**
 * CC1101
 *
 * Class constructor
 */
void CC1101(void)
{
    paTableByte = PA_LowPower;            // Priority = Low power
}

/**
 * wakeUp
 *
 * Wake up CC1101 from Power Down state
 */
void wakeUp(void)
{
    cc1101_Select();                      // Select CC1101
    wait_Miso();                          // Wait until MISO goes low
    cc1101_Deselect();                    // Deselect CC1101
}

/**
 * writeReg
 *
 * Write single register into the CC1101 IC via SPI
 *
 * 'regAddr'	Register address
 * 'value'	Value to be writen
 */
void writeReg(byte regAddr, byte value)
{
    cc1101_Select();                      // Select CC1101
    wait_Miso();                          // Wait until MISO goes low
    spisend(regAddr);                    // Send register address
    spisend(value);                      // Send value
    cc1101_Deselect();                    // Deselect CC1101
}

/**
 * writeBurstReg
 *
 * Write multiple registers into the CC1101 IC via SPI
 *
 * 'regAddr'	Register address
 * 'buffer'	Data to be writen
 * 'len'	Data length
 */
void writeBurstReg(byte regAddr, byte* buffer, byte len)
{
    byte addr, i;
    
    addr = regAddr | WRITE_BURST;         // Enable burst transfer
    cc1101_Select();                      // Select CC1101
    wait_Miso();                          // Wait until MISO goes low
    spisend(addr);                       // Send register address
    
    for(i=0 ; i<len ; i++)
        spisend(buffer[i]);                // Send value
    
    cc1101_Deselect();                    // Deselect CC1101
}

/**
 * cmdStrobe
 *
 * Send command strobe to the CC1101 IC via SPI
 *
 * 'cmd'	Command strobe
 */
void cmdStrobe(byte cmd)
{
    cc1101_Select();                      // Select CC1101
    wait_Miso();                          // Wait until MISO goes low
    spisend(cmd);                        // Send strobe command
    cc1101_Deselect();                    // Deselect CC1101
}

/**
 * readReg
 *
 * Read CC1101 register via SPI
 *
 * 'regAddr'	Register address
 * 'regType'	Type of register: CC1101_CONFIG_REGISTER or CC1101_STATUS_REGISTER
 *
 * Return:
 * 	Data byte returned by the CC1101 IC
 */
byte readReg(byte regAddr, byte regType)
{
    byte addr, val;
    
    addr = regAddr | regType;
    cc1101_Select();                      // Select CC1101
    wait_Miso();                          // Wait until MISO goes low
    spisend(addr);                       // Send register address
    val = spisend(0x00);                 // Read result
    cc1101_Deselect();                    // Deselect CC1101
    
    return val;
}

/**
 * readBurstReg
 *
 * Read burst data from CC1101 via SPI
 *
 * 'buffer'	Buffer where to copy the result to
 * 'regAddr'	Register address
 * 'len'	Data length
 */
void readBurstReg(byte * buffer, byte regAddr, byte len)
{
    byte addr, i;
    
    addr = regAddr | READ_BURST;
    cc1101_Select();                      // Select CC1101
    wait_Miso();                          // Wait until MISO goes low
    spisend(addr);                       // Send register address
    for(i=0 ; i<len ; i++)
        buffer[i] = spisend(0x00);         // Read result byte by byte
    cc1101_Deselect();                    // Deselect CC1101
}

/**
 * reset
 *
 * Reset CC1101
 */
void reset(void)
{
    cc1101_Deselect();                    // Deselect CC1101
    _delay_ms(5);
    cc1101_Select();                      // Select CC1101
    _delay_ms(10);
    cc1101_Deselect();                    // Deselect CC1101
    _delay_ms(41);
    cc1101_Select();                      // Select CC1101
    
    wait_Miso();                          // Wait until MISO goes low
    spisend(CC1101_SRES);                // Send reset command strobe
    wait_Miso();                          // Wait until MISO goes low
    
    cc1101_Deselect();                    // Deselect CC1101
    
    setDefaultRegs();                     // Reconfigure CC1101
    setRegsFromEeprom();                  // Take user settings from EEPROM
}

/**
 * setDefaultRegs
 *
 * Configure CC1101 registers
 */
void setDefaultRegs(void)
{
    writeReg(CC1101_IOCFG2,  CC1101_DEFVAL_IOCFG2);
    writeReg(CC1101_IOCFG1,  CC1101_DEFVAL_IOCFG1);
    writeReg(CC1101_IOCFG0,  CC1101_DEFVAL_IOCFG0);
    writeReg(CC1101_FIFOTHR,  CC1101_DEFVAL_FIFOTHR);
    writeReg(CC1101_PKTLEN,  CC1101_DEFVAL_PKTLEN);
    writeReg(CC1101_PKTCTRL1,  CC1101_DEFVAL_PKTCTRL1);
    writeReg(CC1101_PKTCTRL0,  CC1101_DEFVAL_PKTCTRL0);
    
    // Set default synchronization word
     setSyncWord(CC1101_DEFVAL_SYNC1, CC1101_DEFVAL_SYNC0, false);
    
    // // Set default device address
     setDevAddress(CC1101_DEFVAL_ADDR, false);
    // // Set default frequency channel
     setChannel(CC1101_DEFVAL_CHANNR, false);
    
    writeReg(CC1101_FSCTRL1,  CC1101_DEFVAL_FSCTRL1);
    writeReg(CC1101_FSCTRL0,  CC1101_DEFVAL_FSCTRL0);
    
    // Set default carrier frequency = 868 MHz
    setCarrierFreq(CFREQ_868);
    
    writeReg(CC1101_MDMCFG4,  CC1101_DEFVAL_MDMCFG4);
    writeReg(CC1101_MDMCFG3,  CC1101_DEFVAL_MDMCFG3);
    writeReg(CC1101_MDMCFG2,  CC1101_DEFVAL_MDMCFG2);
    writeReg(CC1101_MDMCFG1,  CC1101_DEFVAL_MDMCFG1);
    writeReg(CC1101_MDMCFG0,  CC1101_DEFVAL_MDMCFG0);
    writeReg(CC1101_DEVIATN,  CC1101_DEFVAL_DEVIATN);
    writeReg(CC1101_MCSM2,  CC1101_DEFVAL_MCSM2);
    writeReg(CC1101_MCSM1,  CC1101_DEFVAL_MCSM1);
    writeReg(CC1101_MCSM0,  CC1101_DEFVAL_MCSM0);
    writeReg(CC1101_FOCCFG,  CC1101_DEFVAL_FOCCFG);
    writeReg(CC1101_BSCFG,  CC1101_DEFVAL_BSCFG);
    writeReg(CC1101_AGCCTRL2,  CC1101_DEFVAL_AGCCTRL2);
    writeReg(CC1101_AGCCTRL1,  CC1101_DEFVAL_AGCCTRL1);
    writeReg(CC1101_AGCCTRL0,  CC1101_DEFVAL_AGCCTRL0);
    writeReg(CC1101_WOREVT1,  CC1101_DEFVAL_WOREVT1);
    writeReg(CC1101_WOREVT0,  CC1101_DEFVAL_WOREVT0);
    writeReg(CC1101_WORCTRL,  CC1101_DEFVAL_WORCTRL);
    writeReg(CC1101_FREND1,  CC1101_DEFVAL_FREND1);
    writeReg(CC1101_FREND0,  CC1101_DEFVAL_FREND0);
    writeReg(CC1101_FSCAL3,  CC1101_DEFVAL_FSCAL3);
    writeReg(CC1101_FSCAL2,  CC1101_DEFVAL_FSCAL2);
    writeReg(CC1101_FSCAL1,  CC1101_DEFVAL_FSCAL1);
    writeReg(CC1101_FSCAL0,  CC1101_DEFVAL_FSCAL0);
    writeReg(CC1101_RCCTRL1,  CC1101_DEFVAL_RCCTRL1);
    writeReg(CC1101_RCCTRL0,  CC1101_DEFVAL_RCCTRL0);
    writeReg(CC1101_FSTEST,  CC1101_DEFVAL_FSTEST);
    writeReg(CC1101_PTEST,  CC1101_DEFVAL_PTEST);
    writeReg(CC1101_AGCTEST,  CC1101_DEFVAL_AGCTEST);
    writeReg(CC1101_TEST2,  CC1101_DEFVAL_TEST2);
    writeReg(CC1101_TEST1,  CC1101_DEFVAL_TEST1);
    writeReg(CC1101_TEST0,  CC1101_DEFVAL_TEST0);
}

/**
 * init
 *
 * Initialize CC1101
 */
void init(void)
{
    spiinit();                           // Initialize SPI interface
    //spi.setClockDivider(SPI_CLOCK_DIV16);
    //spi.setBitOrder(MSBFIRST);
    
    DDRD=(0<<5);//pinMode(GDO0, INPUT);                 // Config GDO0 as input
    
    reset();                              // Reset CC1101
    
    // Configure PATABLE
    //writeBurstReg(CC1101_PATABLE, (byte*)paTable, 8);
    writeReg(CC1101_PATABLE, paTableByte);
}

/**
 * setSyncWord
 *
 * Set synchronization word
 *
 * 'syncH'	Synchronization word - High byte
 * 'syncL'	Synchronization word - Low byte
 * 'save' If TRUE, save parameter in EEPROM
 */
void setSyncWord(uint8_t syncH, uint8_t syncL, bool save)
{
    if ((syncWord[0] != syncH) || (syncWord[1] != syncL))
    {
        writeReg(CC1101_SYNC1, syncH);
        writeReg(CC1101_SYNC0, syncL);
        syncWord[0] = syncH;
        syncWord[1] = syncL;
        // Save in EEPROM
        if (save)
        {
            eeprom_write_byte(EEPROM_SYNC_WORD, syncH);
            eeprom_write_byte(EEPROM_SYNC_WORD + 1, syncL);
        }
    }
}

/**
 * setSyncWord (overriding method)
 *
 * Set synchronization word
 *
 * 'syncH'	Synchronization word - pointer to 2-byte array
 * 'save' If TRUE, save parameter in EEPROM
 */
void setSyncWordbytes(byte *sync, bool save)
{
    setSyncWord(sync[0], sync[1], save);
}

/**
 * setDevAddress
 *
 * Set device address
 *
 * 'addr'	Device address
 * 'save' If TRUE, save parameter in EEPROM
 */
void setDevAddress(byte addr, bool save)
{
    if (devAddress != addr)
    {
        writeReg(CC1101_ADDR, addr);
        devAddress = addr;
        // Save in EEPROM
        if (save)
            eeprom_write_byte(EEPROM_DEVICE_ADDR, addr);
    }
}

/**
 * setChannel
 *
 * Set frequency channel
 *
 * 'chnl'	Frequency channel
 * 'save' If TRUE, save parameter in EEPROM
 */
void setChannel(byte chnl, bool save)
{
    if (channel != chnl)
    {
        writeReg(CC1101_CHANNR,  chnl);
        channel = chnl;
        // Save in EEPROM
        if (save)
            eeprom_write_byte(EEPROM_FREQ_CHANNEL, chnl);
    }
}

/**
 * setCarrierFreq
 *
 * Set carrier frequency
 *
 * 'freq'	New carrier frequency
 */
void setCarrierFreq(byte freq)
{
    switch(freq)
    {
        case CFREQ_915:
            writeReg(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2_915);
            writeReg(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1_915);
            writeReg(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0_915);
            break;
        case CFREQ_433:
            writeReg(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2_433);
            writeReg(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1_433);
            writeReg(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0_433);
            break;
        default:
            writeReg(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2_868);
            writeReg(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1_868);
            writeReg(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0_868);
            break;
    }
    
    carrierFreq = freq;
}

/**
 * setRegsFromEeprom
 *
 * Set registers from EEPROM
 */
void setRegsFromEeprom(void)
{
    byte bVal;
    byte arrV[2];
    
    // Read RF channel from EEPROM
    bVal = eeprom_read_byte(EEPROM_FREQ_CHANNEL);
    // Set RF channel
    if (bVal < NUMBER_OF_FCHANNELS )
        setChannel(bVal, false);
    // Read Sync word from EEPROM
    arrV[0] = eeprom_read_byte(EEPROM_SYNC_WORD);
    arrV[1] = eeprom_read_byte(EEPROM_SYNC_WORD + 1);
    // Set Sync word. 0x00 and 0xFF values are not allowed
    if (((arrV[0] != 0x00) && (arrV[0] != 0xFF)) || ((arrV[1] != 0x00) && (arrV[1] != 0xFF)))
        setSyncWord(arrV[0], arrV[1], false);
    // Read device address from EEPROM
    bVal = eeprom_read_byte(EEPROM_DEVICE_ADDR);
    // Set device address
    if (bVal > 0)
        setDevAddress(bVal, false);
}

/**
 * setPowerDownState
 *
 * Put CC1101 into power-down state
 */
void setPowerDownState(void)
{
    // Comming from RX state, we need to enter the IDLE state first
    cmdStrobe(CC1101_SIDLE);
    // Enter Power-down state
    cmdStrobe(CC1101_SPWD);
}

/**
 * sendData
 *
 * Send data packet via RF
 *
 * 'packet'	Packet to be transmitted. First byte is the destination address
 *
 *  Return:
 *    True if the transmission succeeds
 *    False otherwise
 */
bool sendData(struct CCPACKET packet)
{
    byte marcState;
    bool res = false;
    
    // Declare to be in Tx state. This will avoid receiving packets whilst
    // transmitting
    rfState = RFSTATE_TX;
    
    // Enter RX state
    setRxState();
    
    // Check that the RX state has been entered
    //check marcstate !=0x11
    while (((marcState = readStatusReg(CC1101_MARCSTATE)) & 0x1F) != 0x0D)
    {
        if (marcState == 0x11)        // RX_OVERFLOW
            flushRxFifo();              // flush receive queue
    }
    
    _delay_ms(500);
    
    // Set data length at the first position of the TX FIFO
    writeReg(CC1101_TXFIFO,  packet.length);
    // Write data into the TX FIFO
    writeBurstReg(CC1101_TXFIFO, packet.data, packet.length);
    
    // CCA enabled: will enter TX state only if the channel is clear
    setTxState();
    
    // Check that TX state is being entered (state = RXTX_SETTLING)
    marcState = readStatusReg(CC1101_MARCSTATE) & 0x1F;
    if((marcState != 0x13) && (marcState != 0x14) && (marcState != 0x15))
    {
        setIdleState();       // Enter IDLE state
        flushTxFifo();        // Flush Tx FIFO
        setRxState();         // Back to RX state
        
        // Declare to be in Rx state
        rfState = RFSTATE_RX;
        return false;
    }
//
    // Wait for the sync word to be transmitted
    wait_GDO0_high();
    
    // Wait until the end of the packet transmission
    wait_GDO0_low();
//
//    // Check that the TX FIFO is empty
    if((readStatusReg(CC1101_TXBYTES) & 0x7F) == 0)
        res = true;
    
    setIdleState();       // Enter IDLE state
    _delay_ms(2000);
    flushTxFifo();        // Flush Tx FIFO
    
    // Enter back into RX state
    setRxState();
    
    // Declare to be in Rx state
    rfState = RFSTATE_RX;
//
    return res;
}

/**
 * receiveData
 *
 * Read data packet from RX FIFO
 *
 * 'packet'	Container for the packet received
 *
 * Return:
 * 	Amount of bytes received
 */
byte receiveData(struct CCPACKET * packet)
{
    byte val;
    byte rxBytes = readStatusReg(CC1101_RXBYTES);
    
    // Rx FIFO overflow?
    if ((readStatusReg(CC1101_MARCSTATE) & 0x1F) == 0x11)
    {
        // Flush Rx FIFO
        cmdStrobe(CC1101_SFRX);
        packet->length = 0;
    }
    // Any byte waiting to be read?
    else if (readStatusReg(CC1101_RXBYTES) & 0x7F)
    {
        // Read data length
        packet->length = readConfigReg(CC1101_RXFIFO);
        // If packet is too long
        if (packet->length > CC1101_DATA_LEN)
            packet->length = 0;   // Discard packet
        else
        {
            // Read data packet
            readBurstReg(packet->data, CC1101_RXFIFO, packet->length);
            // Read RSSI
            packet->rssi = readConfigReg(CC1101_RXFIFO);
            // Read LQI and CRC_OK
            val = readConfigReg(CC1101_RXFIFO);
            packet->lqi = val & 0x7F;
            packet->crc_ok = bitRead(val, 7);
        }
    }
    else
        packet->length = 0;
    
    // Flush RX FIFO. Don't uncomment
    //cmdStrobe(CC1101_SFRX);
    
    // Enter back into RX state
    setRxState();
    
    return packet->length;
    
    setIdleState();       // Enter IDLE state
    flushRxFifo();        // Flush Rx FIFO
    //cmdStrobe(CC1101_SCAL);
    
    // Back to RX state
    setRxState();
    
    return packet->length;
}

void spiinit(void)
{
    volatile char IOReg;
    DDRB = (1<<PB5)|(1<<PB3)|(1<<PB2);
    
    //Enable SPI master mode
    SPCR = (1<<SPE)|(1<<MSTR);
    IOReg   = SPSR;                 	// clear SPIF bit in SPSR
    IOReg   = SPDR;
    
}

/**
 * send
 *
 * Send byte via SPI
 *
 * 'value'	Value to be sent
 *
 * Return:
 * 	Response received from SPI slave
 */
byte spisend(byte value)
{
    SPDR = value;                          // Transfer byte via SPI
    wait_Spi();                            // Wait until SPI operation is terminated
    return SPDR;
}
byte spiread(void)
{
    byte data;
    wait_Spi();
    data = SPDR;
    return data;
}


//attachinterrupts
void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), int mode) {
    if(interruptNum < EXTERNAL_NUM_INTERRUPTS) {
        intFunc[interruptNum] = userFunc;
        
        // Configure the interrupt mode (trigger on low input, any change, rising
        // edge, or falling edge).  The mode constants were chosen to correspond
        // to the configuration bits in the hardware register, so we simply shift
        // the mode into place.
        
        // Enable the interrupt.
        
        switch (interruptNum) {
#if defined(__AVR_ATmega32U4__)
                // I hate doing this, but the register assignment differs between the 1280/2560
                // and the 32U4.  Since avrlib defines registers PCMSK1 and PCMSK2 that aren't
                // even present on the 32U4 this is the only way to distinguish between them.
            case 0:
                EICRA = (EICRA & ~((1<<ISC00) | (1<<ISC01))) | (mode << ISC00);
                EIMSK |= (1<<INT0);
                break;
            case 1:
                EICRA = (EICRA & ~((1<<ISC10) | (1<<ISC11))) | (mode << ISC10);
                EIMSK |= (1<<INT1);
                break;
            case 2:
                EICRA = (EICRA & ~((1<<ISC20) | (1<<ISC21))) | (mode << ISC20);
                EIMSK |= (1<<INT2);
                break;
            case 3:
                EICRA = (EICRA & ~((1<<ISC30) | (1<<ISC31))) | (mode << ISC30);
                EIMSK |= (1<<INT3);
                break;
            case 4:
                EICRB = (EICRB & ~((1<<ISC60) | (1<<ISC61))) | (mode << ISC60);
                EIMSK |= (1<<INT6);
                break;
#elif defined(EICRA) && defined(EICRB) && defined(EIMSK)
            case 2:
                EICRA = (EICRA & ~((1 << ISC00) | (1 << ISC01))) | (mode << ISC00);
                EIMSK |= (1 << INT0);
                break;
            case 3:
                EICRA = (EICRA & ~((1 << ISC10) | (1 << ISC11))) | (mode << ISC10);
                EIMSK |= (1 << INT1);
                break;
            case 4:
                EICRA = (EICRA & ~((1 << ISC20) | (1 << ISC21))) | (mode << ISC20);
                EIMSK |= (1 << INT2);
                break;
            case 5:
                EICRA = (EICRA & ~((1 << ISC30) | (1 << ISC31))) | (mode << ISC30);
                EIMSK |= (1 << INT3);
                break;
            case 0:
                EICRB = (EICRB & ~((1 << ISC40) | (1 << ISC41))) | (mode << ISC40);
                EIMSK |= (1 << INT4);
                break;
            case 1:
                EICRB = (EICRB & ~((1 << ISC50) | (1 << ISC51))) | (mode << ISC50);
                EIMSK |= (1 << INT5);
                break;
            case 6:
                EICRB = (EICRB & ~((1 << ISC60) | (1 << ISC61))) | (mode << ISC60);
                EIMSK |= (1 << INT6);
                break;
            case 7:
                EICRB = (EICRB & ~((1 << ISC70) | (1 << ISC71))) | (mode << ISC70);
                EIMSK |= (1 << INT7);
                break;
#else
            case 0:
#if defined(EICRA) && defined(ISC00) && defined(EIMSK)
                EICRA = (EICRA & ~((1 << ISC00) | (1 << ISC01))) | (mode << ISC00);
                EIMSK |= (1 << INT0);
#elif defined(MCUCR) && defined(ISC00) && defined(GICR)
                MCUCR = (MCUCR & ~((1 << ISC00) | (1 << ISC01))) | (mode << ISC00);
                GICR |= (1 << INT0);
#elif defined(MCUCR) && defined(ISC00) && defined(GIMSK)
                MCUCR = (MCUCR & ~((1 << ISC00) | (1 << ISC01))) | (mode << ISC00);
                GIMSK |= (1 << INT0);
#else
#error attachInterrupt not finished for this CPU (case 0)
#endif
                break;
                
            case 1:
#if defined(EICRA) && defined(ISC10) && defined(ISC11) && defined(EIMSK)
                EICRA = (EICRA & ~((1 << ISC10) | (1 << ISC11))) | (mode << ISC10);
                EIMSK |= (1 << INT1);
#elif defined(MCUCR) && defined(ISC10) && defined(ISC11) && defined(GICR)
                MCUCR = (MCUCR & ~((1 << ISC10) | (1 << ISC11))) | (mode << ISC10);
                GICR |= (1 << INT1);
#elif defined(MCUCR) && defined(ISC10) && defined(GIMSK) && defined(GIMSK)
                MCUCR = (MCUCR & ~((1 << ISC10) | (1 << ISC11))) | (mode << ISC10);
                GIMSK |= (1 << INT1);
#else
#warning attachInterrupt may need some more work for this cpu (case 1)
#endif
                break;
                
            case 2:
#if defined(EICRA) && defined(ISC20) && defined(ISC21) && defined(EIMSK)
                EICRA = (EICRA & ~((1 << ISC20) | (1 << ISC21))) | (mode << ISC20);
                EIMSK |= (1 << INT2);
#elif defined(MCUCR) && defined(ISC20) && defined(ISC21) && defined(GICR)
                MCUCR = (MCUCR & ~((1 << ISC20) | (1 << ISC21))) | (mode << ISC20);
                GICR |= (1 << INT2);
#elif defined(MCUCR) && defined(ISC20) && defined(GIMSK) && defined(GIMSK)
                MCUCR = (MCUCR & ~((1 << ISC20) | (1 << ISC21))) | (mode << ISC20);
                GIMSK |= (1 << INT2);
#endif
                break;
#endif
        }
    }
}



void Enable_Interrupt(uint8_t INT_NO)
{
    switch(INT_NO)
    {
        case 0:EIMSK|=(1<<INT0);
            break;
        case 1:EIMSK|=(1<<INT1);
            break;
        default:break;
    }
}

/*! \brief This function enables the external pin change interrupt.
 *
 *  \param PCINT_NO	The pin change interrupt which has to be enabled.
 */
void Enable_Pcinterrupt(uint8_t PCINT_NO,void (*userFunc)(void))
{
    intFunc[PCINT_NO] = userFunc;
    if(PCINT_NO>=0 && PCINT_NO<=7)
    {
        PCICR=(PCICR&(~(1<<PCIE0)))|(1<<PCIE0);
        
        switch(PCINT_NO)
        {
            case 0:PCMSK0|=(1<<PCINT0);
                break;
            case 1:PCMSK0|=(1<<PCINT1);
                break;
            case 2:PCMSK0|=(1<<PCINT2);
                break;
            case 3:PCMSK0|=(1<<PCINT3);
                break;
            case 4:PCMSK0|=(1<<PCINT4);
                break;
            case 5:PCMSK0|=(1<<PCINT5);
                break;
            case 6:PCMSK0|=(1<<PCINT6);
                break;
            case 7:PCMSK0|=(1<<PCINT7);
                break;
            default:break;
        }
    }
    else if(PCINT_NO>=8 && PCINT_NO<=15)
    {
        PCICR=(PCICR&(~(1<<PCIE1)))|(1<<PCIE1);
        
        switch(PCINT_NO)
        {
            case 8:PCMSK1|=(1<<PCINT8);
                break;
            case 9:PCMSK1|=(1<<PCINT9);
                break;
            case 10:PCMSK1|=(1<<PCINT10);
                break;
            case 11:PCMSK1|=(1<<PCINT11);
                break;
            case 12:PCMSK1|=(1<<PCINT12);
                break;
            case 13:PCMSK1|=(1<<PCINT13);
                break;
            case 14:PCMSK1|=(1<<PCINT14);
                break;
            case 15:PCMSK1|=(1<<PCINT15);
                break;
            default:break;
        }
    }
    else if(PCINT_NO>=16 && PCINT_NO<=23)
    {
        PCICR=(PCICR&(~(1<<PCIE2)))|(1<<PCIE2);
        
        switch(PCINT_NO)
        {
            case 16:PCMSK2|=(1<<PCINT16);
                break;
            case 17:PCMSK2|=(1<<PCINT17);
                break;
            case 18:PCMSK2|=(1<<PCINT18);
                break;
            case 19:PCMSK2|=(1<<PCINT19);
                break;
            case 20:PCMSK2|=(1<<PCINT20);
                break;
            case 21:PCMSK2|=(1<<PCINT21);
                break;
            case 22:PCMSK2|=(1<<PCINT22);
                break;
            case 23:PCMSK2|=(1<<PCINT23);
                break;
            default:break;
        }
    }
    else
    {
        PCICR=(PCICR&(~(1<<PCIE3)))|(1<<PCIE3);
        
        switch(PCINT_NO)
        {
            case 24:PCMSK3|=(1<<PCINT24);
                break;
            case 25:PCMSK3|=(1<<PCINT25);
                break;
            case 26:PCMSK3|=(1<<PCINT26);
                break;
            case 27:PCMSK3|=(1<<PCINT27);
                break;
            default:break;
        }
    }
}

/*! \brief This function disables the external interrupt.
 *
 *  \param INT_NO	The interrupt which has to be disabled.
 */
void Disable_Interrupt(uint8_t INT_NO)
{
    switch(INT_NO)
    {
        case 0:EIMSK=(EIMSK&(~(1<<INT0)));
            break;
        case 1:EIMSK=(EIMSK&(~(1<<INT1)));
            break;
        default:break;
    }
}

/*! \brief This function disables the external pin change interrupt.
 *
 *  \param PCINT_NO	The pin change interrupt which has to be disabled.
 */
void Disable_Pcinterrupt(uint8_t PCINT_NO)
{
    switch(PCINT_NO)
    {
        case 0:PCMSK0=(PCMSK0&(~(1<<PCINT0)));
            break;
        case 1:PCMSK0=(PCMSK0&(~(1<<PCINT1)));
            break;
        case 2:PCMSK0=(PCMSK0&(~(1<<PCINT2)));
            break;
        case 3:PCMSK0=(PCMSK0&(~(1<<PCINT3)));
            break;
        case 4:PCMSK0=(PCMSK0&(~(1<<PCINT4)));
            break;
        case 5:PCMSK0=(PCMSK0&(~(1<<PCINT5)));
            break;
        case 6:PCMSK0=(PCMSK0&(~(1<<PCINT6)));
            break;
        case 7:PCMSK0=(PCMSK0&(~(1<<PCINT7)));
            break;
        case 8:PCMSK1=(PCMSK1&(~(1<<PCINT8)));
            break;
        case 9:PCMSK1=(PCMSK1&(~(1<<PCINT9)));
            break;
        case 10:PCMSK1=(PCMSK1&(~(1<<PCINT10)));
            break;
        case 11:PCMSK1=(PCMSK1&(~(1<<PCINT11)));
            break;
        case 12:PCMSK1=(PCMSK1&(~(1<<PCINT12)));
            break;
        case 13:PCMSK1=(PCMSK1&(~(1<<PCINT13)));
            break;
        case 14:PCMSK1=(PCMSK1&(~(1<<PCINT14)));
            break;
        case 15:PCMSK1=(PCMSK1&(~(1<<PCINT15)));
            break;
        case 16:PCMSK2=(PCMSK2&(~(1<<PCINT16)));
            break;
        case 17:PCMSK2=(PCMSK2&(~(1<<PCINT17)));
            break;
        case 18:PCMSK2=(PCMSK2&(~(1<<PCINT18)));
            break;
        case 19:PCMSK2=(PCMSK2&(~(1<<PCINT19)));
            break;
        case 20:PCMSK2=(PCMSK2&(~(1<<PCINT20)));
            break;
        case 21:PCMSK2=(PCMSK2&(~(1<<PCINT21)));
            break;
        case 22:PCMSK2=(PCMSK2&(~(1<<PCINT22)));
            break;
        case 23:PCMSK2=(PCMSK2&(~(1<<PCINT23)));
            break;
        case 24:PCMSK3=(PCMSK3&(~(1<<PCINT24)));
            break;
        case 25:PCMSK3=(PCMSK3&(~(1<<PCINT25)));
            break;
        case 26:PCMSK3=(PCMSK3&(~(1<<PCINT26)));
            break;
        case 27:PCMSK3=(PCMSK3&(~(1<<PCINT27)));
            break;
        default:break;
    }
    
    if(PCMSK0 == 0x00)
    {
        PCICR=(PCICR&(~(1<<PCIE0)));
    }
    else if(PCMSK1 == 0x00)
    {
        PCICR=(PCICR&(~(1<<PCIE1)));
    }
    else if(PCMSK2 == 0x00)
    {
        PCICR=(PCICR&(~(1<<PCIE2)));
    }
    else if(PCMSK3 == 0x00)
    {
        PCICR=(PCICR&(~(1<<PCIE3)));
    }
}

//byte halSpiReadStatus(byte addr)
//{
//    unsigned char value;
//    cc1101_Select();
//    wait_GDO1_high();
//    addr|=READ_BURST;
//    SPI_write(addr);
//    value = spiread();
//    cc1101_Deselect();
//    return value;
//}








