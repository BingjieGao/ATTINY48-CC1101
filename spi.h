#ifndef _SPI_H
#define _SPI_H



/**
 * SPI pins
 */
#define SPI_SS   PB2     // PB2 = SPI_SS
#define SPI_MOSI PB3     // PB3 = MOSI
#define SPI_MISO PB4     // PB4 = MISO
#define SPI_SCK  PB5     // PB5 = SCK
#define GDO0	 PD5        // PD2 = INT0

#define PORT_SPI_MISO  PINB
#define BIT_SPI_MISO  4

#define PORT_SPI_SS  PORTB
#define BIT_SPI_SS   2

#define PORT_GDO0  PIND
#define BIT_GDO0  2

/**
 * Macros
 */
// Wait until SPI operation is terminated
#define wait_Spi()  while(!(SPSR & _BV(SPIF)))

/**
 * Class: SPI
 *
 * Description:
 * Basic SPI class
 */
class SPI
{
public:
    /**
     * init
     *
     * SPI initialization
     */
    void init();
    
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
    byte send(byte value);
};
#endif