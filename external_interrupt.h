/*! \file *********************************************************************
 *
 * \brief  ATtiny88 external interrupts driver header file.
 *
 *      This file contains the enumerator definitions for various
 *      external interrupts for the ATmega2560 external interrupt driver.
 *
 *      The driver is not intended for size and/or speed critical code, since
 *      most functions are just a few lines of code, and the function call
 *      overhead would decrease code performance. The driver is intended for
 *      rapid prototyping and documentation purposes for getting started with
 *      the ATtiny88 external interrupts.
 *
 *      For size and/or speed critical code, it is recommended to copy the
 *      function contents directly into your application instead of making
 *      a function call.
 *
 * \par Application note:
 *      AVR1201: Using external interrupts for tinyAVR devices
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * Copyright (c) 2011, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include<avr/io.h>

/* Definition of macros */

/*! \brief Macros for ordinary external interrupts
 *
 *  These macros assigns an integer value for each INT interrupt
 */
#define INTR0 0
#define INTR1 1

/*! \brief Macros for interrupt sense configuration
 *
 *  These macros assigns an integer value for each INT sense configuration
 */
#define LEVEL 0
#define EDGE 1
#define FALLING 2
#define RISING 3

/*! \brief Macros for pin change external interrupts
 *
 *  These macros assigns an integer value for each pin change interrupt
 */
#define PCINTR0 0
#define PCINTR1 1
#define PCINTR2 2
#define PCINTR3 3
#define PCINTR4 4
#define PCINTR5 5
#define PCINTR6 6
#define PCINTR7 7
#define PCINTR8 8
#define PCINTR9 9
#define PCINTR10 10
#define PCINTR11 11
#define PCINTR12 12
#define PCINTR13 13
#define PCINTR14 14
#define PCINTR15 15
#define PCINTR16 16
#define PCINTR17 17
#define PCINTR18 18
#define PCINTR19 19
#define PCINTR20 20
#define PCINTR21 21
#define PCINTR22 22
#define PCINTR23 23
#define PCINTR24 24
#define PCINTR25 25
#define PCINTR26 26
#define PCINTR27 27
