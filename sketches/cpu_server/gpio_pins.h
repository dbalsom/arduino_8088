/*
    Arduino8088 Copyright 2022-2024 Daniel Balsom
    https://github.com/dbalsom/arduino_8088

    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the “Software”),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER   
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
*/
#ifndef _ARDUINO8088_PINS_H
#define _ARDUINO8088_PINS_H

#define BIT00  (1u << 0)
#define BIT01  (1u << 1)
#define BIT02  (1u << 2)
#define BIT03  (1u << 3)
#define BIT04  (1u << 4)
#define BIT05  (1u << 5)
#define BIT06  (1u << 6)
#define BIT07  (1u << 7)
#define BIT08  (1u << 8)
#define BIT09  (1u << 9)
#define BIT10  (1u << 10)
#define BIT11  (1u << 11)
#define BIT12  (1u << 12)
#define BIT13  (1u << 13)
#define BIT14  (1u << 14)
#define BIT15  (1u << 15)
#define BIT16  (1u << 16)
#define BIT17  (1u << 17)
#define BIT18  (1u << 18)
#define BIT19  (1u << 19)
#define BIT20  (1u << 20)
#define BIT21  (1u << 21)
#define BIT22  (1u << 22)
#define BIT23  (1u << 23)
#define BIT24  (1u << 24)
#define BIT25  (1u << 25)
#define BIT26  (1u << 26)
#define BIT27  (1u << 27)
#define BIT28  (1u << 28)
#define BIT29  (1u << 29)
#define BIT30  (1u << 30)
#define BIT31  (1u << 31)

#define SET_BIT00  (1u << 0)
#define SET_BIT01  (1u << 1)
#define SET_BIT02  (1u << 2)
#define SET_BIT03  (1u << 3)
#define SET_BIT04  (1u << 4)
#define SET_BIT05  (1u << 5)
#define SET_BIT06  (1u << 6)
#define SET_BIT07  (1u << 7)
#define SET_BIT08  (1u << 8)
#define SET_BIT09  (1u << 9)
#define SET_BIT10  (1u << 10)
#define SET_BIT11  (1u << 11)
#define SET_BIT12  (1u << 12)
#define SET_BIT13  (1u << 13)
#define SET_BIT14  (1u << 14)
#define SET_BIT15  (1u << 15)

#define CLR_BIT00  ((1u << 0) << 16)
#define CLR_BIT01  ((1u << 1) << 16)
#define CLR_BIT02  ((1u << 2) << 16)
#define CLR_BIT03  ((1u << 3) << 16)
#define CLR_BIT04  ((1u << 4) << 16)
#define CLR_BIT05  ((1u << 5) << 16)
#define CLR_BIT06  ((1u << 6) << 16)
#define CLR_BIT07  ((1u << 7) << 16)
#define CLR_BIT08  ((1u << 8) << 16)
#define CLR_BIT09  ((1u << 9) << 16)
#define CLR_BIT10  ((1u << 10) << 16)
#define CLR_BIT11  ((1u << 11) << 16)
#define CLR_BIT12  ((1u << 12) << 16)
#define CLR_BIT13  ((1u << 13) << 16)
#define CLR_BIT14  ((1u << 14) << 16)
#define CLR_BIT15  ((1u << 15) << 16)

#if defined (ARDUINO_GIGA)
    // If Arduino GIGA

#elif defined(__SAM3X8E__) 
    // If Arduino DUE

    #define READ_PIN_D14      ((PIOD->PIO_PDSR & BIT04) != 0)
    #define READ_PIN_D15      ((PIOD->PIO_PDSR & BIT05) != 0)
    #define READ_PIN_D16      ((PIOA->PIO_PDSR & BIT13) != 0)

    #define READ_PIN_D09       ((PIOC->PIO_PDSR & BIT21) != 0)
    #define READ_PIN_D08       ((PIOC->PIO_PDSR & BIT22) != 0)

    #define READ_PIN_D38      ((PIOC->PIO_PDSR & BIT06) != 0)
    #define READ_PIN_D39      ((PIOC->PIO_PDSR & BIT07) != 0)
    #define READ_PIN_D40      ((PIOC->PIO_PDSR & BIT08) != 0)

    #define READ_PIN_D45      ((PIOC->PIO_PDSR & BIT18) != 0)
    #define READ_PIN_D46      ((PIOC->PIO_PDSR & BIT17) != 0)
    #define READ_PIN_D47      ((PIOC->PIO_PDSR & BIT16) != 0)
    #define READ_PIN_D48      ((PIOC->PIO_PDSR & BIT15) != 0)
    
    #define READ_PIN_D50      ((PIOC->PIO_PDSR & BIT13) != 0)
    #define READ_PIN_D51      ((PIOC->PIO_PDSR & BIT12) != 0)
    #define READ_PIN_D52      ((PIOB->PIO_PDSR & BIT21) != 0)
    #define READ_PIN_D53      ((PIOB->PIO_PDSR & BIT14) != 0)

#elif defined(__AVR_ATmega2560__) 
    // If Arduino MEGA
    #define READ_PIN_D08  ((PINH & BIT5) != 0) // QS1 - Pin 8 (H5)
    #define READ_PIN_D09  ((PINH & BIT6) != 0) // QS0 - Pin 9 (H6)

    #define READ_PIN_D14  ((PINJ & BIT1) != 0) // S0  - Pin 14
    #define READ_PIN_D15  ((PINJ & BIT0) != 0) // S1  - Pin 15
    #define READ_PIN_D16  ((PINH & BIT1) != 0) // S2  - Pin 16 (H1)
    #define READ_PIN_D38  ((PIND & BIT7) != 0) // S3  - Pin 38 (D7)
    #define READ_PIN_D39  ((PING & BIT2) != 0) // S4  - Pin 39 (G2)
    #define READ_PIN_D40  ((PING & BIT1) != 0) // S5  - Pin 40 (G1)
    
    #define READ_PIN_A0   ((PINF & 0x01) != 0)
    #define READ_PIN_A1   ((PINF & 0x02) != 0)
    #define READ_PIN_D50  ((PINB & 0x08) != 0)
    #define READ_PIN_D51  ((PINB & 0x04) != 0)
    #define READ_PIN_D52  ((PINB & 0x02) != 0)
    #define READ_PIN_D53  ((PINB & 0x01) != 0)
    #define READ_PIN_D46  ((PINL & 0x08) != 0)
    #define READ_PIN_D48  ((PINL & 0x02) != 0)
    #define READ_PIN_D47  ((PINL & 0x04) != 0)
    #define READ_PIN_D45  ((PINL & 0x10) != 0)
#endif
