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
#include "arduino8088.h"

uint32_t calc_flat_address(uint16_t seg, uint16_t offset) {
  return ((uint32_t)seg << 4) + offset;
}

// -------------------------- CPU Interface -----------------------------------

// Execute one clock pulse to the CPU
void clock_tick() {
  WRITE_CLK(1);
  delayMicroseconds(CLOCK_PIN_HIGH_DELAY);
  WRITE_CLK(0);
  delayMicroseconds(CLOCK_PIN_LOW_DELAY);
  tick_i8288();
}

void tick_i8288() {
  #if EMULATE_8288
    I8288.last_status = (s_state)I8288.status;
    I8288.status = (s_state)(read_status0_raw() & 0x07);

    // TODO: Handle wait states
    switch(I8288.tcycle) {
      case TI:
        break;
      case T1:
        I8288.ale = false;
        I8288.tcycle = T2;
        switch(I8288.status_latch) {
            case IOR:
              I8288.iorc = true;
              break;
            case IOW:
              // Set AIOWC line on T3, IOWC is delayed to T3
              I8288.aiowc = true;
              break;
            case MEMW:
              // Set AMWC line on T2, MWTC is delayed to T3
              I8288.amwc = true;
              break;
            case CODE:
              I8288.mrdc = true;
              break;          
            case MEMR:
              I8288.mrdc = true;
              break;                  
        }
        break;
      case T2:
        I8288.tcycle = T3;
        switch(I8288.status_latch) {
            case IRQA:
              break;
            case IOW:
              I8288.iowc = true;
              break;
            case MEMW:
              I8288.mwtc = true;
              break;
        }        
        break;
      case T3:
        I8288.tcycle = T4;
        I8288.iorc = false;
        I8288.amwc = false;
        I8288.iowc = false;
        I8288.mrdc = false;
        I8288.aiowc = false;
        I8288.mwtc = false;        
      case T4:
        I8288.tcycle = TI;
        break;        
    }

    if (I8288.last_status == PASV && I8288.status != PASV) {
      // We started a bus cycle; enter t1 and set ALE
      I8288.ale = true;
      I8288.tcycle = T1;
      I8288.status_latch = I8288.status;
    }

  #else
    // Nothing to tick.
  #endif
}

void reset_i8288() {
  #if EMULATE_8288
    I8288.last_status = PASV;
    I8288.status = PASV;
    I8288.status_latch = PASV;
    I8288.tcycle = TI;
    I8288.ale = false;
    I8288.mrdc = false;
    I8288.amwc = false;
    I8288.mwtc = false;
    I8288.iorc = false;
    I8288.aiowc = false;
    I8288.iowc = false;
    I8288.inta = false;
  #endif
}

// Write a value to the CPU's data bus
void data_bus_write(DATA_BUS_TYPE data) {

  #if defined(__SAM3X8E__) // If Arduino DUE
    // Set data bus pins to OUTPUT
    PIOB->PIO_OER = BIT26;      // Pin 22
    PIOA->PIO_OER = BIT14 | BIT15; // Pins 23, 24
    PIOD->PIO_OER = BIT00 | BIT01 | BIT02 | BIT03 | BIT06; // Pins 25-29 except 28

    delayMicroseconds(PIN_CHANGE_DELAY);

    // Write byte to data bus pins
    (data & 0x01) ? PIOB->PIO_SODR = BIT26 : PIOB->PIO_CODR = BIT26;     // Pin 22
    (data & 0x02) ? PIOA->PIO_SODR = BIT14 : PIOA->PIO_CODR = BIT14;     // Pin 23
    (data & 0x04) ? PIOA->PIO_SODR = BIT15 : PIOA->PIO_CODR = BIT15;     // Pin 24
    (data & 0x08) ? PIOD->PIO_SODR = BIT00 : PIOD->PIO_CODR = BIT00;      // Pin 25
    (data & 0x10) ? PIOD->PIO_SODR = BIT01 : PIOD->PIO_CODR = BIT01;      // Pin 26
    (data & 0x20) ? PIOD->PIO_SODR = BIT02 : PIOD->PIO_CODR = BIT02;      // Pin 27
    (data & 0x40) ? PIOD->PIO_SODR = BIT03 : PIOD->PIO_CODR = BIT03;      // Pin 28
    (data & 0x80) ? PIOD->PIO_SODR = BIT06 : PIOD->PIO_CODR = BIT06;      // Pin 29
  
    #if(DATA_BUS_SIZE == 2)
    (data & 0x0100) ? PIOD->PIO_SODR = BIT09 : PIOD->PIO_CODR = BIT09;     // AD8 Pin 30 (PD9)
    (data & 0x0200) ? PIOA->PIO_SODR = BIT07 : PIOA->PIO_CODR = BIT07;     // AD9 Pin 31 (PA7)
    (data & 0x0400) ? PIOD->PIO_SODR = BIT10 : PIOD->PIO_CODR = BIT10;   // AD10 Pin 32 (PD10)
    (data & 0x0800) ? PIOC->PIO_SODR = BIT01 : PIOC->PIO_CODR = BIT01;    // AD11 Pin 33 (PC1)
    (data & 0x1000) ? PIOC->PIO_SODR = BIT02 : PIOC->PIO_CODR = BIT02;    // AD12 Pin 34 (PC2)
    (data & 0x2000) ? PIOC->PIO_SODR = BIT03 : PIOC->PIO_CODR = BIT03;    // AD13 Pin 35 (PC3)
    (data & 0x4000) ? PIOC->PIO_SODR = BIT04 : PIOC->PIO_CODR = BIT04;    // AD14 Pin 36 (PC4)
    (data & 0x8000) ? PIOC->PIO_SODR = BIT05 : PIOC->PIO_CODR = BIT05;    // AD15 Pin 37 (PC5)
    #endif

  #elif defined(__AVR_ATmega2560__) // If Arduino MEGA  
    // Set data bus pins 22-29 to OUTPUT
    DDRA = 0xFF;
    delayMicroseconds(PIN_CHANGE_DELAY);
    // TODO: Support 8086
    // Write byte to data bus pins 22-29
    PORTA = data;
  #endif
}

// Read a value from the CPU's data bus
DATA_BUS_TYPE data_bus_read() {

  #if defined(__SAM3X8E__) // If Arduino DUE  
    uint8_t data = 0;

    // Set data bus pins to INPUT
    PIOB->PIO_ODR = BIT26;      // Pin 22
    PIOA->PIO_ODR = BIT14 | BIT15; // Pins 23, 24
    PIOD->PIO_ODR = BIT00 | BIT01 | BIT02 | BIT03 | BIT06; // Pins 25-29 except 28

    delayMicroseconds(PIN_CHANGE_DELAY);

    // Read data from bus pins
    data |= (PIOB->PIO_PDSR & BIT26) ? 0x01 : 0x00;     // Pin 22, Bit 0 of byte
    data |= (PIOA->PIO_PDSR & BIT14) ? 0x02 : 0x00;     // Pin 23, Bit 1 of byte
    data |= (PIOA->PIO_PDSR & BIT15) ? 0x04 : 0x00;     // Pin 24, Bit 2 of byte
    data |= (PIOD->PIO_PDSR & BIT00) ? 0x08 : 0x00;     // Pin 25, Bit 3 of byte
    data |= (PIOD->PIO_PDSR & BIT01) ? 0x10 : 0x00;     // Pin 26, Bit 4 of byte
    data |= (PIOD->PIO_PDSR & BIT02) ? 0x20 : 0x00;     // Pin 27, Bit 5 of byte
    data |= (PIOD->PIO_PDSR & BIT03) ? 0x40 : 0x00;     // Pin 28, Bit 6 of byte
    data |= (PIOD->PIO_PDSR & BIT06) ? 0x80 : 0x00;     // Pin 29, Bit 7 of byte

    #if(DATA_BUS_SIZE == 2)
    data |= PIOD->PIO_PDSR & BIT09 ? 0x0100 : 0x0000;   // AD8 Pin 30 (PD9)
    data |= PIOA->PIO_PDSR & BIT07 ? 0x0200 : 0x0000;   // AD9 Pin 31 (PA7)
    data |= PIOD->PIO_PDSR & BIT10 ? 0x0400 : 0x0000;   // AD10 Pin 32 (PD10)
    data |= PIOC->PIO_PDSR & BIT01 ? 0x0800 : 0x0000;   // AD11 Pin 33 (PC1)
    data |= PIOC->PIO_PDSR & BIT02 ? 0x1000 : 0x0000;   // AD12 Pin 34 (PC2)
    data |= PIOC->PIO_PDSR & BIT03 ? 0x2000 : 0x0000;   // AD13 Pin 35 (PC3)
    data |= PIOC->PIO_PDSR & BIT04 ? 0x4000 : 0x0000;   // AD14 Pin 36 (PC4)
    data |= PIOC->PIO_PDSR & BIT05 ? 0x8000 : 0x0000;   // AD15 Pin 37 (PC5)
    #endif

    return data;

  #elif defined(__AVR_ATmega2560__) // If Arduino MEGA  
    // Set data bus pins 22-29 to INPUT
    DDRA = 0;
    delayMicroseconds(PIN_CHANGE_DELAY);
    // Read byte from data bus pins 22-29
    return PINA;

  #endif
}

// Read what value is being output on the data bus
DATA_BUS_TYPE data_bus_peek() {

  #if defined(__SAM3X8E__) // If Arduino DUE  
    uint8_t data = 0;

    // Read data from bus pins
    data |= (PIOB->PIO_ODSR & BIT26) ? 0x01 : 0x00;     // Pin 22, Bit 0 of byte
    data |= (PIOA->PIO_ODSR & BIT14) ? 0x02 : 0x00;     // Pin 23, Bit 1 of byte
    data |= (PIOA->PIO_ODSR & BIT15) ? 0x04 : 0x00;     // Pin 24, Bit 2 of byte
    data |= (PIOD->PIO_ODSR & BIT00) ? 0x08 : 0x00;     // Pin 25, Bit 3 of byte
    data |= (PIOD->PIO_ODSR & BIT01) ? 0x10 : 0x00;     // Pin 26, Bit 4 of byte
    data |= (PIOD->PIO_ODSR & BIT02) ? 0x20 : 0x00;     // Pin 27, Bit 5 of byte
    data |= (PIOD->PIO_ODSR & BIT03) ? 0x40 : 0x00;     // Pin 28, Bit 6 of byte
    data |= (PIOD->PIO_ODSR & BIT06) ? 0x80 : 0x00;     // Pin 29, Bit 7 of byte

    #if(DATA_BUS_SIZE == 2)
    data |= PIOD->PIO_ODSR & BIT09 ? 0x0100 : 0x0000;   // AD8 Pin 30 (PD9)
    data |= PIOA->PIO_ODSR & BIT07 ? 0x0200 : 0x0000;   // AD9 Pin 31 (PA7)
    data |= PIOD->PIO_ODSR & BIT10 ? 0x0400 : 0x0000;   // AD10 Pin 32 (PD10)
    data |= PIOC->PIO_ODSR & BIT01 ? 0x0800 : 0x0000;   // AD11 Pin 33 (PC1)
    data |= PIOC->PIO_ODSR & BIT02 ? 0x1000 : 0x0000;   // AD12 Pin 34 (PC2)
    data |= PIOC->PIO_ODSR & BIT03 ? 0x2000 : 0x0000;   // AD13 Pin 35 (PC3)
    data |= PIOC->PIO_ODSR & BIT04 ? 0x4000 : 0x0000;   // AD14 Pin 36 (PC4)
    data |= PIOC->PIO_ODSR & BIT05 ? 0x8000 : 0x0000;   // AD15 Pin 37 (PC5)
    #endif

    return data;

  #elif defined(__AVR_ATmega2560__) // If Arduino MEGA  
    // Read byte from data bus pins 22-29
    return PINA;
  #endif
}

void read_address() {
  CPU.address_bus = read_address_pins();
}

void latch_address() {
  uint32_t addr = read_address_pins();
  CPU.address_bus = addr;
  CPU.address_latch = addr;
}

/*
  Read the address pins and return the 20 bit value in a uint32
  Note: address is only valid while ALE is HIGH (on T1) Otherwise mutiplexed with status and data.
*/
uint32_t read_address_pins() {

  uint32_t address = 0;

  #if defined(__SAM3X8E__) // If Arduino DUE  
    
    // Set data bus pins to INPUT
    uint32_t pins_b = BIT26;                                    // Pin 22
    uint32_t pins_a = BIT07 | BIT14 | BIT15;                    // Pins 23, 24, 31
    uint32_t pins_c = 0x01FF;                                   // Pins 33-41
    uint32_t pins_d = BIT00 | BIT01 | BIT02 | BIT03 | BIT06 | BIT09 | BIT10;    // Pins 25-32 except 31  
    
    //PIOA->PIO_PER = pins_a;
    //PIOB->PIO_PER = pins_b;
    //PIOC->PIO_PER = pins_c;
    //PIOD->PIO_PER = pins_d;

    //delayMicroseconds(PIN_CHANGE_DELAY); // Wait for pin state change before reading

    PIOA->PIO_ODR = pins_a;
    PIOB->PIO_ODR = pins_b;
    PIOC->PIO_ODR = pins_c;
    PIOD->PIO_ODR = pins_d;
    //PIOA->PIO_PUER = pins_a;
    //PIOB->PIO_PUER = pins_b;
    //PIOC->PIO_PUER = pins_c;
    //PIOD->PIO_PUER = pins_d;

    delayMicroseconds(PIN_CHANGE_DELAY); // Wait for pin state change before reading
    
    address |= (PIOB->PIO_PDSR & BIT26) ? 0x00000001 : 0;     // AD0  Pin 22 (PB26)

    address |= (PIOA->PIO_PDSR & BIT14) ? 0x00000002 : 0;     // AD1  Pin 23 (PA14)
    address |= (PIOA->PIO_PDSR & BIT15) ? 0x00000004 : 0;     // AD2  Pin 24 (PA15)
    address |= (PIOD->PIO_PDSR & BIT00) ? 0x00000008 : 0;     // AD3  Pin 25 (PD0)
    address |= (PIOD->PIO_PDSR & BIT01) ? 0x00000010 : 0;     // AD4  Pin 26 (PD1)
    address |= (PIOD->PIO_PDSR & BIT02) ? 0x00000020 : 0;     // AD5  Pin 27 (PD2)
    address |= (PIOD->PIO_PDSR & BIT03) ? 0x00000040 : 0;     // AD6  Pin 28 (PD3)
    address |= (PIOD->PIO_PDSR & BIT06) ? 0x00000080 : 0;     // AD7  Pin 29 (PD6)
    address |= (PIOD->PIO_PDSR & BIT09) ? 0x00000100 : 0;     // AD8  Pin 30 (PD9)
    address |= (PIOA->PIO_PDSR & BIT07) ? 0x00000200 : 0;     // AD9  Pin 31 (PA7)
    address |= (PIOD->PIO_PDSR & BIT10) ? 0x00000400 : 0;     // AD10 Pin 32 (PD10)

    address |= (PIOC->PIO_PDSR & BIT01) ? 0x00000800 : 0;     // AD11 Pin 33
    address |= (PIOC->PIO_PDSR & BIT02) ? 0x00001000 : 0;     // AD12 Pin 34
    address |= (PIOC->PIO_PDSR & BIT03) ? 0x00002000 : 0;     // AD13 Pin 35
    address |= (PIOC->PIO_PDSR & BIT04) ? 0x00004000 : 0;     // AD14 Pin 36
    address |= (PIOC->PIO_PDSR & BIT05) ? 0x00008000 : 0;     // AD15 Pin 37
    address |= (PIOC->PIO_PDSR & BIT06) ? 0x00010000 : 0;     // AD16 Pin 38
    address |= (PIOC->PIO_PDSR & BIT07) ? 0x00020000 : 0;     // AD17 Pin 39
    address |= (PIOC->PIO_PDSR & BIT08) ? 0x00040000 : 0;     // AD18 Pin 40
    address |= (PIOC->PIO_PDSR & BIT09) ? 0x00080000 : 0;     // AD19 Pin 41

  #elif defined(__AVR_ATmega2560__) // If Arduino MEGA 
    // Set data bus pins 22-29 to INPUT
    DDRA = 0;
    delayMicroseconds(PIN_CHANGE_DELAY); // Wait for pin state change before reading
    address = PINA; // Pins 22-29
    address |= (unsigned long)BIT_REVERSE_TABLE[PINC] << 8; // Pins 30-37 (Bit order reversed)
    address |= (unsigned long)(PIND & 0x80) << 9; // Pin 38
    address |= (unsigned long)(BIT_REVERSE_TABLE[PING] & 0xE0) << 12; // Pins 39-40 (Bit order reversed)  
  #endif

  return address;
}

// Read the status lines S0-S5 as well as queue status lines QS0-QS1.
uint8_t read_status0_raw() {
  
  uint8_t status0 = 0;
  #if defined(__SAM3X8E__) // If Arduino DUE  
    status0 |= (PIOD->PIO_PDSR & BIT04) ? 0x01 : 0;     // S0  - Pin 14 (PD4)
    status0 |= (PIOD->PIO_PDSR & BIT05) ? 0x02 : 0;     // S1  - Pin 15 (PD5)
    status0 |= (PIOA->PIO_PDSR & BIT13) ? 0x04 : 0;     // S2  - Pin 16 (PA13)
    status0 |= (PIOC->PIO_PDSR & BIT06) ? 0x08 : 0;     // S3  - Pin 38 (PC6)
    status0 |= (PIOC->PIO_PDSR & BIT07) ? 0x10 : 0;     // S4  - Pin 39 (PC7)
    status0 |= (PIOC->PIO_PDSR & BIT08) ? 0x20 : 0;     // S5  - Pin 40 (PC8)
    status0 |= (PIOC->PIO_PDSR & BIT21) ? 0x40 : 0;     // QS0 - Pin 9 (PC21)
    status0 |= (PIOC->PIO_PDSR & BIT22) ? 0x80 : 0;     // QS1 - Pin 8 (PC22)

  #elif defined(__AVR_ATmega2560__) // If Arduino MEGA 
    status0 = (PINJ & BIT1) >> 1;         // S0  - Pin 14
    status0 |= (PINJ & BIT0) << 1;        // S1  - Pin 15
    status0 |= ((PINH & BIT1) >> 1) << 2; // S2  - Pin 16 (H1)
    status0 |= ((PIND & BIT7) >> 7) << 3; // S3  - Pin 38 (D7)
    status0 |= ((PING & BIT2) >> 2) << 4; // S4  - Pin 39 (G2)
    status0 |= ((PING & BIT1) >> 1) << 5; // S5  - Pin 40 (G1)
    status0 |= ((PINH & BIT6) >> 6) << 6; // QS0 - Pin 9 (H6)
    status0 |= ((PINH & BIT5) >> 5) << 7; // QS1 - Pin 8 (H5)
  #endif

  return status0;
}

void read_status0() {
  CPU.status0 = read_status0_raw();
}

// Read the i8288 command lines
void read_8288_command_bits() {
  uint8_t command = 0;
  command |= READ_MRDC_PIN ? 0x01 : 0;     // MRDC - Pin 51 (PC12)
  command |= READ_AMWC_PIN ? 0x02 : 0;     // AMWC - Pin 52 (PB21)
  command |= READ_MWTC_PIN ? 0x04 : 0;     // MWTC - Pin 53 (PB14)
  command |= READ_IORC_PIN ? 0x08 : 0;     // IORC - Pin 46 (PC17)
  command |= READ_AIOWC_PIN ? 0x10 : 0;     // AIOWC- Pin 48 (PC15)
  command |= READ_IOWC_PIN ? 0x20 : 0;     // IOWC - Pin 47 (PC16)
  command |= READ_INTA_PIN ? 0x40 : 0;     // INTA - Pin 45 (PC18)
  // Although not technically a command status, we have an extra bit, so we can stick ALE in here.
  command |= READ_ALE_PIN ? 0x80 : 0;     // ALE  - Pin 50 (PC13)
  CPU.command_bits = command;
}

// Read the i8288 control lines
void read_8288_control_bits() {
  uint8_t control = 0;
  #if defined(__SAM3X8E__) // If Arduino DUE  

    control |= READ_ALE_PIN ? 0x80 : 0;     // ALE      - Pin 50 (PC13)
    control |= (PIOC->PIO_PDSR & BIT14) ? 0x02 : 0;     // DTR      - Pin 49 (PC14)
    control |= (PIOA->PIO_PDSR & BIT20) ? 0x04 : 0;     // MCE/PDEN - Pin 43 (PA20)
    control |= (PIOC->PIO_PDSR & BIT19) ? 0x08 : 0;     // DEN      - Pin 44 (PC19)
    CPU.control_bits = control;
  #elif defined(__AVR_ATmega2560__) // If Arduino MEGA 

    control = READ_ALE_PIN ? 0x80 : 0;    // ALE      - Pin 50 (B3)
    control |= (PINL & BIT0) << 1;        // DTR      - Pin 49 (L0)
    control |= ((PINL & BIT6) >> 6) << 2; // MCE/PDEN - Pin 43 (L6)
    control |= ((PINL & BIT5) >> 5) << 3; // DEN      - Pin 44 (L5)
    CPU.control_bits = control;
  #endif
}

// Resets the CPU by asserting RESET line for at least 4 cycles and waits for ALE signal.
bool cpu_reset() {

  static char buf[7];

  digitalWrite(TEST_PIN, LOW);
  digitalWrite(INTR_PIN, LOW); 
  digitalWrite(NMI_PIN, LOW);

  memset(&CPU, 0, sizeof CPU);

  //CYCLE_NUM_H = 0;
  CYCLE_NUM = 0;
  bool ale_went_off = false;
  CPU.state_begin_time = 0;
  change_state(Reset);
  CPU.data_bus = 0x00; 
  init_queue();

  // Hold RESET high for 4 cycles

  WRITE_RESET(1);

  for (int i = 0; i < RESET_HOLD_CYCLE_COUNT; i++) {
    if (READ_ALE_PIN == false) {
      ale_went_off = true;
    }
    cycle();
  }

  // CPU didn't reset for some reason.
  if (ale_went_off == false) {
    set_error("CPU failed to reset: ALE not off!");   
    return false;
  }

  WRITE_RESET(0);

  // Clock CPU while waiting for ALE
  int ale_cycles = 0;

  // Reset takes 7 cycles, bit we can try for longer
  for ( int i = 0; i < RESET_CYCLE_TIMEOUT; i++ ) {
    cycle();

    read_status0();
    #if MODE_ASCII  
      snprintf(buf, 3, "%01X", CPU.status0 & 0x07);
      SERIAL.print(buf);
    #endif
    //clock_tick();
    ale_cycles++;      

    if (READ_ALE_PIN) {
      // ALE is active! CPU has successfully reset
      CPU.doing_reset = false;
      #if DEBUG_RESET
        Serial1.println("###############");
        Serial1.println("## Reset CPU ##");
        Serial1.println("###############");
      #endif
      return true;
    }
  }

  // ALE did not turn on within the specified cycle timeout, so we failed to reset the cpu.
  #if DEBUG_RESET
    Serial1.println("## Failed to reset CPU! ##");
  #endif
  set_error("CPU failed to reset: No ALE!");   
  return false;
}

// ---------------------- Processor Instruction Queue -------------------------
void init_queue() {
  CPU.queue.len = 0;
  CPU.queue.back = 0;
  CPU.queue.front = 0;
}

void push_queue(uint8_t byte, uint8_t dtype) {
  if(CPU.queue.len < QUEUE_MAX) {
    CPU.queue.queue[CPU.queue.front] = byte;
    CPU.queue.types[CPU.queue.front] = dtype;
    CPU.queue.front = (CPU.queue.front + 1) & 0x03;
    CPU.queue.len++;
  }
}

uint8_t pop_queue(uint8_t *byte, uint8_t *dtype) {
  if(CPU.queue.len > 0) {
    *byte = CPU.queue.queue[CPU.queue.back];
    *dtype = CPU.queue.types[CPU.queue.back];
    CPU.queue.back = (CPU.queue.back + 1) & 0x03;
    CPU.queue.len--;
    return *byte;
  }
  else {
    return 0;
  }
}

void empty_queue() {
  // Need to rewind the program counter by length of queue on flush.
  // Otherwise we would lose opcodes already fetched.
  CPU.v_pc -= CPU.queue.len;
  init_queue();
}

void print_queue() {
  char buf[(QUEUE_MAX * 2) + 1] = {0};
  char hex[3] = {0};
  uint8_t i;
  uint8_t byte;
  for(i = 0; i < CPU.queue.len; i++ ) {
    byte = CPU.queue.queue[(CPU.queue.back + i) % QUEUE_MAX];
    sprintf(hex, "%02X", byte);
    strcat(buf, hex);
  }
  Serial.println(buf);
}

uint8_t read_queue(size_t idx) {
  if(idx < CPU.queue.len) {
    return CPU.queue.queue[(CPU.queue.back + idx) % QUEUE_MAX];
  }
  else {
    return 0;
  }
}

const char *queue_to_string() {
  const size_t buf_len = (QUEUE_MAX * 2) + 1;
  static char buf[buf_len];
  char *buf_p = buf;
  *buf_p = 0;
  uint8_t byte;
  for(uint8_t i = 0; i < CPU.queue.len; i++ ) {
    byte = CPU.queue.queue[(CPU.queue.back + i) % QUEUE_MAX];
    snprintf(buf_p, buf_len - (i * 2), "%02X", byte);
    buf_p += 2;
  }  

  return buf;
}

void test_queue() {

  uint8_t i;
  for(i = 0; i < 4; i++ ) {
    push_queue(i, 0);
  }

  print_queue();

  uint8_t qt, qb;

  pop_queue(&qt, &qb);
  pop_queue(&qt, &qb);
  push_queue(5, 0);

  print_queue();
}

// ----------------------------------Buzzer------------------------------------
void beep(uint32_t time) {
  pinMode(BUZZER_PIN, OUTPUT);
  
  digitalWrite(BUZZER_PIN, HIGH);
  delay(time);
  digitalWrite(BUZZER_PIN, LOW);
  /*
  BUZZER_ON;
  delay(time);
  BUZZER_OFF;
  */
}

void error_beep() {
  beep(100);
  delay(100);
  beep(100);
  delay(100);
  beep(100);
}

// ----------------------------------Opcodes-----------------------------------

// Return the mnemonic name for the specified opcode. If the opcode is a group
// opcode, op2 should be specified and modrm set to true.
const char *get_opcode_str(uint8_t op1, uint8_t op2, bool modrm) {

  size_t op_idx = OPCODE_REFS[op1];
  size_t grp_idx = 0;

  if(!modrm) {
    // Just return primary opcode
    return OPCODE_STRS[op_idx];
  }
  else {
    // modrm is in use, check if this is a group instruction...
    if(IS_GRP_OP(op1)) {  
      // Lookup opcode group
      grp_idx = MODRM_OP(op2);

      switch(OPCODE_REFS[op1]) {
        case GRP1:
          return OPCODE_STRS_GRP1[grp_idx];
          break;        
        case GRP2A:
          return OPCODE_STRS_GRP2A[grp_idx];        
          break;    
        case GRP2B:
          return OPCODE_STRS_GRP2B[grp_idx];         
          break;                   
        case GRP3:
          return OPCODE_STRS_GRP3[grp_idx];        
          break;        
        case GRP4:
          return OPCODE_STRS_GRP4[grp_idx];          
          break;        
        case GRP5:
          return OPCODE_STRS_GRP5[grp_idx];         
          break;
        default:
          return "***";
          break;
      }
    }
    else {
      // Not a group instruction, just return as normal
      return OPCODE_STRS[op_idx];
    }
  }
}