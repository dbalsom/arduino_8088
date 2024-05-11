/*
  (C)2023-2024 Daniel Balsom
  https://github.com/dbalsom/arduino_8088

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/
#ifndef _ARDUINO8088_H
#define _ARDUINO8088_H

// Set this to 0 if 8088, 1 if 8086
#define CPU_TYPE 0

// Set this to 1 to use i8288 emulation
#define EMULATE_8288 1

// Define the board type in use.
#define ELEGOO_MEGA 1
#define ARDUINO_MEGA 2
#define ARDUINO_DUE 3
#define BOARD_TYPE ARDUINO_DUE

// These defines control debugging output for each state.
#define TRACE_RESET 0
#define TRACE_VECTOR 0
#define TRACE_LOAD 0
#define TRACE_EXECUTE 0
#define TRACE_STORE 0
#define DEBUG_STORE 0
#define TRACE_FINALIZE 0
#define DEBUG_FINALIZE 0

// Debugging output for queue operations (flushes, regular queue ops are always reported)
#define TRACE_QUEUE 0
// Report state changes and time spent in each state
#define TRACE_STATE 0


#if defined(__SAM3X8E__) // If Arduino DUE

  #define SERIAL SerialUSB
  #define FLUSH SERIAL.flush()
#elif defined(__AVR_ATmega2560__) // If Arduino MEGA

  #define SERIAL Serial
  #define FLUSH 
#elif defined(ARDUINO_GIGA)

  #define SERIAL Serial
  #define FLUSH 
#endif

// Determines whether to jump to the specified load segment before executing the load program,
// or just let the address lines roll over.
// Enables the JumpVector state.
// If disabled, user programs shouldn't use the first few k of memory.
#define USE_LOAD_SEG 1

// Code segment to use for load program. User programs shouldn't jump here.
const uint16_t LOAD_SEG = 0xD000;

#if (CPU_TYPE==0)
  // Maximum size of the processor instruction queue. For 8088 == 4, 8086 == 6. 
  #define QUEUE_MAX 4
  #define DATA_BUS_TYPE uint8_t
  #define DATA_BUS_SIZE 1
#else
  #define QUEUE_MAX 6
  #define DATA_BUS_TYPE uint16_t
  #define DATA_BUS_SIZE 2
#endif



// States for main program state machine:
// ----------------------------------------------------------------------------
// Reset - CPU is being reset
// JumpVector - CPU is jumping from reset vector to load segment (optional?)
// Load - CPU is executing register Load program
// LoadDone - CPU has finished executing Load program and waiting for program execution to start
// Execute - CPU is executing user program
// Store - CPU has is executing register Store program
typedef enum {
  Reset = 0,
  JumpVector,
  Load,
  LoadDone,
  Execute,
  ExecuteFinalize,
  ExecuteDone,
  Store,
  StoreDone,
  Done
} machine_state;

const char MACHINE_STATE_CHARS[] = {
  'R', 'J', 'L', 'M', 'E', 'F', 'X', 'S', 'T', 'D'
};

const char* MACHINE_STATE_STRINGS[] = {
  "Reset",
  "JumpVector",
  "Load",
  "LoadDone",
  "Execute",
  "ExecuteFinalize",
  "ExecuteDone",
  "Store",
  "StoreDone",
  "Done"
};

// Bus transfer states, as determined by status lines S0-S2.
typedef enum {
  IRQA = 0,   // IRQ Acknowledge
  IOR  = 1,   // IO Read
  IOW  = 2,   // IO Write
  HALT = 3,   // Halt
  CODE = 4,   // Code
  MEMR = 5,   // Memory Read
  MEMW = 6,   // Memory Write
  PASV = 7    // Passive
} s_state;

// Strings for printing bus states cycles.
const char *BUS_STATE_STRINGS[] = {
  "IRQA",
  "IOR ",
  "IOW ",
  "HALT",
  "CODE",
  "MEMR",
  "MEMW",
  "PASV"
};

// Bus transfer cycles. Tw is wait state, inserted if READY is not asserted during T3.
typedef enum { 
  T1 = 0,
  T2 = 1,
  T3 = 2,
  T4 = 3,
  TW = 4,
  TI = 5,
} t_cycle;


// Strings for printing bus transfer cycles.
const char *CYCLE_STRINGS[] = {
  "t1", "t2", "t3", "t4", "tw", "ti"
};

const char *SEGMENT_STRINGS[] = {
  "ES", "SS", "CS", "DS"
};

// CPU Registers
typedef struct registers {
  uint16_t ax;
  uint16_t bx;
  uint16_t cx;
  uint16_t dx;
  uint16_t ss;
  uint16_t sp;
  uint16_t flags;
  uint16_t ip;
  uint16_t cs;
  uint16_t ds;
  uint16_t es;
  uint16_t bp;
  uint16_t si;
  uint16_t di;
} registers __attribute__((packed)) ;

// Processor instruction queue
typedef struct queue {
  uint8_t queue[QUEUE_MAX];
  uint8_t types[QUEUE_MAX];
  uint8_t front;
  uint8_t back;
  uint8_t len;
} Queue;

#define QUEUE_IDLE 0x00
#define QUEUE_FIRST 0x01
#define QUEUE_FLUSHED 0x02
#define QUEUE_SUBSEQUENT 0x03

// Strings for pretty-printing instruction queue status from QS0,QS1
// '.' = Idle  
// 'F' = First byte fetched 
// 'E' = Queue Emptied 
// 'S' = Subsequent byte fetched
const char QUEUE_STATUS_CHARS[] = {
  ' ', 'F', 'E', 'S'
};

// Data bus data types. These are stored when pushing to the prefetch queue, so we know what 
// kind of byte we are retrieving from the processor queue. This allows us to detect program
// end when the first non-program byte is fetched as the first byte of an instruction.
#define DATA_PROGRAM 0x00
#define DATA_PROGRAM_END 0x01

typedef struct program_stats {
  uint16_t code_read_xfers;
  uint16_t memory_read_xfers;
  uint16_t memory_write_xfers;
  uint16_t io_read_xfers;
  uint16_t io_write_xfers;
  uint32_t idle_cycles;
  uint32_t program_cycles;
} p_stats;

// Main CPU State
typedef struct cpu {
  bool doing_reset;
  machine_state v_state;
  uint32_t state_begin_time;
  uint32_t address_latch;
  s_state bus_state_latched; // Bus state latched on T1 and valid for entire bus cycle (immediate bus state goes PASV on T3)
  s_state bus_state; // Bus state is current status of S0-S2 at given cycle (may not be valid)
  t_cycle bus_cycle;
  DATA_BUS_TYPE data_bus;
  uint8_t data_type;
  uint8_t status0; // S0-S5, QS0 & QS1
  uint8_t command_bits; // 8288 command outputs
  uint8_t control_bits; // 8288 control outputs
  uint16_t v_pc; // Virtual program counter
  registers post_regs; // Register state retrieved from Store program
  uint8_t *readback_p;
  Queue queue; // Instruction queue
  uint8_t opcode; // Currently executing opcode
  uint8_t qb; // Last byte value read from queue
  uint8_t qt; // Last data type read from queue
  bool q_ff; // Did we fetch a first instruction byte from the queue this cycle?
  uint8_t q_fn; // What # byte of instruction did we fetch?
} Cpu;

typedef struct i8288 {
  s_state last_status; // S0-S2 of previous cycle
  s_state status; // S0-S2 of current cycle
  s_state status_latch;
  t_cycle tcycle;
  bool ale;
  bool mrdc;
  bool amwc;
  bool mwtc;
  bool iorc;
  bool aiowc;
  bool iowc;
  bool inta;
} Intel8288;

// How many cycles to hold the RESET signal high. Intel says "greater than 4" although 4 seems to work.
const int RESET_HOLD_CYCLE_COUNT = 7; 
// How many cycles it takes to reset the CPU after RESET signal goes low. First ALE should occur after this many cycles.
const int RESET_CYCLE_COUNT = 7; 
// If we didn't receive an ALE signal after this many cycles, give up
const int RESET_CYCLE_TIMEOUT = 14; 

// ----------------------------- CPU FLAGS ----------------------------------//
const uint16_t CPU_FLAG_CARRY      = 0b0000000000000001;
const uint16_t CPU_FLAG_RESERVED1  = 0b0000000000000010;
const uint16_t CPU_FLAG_PARITY     = 0b0000000000000100;
const uint16_t CPU_FLAG_RESERVED3  = 0b0000000000001000;
const uint16_t CPU_FLAG_AUX_CARRY  = 0b0000000000010000;
const uint16_t CPU_FLAG_RESERVED5  = 0b0000000000100000;
const uint16_t CPU_FLAG_ZERO       = 0b0000000001000000;
const uint16_t CPU_FLAG_SIGN       = 0b0000000010000000;
const uint16_t CPU_FLAG_TRAP       = 0b0000000100000000;
const uint16_t CPU_FLAG_INT_ENABLE = 0b0000001000000000;
const uint16_t CPU_FLAG_DIRECTION  = 0b0000010000000000;
const uint16_t CPU_FLAG_OVERFLOW   = 0b0000100000000000;

#define CPU_FLAG_DEFAULT_SET 0xF002
#define CPU_FLAG_DEFAULT_CLEAR 0xFFD7
// ----------------------------- GPIO PINS ----------------------------------//

// Time in microseconds to wait after setting clock HIGH or LOW

#if defined(__AVR_ATmega2560__) // Arduino MEGA
  
  #define CLOCK_PIN_HIGH_DELAY 0
  #define CLOCK_PIN_LOW_DELAY 0

#elif defined(__SAM3X8E__) // If Arduino DUE

  #define CLOCK_PIN_HIGH_DELAY 1
  #define CLOCK_PIN_LOW_DELAY 0

#elif defined(ARDUINO_GIGA) 

  #define CLOCK_PIN_HIGH_DELAY 1
  #define CLOCK_PIN_LOW_DELAY 0

#endif

// Microseconds to wait after a pin direction change. Without some sort of delay
// a subsequent read/write may fail. You may need to tweak this if you have a 
// different board - some types need longer delays

#if defined(__AVR_ATmega2560__) // Arduino MEGA
  
  #if BOARD_TYPE == ELEGOO_MEGA 
    #define PIN_CHANGE_DELAY 3
  #elif BOARD_TYPE == ARDUINO_MEGA
    #define PIN_CHANGE_DELAY 1
  #endif

#elif defined(__SAM3X8E__) // If Arduino DUE

  #define PIN_CHANGE_DELAY 0

#elif defined(ARDUINO_GIGA)

  #define PIN_CHANGE_DELAY 0

#endif


// -----------------------------Buzzer ----------------------------------------
#define BUZZER_PIN 2

// ------------------------- CPU Control pins ---------------------------------

#define CLK_PIN = 4;
#define RESET_PIN = 5;

#if defined(__AVR_ATmega2560__) // If Arduino MEGA
  
  #define WRITE_BUZZER(x) ((x) ? (PORTE |= (1 << 4)) : (PORTE &= ~(1 << 4)))

#elif defined(__SAM3X8E__) // If Arduino DUE

  #define WRITE_BUZZER(x) ((x) ? (PIOB->PIO_SODR = PIO_PB25) : (PIOB->PIO_CODR = PIO_PB25))

#elif defined(ARDUINO_GIGA)

  // do buzzer here
  #define WRITE_BUZZER(x) ((X))
#endif

// -------------------------- CPU Input pins ----------------------------------

#define READY_PIN 6
#define TEST_PIN 7
#define LOCK_PIN 10
#define INTR_PIN 12
#define NMI_PIN 13

// -------------------------- CPU Output pins ---------------------------------
#define RQ_PIN 3

// --------------------------8288 Control Inputs ------------------------------
#define AEN_PIN 54
#define CEN_PIN 55

// --------------------------8288 Control lines -------------------------------
#define ALE_PIN 50
#define DTR_PIN 49
#define MCEPDEN_PIN 43
#define DEN_PIN 44

// --------------------------8288 Command lines -------------------------------
#define MRDC_PIN 51
#define AMWC_PIN 52
#define MWTC_PIN 53
#define IORC_PIN 46
#define AIOWC_PIN 48
#define IOWC_PIN 47
#define INTA_PIN 45

// -------------------------- Macro definitions  ---------------------------------

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


// Write macros
#if defined(__SAM3X8E__) // If Arduino DUE

  // D4: PC26* (some references say PA29 - didn't work)
  #define WRITE_CLK(x) ((x) ? (PIOC->PIO_SODR = BIT26) : (PIOC->PIO_CODR = BIT26))
  // D5: PC25
  #define WRITE_RESET(x) ((x) ? (PIOC->PIO_SODR = PIO_PC25) : (PIOC->PIO_CODR = PIO_PC25))
  // D6: PC24
  #define WRITE_READY_PIN(x) ((x) ? (PIOC->PIO_SODR = BIT24) : (PIOC->PIO_CODR = BIT24))
  // D7: PC23
  #define WRITE_TEST_PIN(x) ((x) ? (PIOC->PIO_SODR = BIT23) : (PIOC->PIO_CODR = BIT23))
  // D10: PC29*
  #define WRITE_LOCK_PIN(x) ((x) ? (PIOC->PIO_SODR = BIT29) : (PIOC->PIO_CODR = BIT29))
  // D12: PD8
  #define WRITE_INTR_PIN(x) ((x) ? (PIOD->PIO_SODR = BIT08) : (PIOD->PIO_CODR = BIT08))
  // D13: PB27
  #define WRITE_NMI_PIN(x) ((x) ? (PIOB->PIO_SODR = BIT27) : (PIOB->PIO_CODR = BIT27))
  // A0: PA16
  #define WRITE_AEN_PIN(x) ((x) ? (PIOA->PIO_SODR = BIT16) : (PIOA->PIO_CODR = BIT16))
  // A1: PA24
  #define WRITE_CEN_PIN(x) ((x) ? (PIOA->PIO_SODR = BIT24) : (PIOA->PIO_CODR = BIT24))

#elif defined(__AVR_ATmega2560__) // If Arduino MEGA

  #define WRITE_CLK(x)    ((x) ? (PORTG |= (1 << 5)) : (PORTG &= ~(1 << 5))) // CLK is PG5
  #define WRITE_RESET(x)  ((x) ? (PORTE |= (1 << 3)) : (PORTE &= ~(1 << 3))) // RESET is PE3
  #define WRITE_READY_PIN(x) ((x) ? (PORTH |= (1 << 3)) : (PORTH &= ~(1 << 3)))
  #define WRITE_TEST_PIN(x) ((x) ? (PORTH |= (1 << 4)) : (PORTH &= ~(1 << 4)))
  #define WRITE_LOCK_PIN(x) ((x) ? (PORTB |= (1 << 4)) : (PORTB &= ~(1 << 4)))
  #define WRITE_INTR_PIN(x) ((x) ? (PORTB |= (1 << 6)) : (PORTB &= ~(1 << 6)))
  #define WRITE_NMI_PIN(x) ((x) ? (PORTB |= (1 << 7)) : (PORTB &= ~(1 << 7)))

  #define WRITE_AEN_PIN(x) ((x) ? (PORTF |= 0x01) : (PORTF &= ~0x01))
  #define WRITE_CEN_PIN(x) ((x) ? (PORTF |= (1 << 1)) : (PORTF &= ~(1 << 1)))

#elif defined (ARDUINO_GIGA)

  // D4: PC26* (some references say PA29 - didn't work)
  #define WRITE_CLK(x) ((x) ? (PIOC->PIO_SODR = BIT26) : (PIOC->PIO_CODR = BIT26))
  // D5: PC25
  #define WRITE_RESET(x) ((x) ? (PIOC->PIO_SODR = PIO_PC25) : (PIOC->PIO_CODR = PIO_PC25))
  // D6: PC24
  #define WRITE_READY_PIN(x) ((x) ? (PIOC->PIO_SODR = BIT24) : (PIOC->PIO_CODR = BIT24))
  // D7: PC23
  #define WRITE_TEST_PIN(x) ((x) ? (PIOC->PIO_SODR = BIT23) : (PIOC->PIO_CODR = BIT23))
  // D10: PC29*
  #define WRITE_LOCK_PIN(x) ((x) ? (PIOC->PIO_SODR = BIT29) : (PIOC->PIO_CODR = BIT29))
  // D12: PD8
  #define WRITE_INTR_PIN(x) ((x) ? (PIOD->PIO_SODR = BIT08) : (PIOD->PIO_CODR = BIT08))
  // D13: PB27
  #define WRITE_NMI_PIN(x) ((x) ? (PIOB->PIO_SODR = BIT27) : (PIOB->PIO_CODR = BIT27))
  // A0: PA16
  #define WRITE_AEN_PIN(x) ((x) ? (PIOA->PIO_SODR = BIT16) : (PIOA->PIO_CODR = BIT16))
  // A1: PA24
  #define WRITE_CEN_PIN(x) ((x) ? (PIOA->PIO_SODR = BIT24) : (PIOA->PIO_CODR = BIT24))

#endif 

// Read macros

#if EMULATE_8288
  // D50: PC13
  #define READ_ALE_PIN       (I8288.ale)
  // D51: PC12
  #define READ_MRDC_PIN      (!I8288.mrdc)
  // D52: PB21
  #define READ_AMWC_PIN      (!I8288.amwc)
  // D53: PB14
  #define READ_MWTC_PIN      (!I8288.mwtc)
  // D46: PC17
  #define READ_IORC_PIN      (!I8288.iorc)
  // D48: PC15
  #define READ_AIOWC_PIN     (!I8288.aiowc)
  // D47: PC16
  #define READ_IOWC_PIN      (!I8288.iowc)
  // D45: PC18
  #define READ_INTA_PIN      (!I8288.inta)
#else
  #if defined(__SAM3X8E__) // If Arduino DUE

    #define READ_AEN_PIN       ((PIOD->PIO_PDSR & BIT10) != 0)
    #define READ_CEN_PIN       ((PIOD->PIO_PDSR & BIT09) != 0)

    // D50: PC13
    #define READ_ALE_PIN       ((PIOC->PIO_PDSR & BIT13) != 0)
    #define READ_DTR_PIN       ((PIOC->PIO_PDSR & BIT03) != 0)
    #define READ_MCEPDEN_PIN   ((PIOC->PIO_PDSR & BIT01) != 0)
    #define READ_DEN_PIN       ((PIOC->PIO_PDSR & BIT02) != 0)

    // D51: PC12
    #define READ_MRDC_PIN      ((PIOC->PIO_PDSR & BIT12) != 0)
    // D52: PB21
    #define READ_AMWC_PIN      ((PIOB->PIO_PDSR & BIT21) != 0)
    // D53: PB14
    #define READ_MWTC_PIN      ((PIOB->PIO_PDSR & BIT14) != 0)
    // D46: PC17
    #define READ_IORC_PIN      ((PIOC->PIO_PDSR & BIT17) != 0)
    // D48: PC15
    #define READ_AIOWC_PIN     ((PIOC->PIO_PDSR & BIT15) != 0)
    // D47: PC16
    #define READ_IOWC_PIN      ((PIOC->PIO_PDSR & BIT16) != 0)
    // D45: PC18
    #define READ_INTA_PIN      ((PIOC->PIO_PDSR & BIT18) != 0)

  #elif defined(__AVR_ATmega2560__) // If Arduino MEGA

    #define READ_AEN_PIN ((PINF & 0x01) != 0)
    #define READ_CEN_PIN ((PINF & 0x02) != 0)

    #define READ_ALE_PIN ((PINB & 0x08) != 0)
    #define READ_DTR_PIN ((PINL & 0x01) != 0)
    #define READ_MCEPDEN_PIN ((PINL & 0x40) != 0) 
    #define READ_DEN_PIN ((PINL & 0x20) != 0)

    #define READ_MRDC_PIN ((PINB & 0x04) != 0)
    #define READ_AMWC_PIN ((PINB & 0x02) != 0)
    #define READ_MWTC_PIN ((PINB & 0x01) != 0)
    #define READ_IORC_PIN ((PINL & 0x08) != 0)
    #define READ_AIOWC_PIN ((PINL & 0x02) != 0)
    #define READ_IOWC_PIN ((PINL & 0x04) != 0)
    #define READ_INTA_PIN ((PINL & 0x10) != 0)
  #elif defined(ARDUINO_GIGA)
    #define TODO
  #endif
#endif

// Address pins, used for slow address reading via digitalRead()
const int ADDRESS_PINS[] = {
  22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41
};
const int ADDRESS_LINES = 20;

// All output pins, used to set pin direction on setup
const int OUTPUT_PINS[] = {
  4,  // CLK
  5,  // RESET
  6,  // READY
  7,  // TEST
  12, // INTR
  13, // NMI,
  54, // AEN,
  55, // CEN
};

// All input pins, used to set pin direction on setup
const int INPUT_PINS[] = {
  3,8,9,10,11,14,15,16,
  22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,
  43,44,45,46,47,48,49,50,51,52,53
};

// High word of cycle count
unsigned long CYCLE_NUM_H = 0;
// Low word of cycle count
unsigned long CYCLE_NUM = 0;

// Bit reverse LUT from http://graphics.stanford.edu/~seander/bithacks.html#BitReverseTable
static const uint8_t BIT_REVERSE_TABLE[256] = 
{
#   define R2(n)    n,     n + 2*64,     n + 1*64,     n + 3*64
#   define R4(n) R2(n), R2(n + 2*16), R2(n + 1*16), R2(n + 3*16)
#   define R6(n) R4(n), R4(n + 2*4 ), R4(n + 1*4 ), R4(n + 3*4 )
    R6(0), R6(2), R6(1), R6(3)
};

#ifndef OPCODE_NOP
  #define OPCODE_NOP 0x90
#endif

// --------------------- Function declarations --------------------------------
uint32_t calc_flat_address(uint16_t seg, uint16_t offset);

void clock_tick();
void data_bus_write(DATA_BUS_TYPE byte);
DATA_BUS_TYPE data_bus_read();

void latch_address();
void read_status0();
bool cpu_reset();

void init_queue();
void push_queue(uint8_t byte, uint8_t dtype);
uint8_t pop_queue(uint8_t *byte, uint8_t *dtype);
void empty_queue();
void print_queue();
void read_queue();

#endif 