/*
  (C)2023 Daniel Balsom
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

#define BAUD_RATE 230400

// Determines whether to jump to the specified load segment before executing the load program,
// or just let the address lines roll over.
// Enables the JumpVector state.
// If disabled, user programs shouldn't use the first few k of memory.
#define USE_LOAD_SEG 0

// Code segment to use for load program. User programs shouldn't jump here.
const u16 LOAD_SEG = 0xD000;

// Maximum size of the processor instruction queue. For 8088 == 4, 8086 == 6. 
#define QUEUE_MAX 4

// Debugging output for queue operations (flushes, regular queue ops are always reported)
#define TRACE_QUEUE 0

// Report state changes and time spent in each state
#define TRACE_STATE 1

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
  Store,
  Done
} machine_state;

// These defines control debugging output for each state.
#define TRACE_RESET 0
#define TRACE_VECTOR 0
#define TRACE_LOAD 0
#define TRACE_EXECUTE 1
#define TRACE_STORE 0
#define DEBUG_STORE 0


static const char MACHINE_STATE_CHARS[] = {
  'R', 'J', 'L', 'M', 'E', 'S', 'D'
};

static const char* MACHINE_STATE_STRINGS[] = {
  "Reset",
  "JumpVector",
  "Load",
  "LoadDone",
  "Execute",
  "Store",
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

// Bus transfer cycles. Tw is wait state, inserted if READY is not asserted during T3.
typedef enum { 
  t1 = 0,
  t2 = 1,
  t3 = 2,
  t4 = 3,
  tw = 4
} t_cycle;

// Strings for pretty-printing bus transfer cycles.
static const char* CYCLE_STRINGS[] = {
  "t1", "t2", "t3", "t4", "tw"
};

// CPU Registers
typedef struct registers {
  u16 ax;
  u16 bx;
  u16 cx;
  u16 dx;
  u16 ss;
  u16 sp;
  u16 flags;
  u16 ip;
  u16 cs;
  u16 ds;
  u16 es;
  u16 bp;
  u16 si;
  u16 di;
} registers __attribute__((packed)) ;

// Processor instruction queue
typedef struct queue {
  u8 queue[QUEUE_MAX];
  u8 types[QUEUE_MAX];
  u8 front;
  u8 back;
  u8 len;
} queue;

#define QUEUE_IDLE 0x00
#define QUEUE_FIRST 0x01
#define QUEUE_FLUSHED 0x02
#define QUEUE_SUBSEQUENT 0x03

// Strings for pretty-printing instruction queue status from QS0,QS1
// '.' = Idle  
// 'F' = First byte fetched 
// 'E' = Queue Emptied 
// 'S' = Subsequent byte fetched
static const char QUEUE_STATUS_CHARS[] = {
  '.', 'F', 'E', 'S'
};

// Data bus data types. These are stored when pushing to the prefetch queue, so we know what 
// kind of byte we are retrieving from the processor queue. This allows us to detect program
// end when the first non-program byte is fetched as the first byte of an instruction.
#define DATA_PROGRAM 0x00
#define DATA_PROGRAM_END 0x01

typedef struct program_stats {
  u16 code_read_xfers;
  u16 memory_read_xfers;
  u16 memory_write_xfers;
  u16 io_read_xfers;
  u16 io_write_xfers;
  u32 idle_cycles;
  u32 program_cycles;
} p_stats;

// Main CPU State
typedef struct cpu {
  bool doing_reset;
  machine_state v_state;
  u32 state_begin_time;
  u32 address_latch;
  s_state cycle_state; // Cycle state is bus state latched on T1
  s_state bus_state; // Bus state is current status of S0-S2 at given cycle (may not be valid)
  t_cycle bus_cycle;
  u8 data_bus;
  u8 data_type;
  u8 status0; // S0-S5, QS0 & QS1
  u8 command_bits; // 8288 command outputs
  u8 control_bits; // 8288 control outputs
  u16 v_pc; // Virtual program counter
  registers post_regs; // Register state retrieved from Store program
  u8 *readback_p;
  queue queue; // Instruction queue
  u8 opcode; // Currently executing opcode
  u8 qb; // Last byte value read from queue
  u8 qt; // Last data type read from queue
  bool q_ff; // Did we fetch a first instruction byte from the queue this cycle?
  u8 q_fn; // What # byte of instruction did we fetch?
} Cpu;

// How many cycles to hold the RESET signal high. Intel says "greater than 4" although 4 seems to work.
const int RESET_HOLD_CYCLE_COUNT = 5; 
// How many cycles it takes to reset the CPU after RESET signal goes low. First ALE should occur after this many cycles.
const int RESET_CYCLE_COUNT = 7; 
// If we didn't receive an ALE signal after this many cycles, give up
const int RESET_CYCLE_TIMEOUT = 14; 

// ----------------------------- CPU FLAGS ----------------------------------//
const u16 CPU_FLAG_CARRY      = 0b0000000000000001;
const u16 CPU_FLAG_RESERVED1  = 0b0000000000000010;
const u16 CPU_FLAG_PARITY     = 0b0000000000000100;
const u16 CPU_FLAG_RESERVED3  = 0b0000000000001000;
const u16 CPU_FLAG_AUX_CARRY  = 0b0000000000010000;
const u16 CPU_FLAG_RESERVED5  = 0b0000000000100000;
const u16 CPU_FLAG_ZERO       = 0b0000000001000000;
const u16 CPU_FLAG_SIGN       = 0b0000000010000000;
const u16 CPU_FLAG_TRAP       = 0b0000000100000000;
const u16 CPU_FLAG_INT_ENABLE = 0b0000001000000000;
const u16 CPU_FLAG_DIRECTION  = 0b0000010000000000;
const u16 CPU_FLAG_OVERFLOW   = 0b0000100000000000;

#define CPU_FLAG_DEFAULT_SET 0xF002
#define CPU_FLAG_DEFAULT_CLEAR 0xFFD7
// ----------------------------- GPIO PINS ----------------------------------//
#define BIT7 0x80
#define BIT6 0x40
#define BIT5 0x20
#define BIT4 0x10
#define BIT3 0x08
#define BIT2 0x04
#define BIT1 0x02
#define BIT0 0x01

// Time in microseconds to wait after setting clock HIGH or LOW
#define CLOCK_PIN_HIGH_DELAY 2
#define CLOCK_PIN_LOW_DELAY 0

// Microseconds to wait after a pin direction change. Without some sort of delay
// a subsequent read/write may fail.
#define PIN_CHANGE_DELAY 4

// Clock line #4 is controlled by PORTG bit #5.
const int CLK_PIN = 4;
#define SET_CLOCK_LOW PORTG &= ~0x20
#define SET_CLOCK_HIGH PORTG |= 0x20

// Reset pin #5 is controlled by PORTE bit #3.
const int RESET_PIN = 5;
#define SET_RESET_LOW PORTE &= ~0x08
#define SET_RESET_HIGH PORTE |= 0x08

const int READY_PIN = 6;
const int TEST_PIN = 7;

// ALE pin #50 status i in PINB bit #3.
#define ALE_PIN 50
#define READ_ALE_PIN ((PINB & 0x08) != 0)

// MRDC pin #51 is read by PINB bit #2
#define MRDC_PIN 51
#define READ_MRDC_PIN ((PINB & 0x04) != 0)

// AMWC pin #52 is read by PINB bit #1
#define AMWC_PIN 52
#define READ_AMWC_PIN ((PINB & 0x04) != 0)

// MWTC pin #53 is read by PINB bit #0
#define MWTC_PIN 53
#define READ_MWTC_PIN ((PINB & 0x01) != 0)

// IORC pin #46 is read by PINL bit #3
#define IORC_PIN 46
#define READ_IORC_PIN ((PINL & 0x08) != 0)

// AIOWC pin #48 is read by PINL bit #1
#define AIOWC_PIN 48
#define READ_AIOWC_PIN ((PINL & 0x02) != 0)

// IOWC pin #47 is read by PINL bit #2
#define IOWC_PIN 47
#define READ_IOWC_PIN ((PINL & 0x04) != 0)

// Address pins, used for slow address reading via digitalRead()
const int ADDRESS_PINS[] = {
  23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42
};
const int ADDRESS_LINES = 20;

// All input pins, used to set pin direction on setup
const int INPUT_PINS[] = {
  14,15,16,
  23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,
  43,44,45,46,47,48,49,50,51,52,53
};

// High word of cycle count
unsigned long CYCLE_NUM_H = 0;
// Low word of cycle count
unsigned long CYCLE_NUM = 0;

// Bit reverse LUT from http://graphics.stanford.edu/~seander/bithacks.html#BitReverseTable
static const u8 BIT_REVERSE_TABLE[256] = 
{
#   define R2(n)     n,     n + 2*64,     n + 1*64,     n + 3*64
#   define R4(n) R2(n), R2(n + 2*16), R2(n + 1*16), R2(n + 3*16)
#   define R6(n) R4(n), R4(n + 2*4 ), R4(n + 1*4 ), R4(n + 3*4 )
    R6(0), R6(2), R6(1), R6(3)
};

// --------------------- Function declarations --------------------------------
u32 calc_flat_address(u16 seg, u16 offset);

void clock_tick();
void data_bus_write(u8 byte);
u8 data_bus_read();

void latch_address();
void read_status0();


void init_queue();
void push_queue(u8 byte, u8 dtype);
void pop_queue(u8 *byte, u8 *dtype);
void empty_queue();
void print_queue();

#endif 