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
#include <Arduino.h>
#include "arduino8088.h"
#include "cpu_server.h"
#include "opcodes.h"

static Server SERVER;
static Cpu CPU;
static Intel8288 I8288;

registers LOAD_REGISTERS = {
  0x0000, // AX
  0x0000, // BX
  0x0000, // CX
  0x0000, // DX
  0x0000, // SS
  0x0000, // SP
  0x0000, // FLAGS
  0x0000, // IP
  0xFFFF, // CS
  0x0000, // DS
  0x0000, // ES
  0x0000, // BP
  0x0000, // SI
  0x0000, // DI
};

// Register load routine.
uint8_t LOAD_PROGRAM[] = {
  0x00, 0x00, 0xB8, 0x00, 0x00, 0x8E, 0xD0, 0x89, 0xC4, 0x9D, 0xBB, 0x00, 0x00, 0xB9, 0x00, 0x00,
  0xBA, 0x00, 0x00, 0xB8, 0x00, 0x00, 0x8E, 0xD0, 0xB8, 0x00, 0x00, 0x8E, 0xD8, 0xB8, 0x00, 0x00,
  0x8E, 0xC0, 0xB8, 0x00, 0x00, 0x89, 0xC4, 0xB8, 0x00, 0x00, 0x89, 0xC5, 0xB8, 0x00, 0x00, 0x89,
  0xC6, 0xB8, 0x00, 0x00, 0x89, 0xC7, 0xB8, 0x00, 0x00, 0xEA, 0x00, 0x00, 0x00, 0x00
};

uint8_t JUMP_VECTOR[] = {
  0xEA, 0x00, 0x00, 0x00, 0x00
};

// Patch offsets for load routine
size_t LOAD_BX = 0x0B;
size_t LOAD_CX = 0x0E;
size_t LOAD_DX = 0x11;
size_t LOAD_SS = 0x14;
size_t LOAD_DS = 0x19;
size_t LOAD_ES = 0x1E;
size_t LOAD_SP = 0x23;
size_t LOAD_BP = 0x28;
size_t LOAD_SI = 0x2D;
size_t LOAD_DI = 0x32;
size_t LOAD_AX = 0x37;
size_t LOAD_IP = 0x3A;
size_t LOAD_CS = 0x3C;

// Register store routine.
// Four NOPs have been padded to the front of the STORE routine to hide it from appearing in cycle
// traces (although we'd need 6 for 8086...). 
// We can probably think of a better way to hide STORE program bytes, as this adds several cycles.
const uint8_t STORE_PROGRAM[] = {
  0x90, 0x90, 0x90, 0x90,
  0xE7, 0xFE, 0x89, 0xD8, 0xE7, 0xFE, 0x89, 0xC8, 0xE7, 0xFE, 0x89, 0xD0, 0xE7, 0xFE, 0x8C, 0xD0,
  0xE7, 0xFE, 0x89, 0xE0, 0xE7, 0xFE, 0xB8, 0x00, 0x00, 0x8E, 0xD0, 0xB8, 0x04, 0x00, 0x89, 0xC4,
  0x9C, 0xE8, 0x00, 0x00, 0x8C, 0xC8, 0xE7, 0xFE, 0x8C, 0xD8, 0xE7, 0xFE, 0x8C, 0xC0, 0xE7, 0xFE,
  0x89, 0xE8, 0xE7, 0xFE, 0x89, 0xF0, 0xE7, 0xFE, 0x89, 0xF8, 0xE7, 0xFE, 0xB0, 0xFF, 0xE6, 0xFD
};

uint8_t COMMAND_BUFFER[MAX_COMMAND_BYTES] = {0};

command_func V_TABLE[] = {
  &cmd_version,
  &cmd_reset,
  &cmd_load,
  &cmd_cycle,
  &cmd_read_address_latch,
  &cmd_read_status,
  &cmd_read_8288_command,
  &cmd_read_8288_control,
  &cmd_read_data_bus,
  &cmd_write_data_bus,
  &cmd_finalize,
  &cmd_begin_store,
  &cmd_store,
  &cmd_queue_len,
  &cmd_queue_bytes,
  &cmd_write_pin,
  &cmd_read_pin,
  &cmd_get_program_state,
  &cmd_get_last_error,
  &cmd_get_cycle_state,
  &cmd_cycle_get_cycle_state,
  &cmd_prefetch_store,
  &cmd_read_address,
  &cmd_invalid
};

const size_t MAX_CMD = (sizeof V_TABLE / sizeof (command_func));

char LAST_ERR[MAX_ERR_LEN] = {0};

// Main Sketch setup routine
void setup() {
  SERIAL.begin(BAUD_RATE);
  Serial1.begin(DEBUG_BAUD_RATE);
  while (!SERIAL);
  while (!Serial1);

  // Set all output pins to OUTPUT
  for( int p = 0; p < (sizeof OUTPUT_PINS / sizeof OUTPUT_PINS[0]); p++ ) {
    pinMode(OUTPUT_PINS[p], OUTPUT);
  }
  // Set all input pins to INPUT
  for( int p = 0; p < (sizeof INPUT_PINS / sizeof INPUT_PINS[0]); p++ ) {
    pinMode(INPUT_PINS[p], INPUT);
  }

  // Default output pin states
  digitalWrite(RQ_PIN, HIGH); // Don't allow other bus masters
  digitalWrite(READY_PIN, HIGH);
  digitalWrite(TEST_PIN, LOW);
  digitalWrite(INTR_PIN, LOW); // Must set these to a known value or risk spurious interrupts!
  digitalWrite(NMI_PIN, LOW);  // Must set these to a known value or risk spurious interrupts!
  digitalWrite(AEN_PIN, LOW); // AEN is enable-low
  digitalWrite(CEN_PIN, HIGH); // Command enable enables the outputs on the i8288

  // Wait for CPU to initialize
  delayMicroseconds(100);

  // Patch the reset vector jump
  patch_vector(JUMP_VECTOR, LOAD_SEG);

  CPU.v_state = Reset;

  beep(100);
  Serial1.println("Arduino8088 Server Initialized!");
  clear_error();
  
  SERVER.c_state = WaitingForCommand;
}

void clear_error() {
  strncpy(LAST_ERR, "No error", MAX_ERR_LEN - 1);
}

void set_error(const char *msg) {
  strncpy(LAST_ERR, msg, MAX_ERR_LEN - 1);
  Serial1.println("");
  Serial1.println("************ ERROR ************");
  Serial1.println(LAST_ERR);
  Serial1.println("*******************************");
  error_beep();
}

// Send a failure code byte in reponse to a failed command
void send_fail() {
  #if MODE_ASCII
    SERIAL.write(RESPONSE_CHRS[RESPONSE_FAIL]);
  #else
    SERIAL.write((uint8_t)RESPONSE_FAIL);
  #endif
}

// Send the success code byte in response to a succesful command
void send_ok() {
  #if MODE_ASCII
    SERIAL.write(RESPONSE_CHRS[RESPONSE_OK]);
  #else
    SERIAL.write((uint8_t)RESPONSE_OK);
  #endif
}

void debug_proto(const char* msg) {
  #if DEBUG_PROTO
    Serial1.print("## ");
    Serial1.print(msg);
    Serial1.println(" ##");
  #endif
}

void debug_cmd(const char *cmd, const char* msg) {
  #if DEBUG_PROTO
    Serial1.print("## cmd ");
    Serial1.print(cmd);
    Serial1.print(": " );
    Serial1.print(msg);
    Serial1.println(" ##");
  #endif
}

// Server command - Version
// Send server identifier 'ard8088' followed by protocol version number in binary
bool cmd_version() {
  debug_cmd("VERSION", "In cmd");
  SERIAL.write((uint8_t *)VERSION_DAT, sizeof VERSION_DAT);
  SERIAL.write(VERSION_NUM);
  FLUSH;
  Serial1.println("Got version query!");
  return true;
}

// Server command - Reset
// Attempt to reset the CPU and report status.
// This will be rarely used by itself as the register state is not set up. The Load 
// command will reset the CPU and set register state.
bool cmd_reset() {
  debug_cmd("RESET", "In cmd");
  bool result;
  snprintf(LAST_ERR, MAX_ERR_LEN, "NO ERROR");

  #if EMULATE_8288
    reset_i8288();
  #endif
  result = cpu_reset();
  if(result) {
    change_state(Execute);
  }
  return result;
}

// Server command - Cycle
// Execute a single CPU cycle
bool cmd_cycle() {
  cycle();
  return true;
}

// Server command - Load
// Load the specified register state into the CPU.
// This command takes 28 bytes, which correspond to the word values of each of the 14
// CPU registers.
// Registers should be loaded in the following order, little-endian:
//
// AX, BX, CX, DX, SS, SP, FLAGS, IP, CS, DS, ES, BP, SI, DI
bool cmd_load() {

  //Serial1.println(">> Got load!");
  snprintf(LAST_ERR, MAX_ERR_LEN, "NO ERROR");

  // Sanity check
  if(SERVER.cmd_byte_n < sizeof LOAD_REGISTERS) {
    set_error("Not enough command bytes");
    return false;
  }

  // Write raw command bytes over register struct.
  // All possible bit representations are valid.
  uint8_t *read_p = (uint8_t *)&LOAD_REGISTERS;

  for(size_t i = 0; i < sizeof LOAD_REGISTERS; i++ ) {
    *read_p++ = COMMAND_BUFFER[i];
  }

  patch_load(&LOAD_REGISTERS, LOAD_PROGRAM);
  LOAD_REGISTERS.flags &= CPU_FLAG_DEFAULT_CLEAR;
  LOAD_REGISTERS.flags |= CPU_FLAG_DEFAULT_SET;

  #if EMULATE_8288
    reset_i8288();
  #endif
  bool result = cpu_reset();
  if(!result) {
    //set_error("Failed to reset CPU");
    return false;
  }

  #if USE_LOAD_SEG
    change_state(JumpVector);
  #else
    change_state(Load);
  #endif

  // Run CPU and wait for load to finish
  int load_timeout = 0;
  while(CPU.v_state != Execute) {
    cycle();
    load_timeout++;

    if(load_timeout > 300) {
      // Something went wrong in load program
       set_error("Load timeout");
       return false;
    }
  }

  #if LOAD_INDICATOR
    Serial1.print(".");
  #endif
  debug_proto("LOAD DONE");
  return true;
}

// Server command - ReadAddressLatch
// Read back the contents of the address latch as a sequence of 3 bytes (little-endian)
bool cmd_read_address_latch() {
  static char buf[7];

  #if MODE_ASCII  
    //buf[0] = 0;
    snprintf(
      buf, 7, 
      "%02X%02X%02X",
      (int)(CPU.address_latch & 0xFF),
      (int)((CPU.address_latch >> 8) & 0xFF),
      (int)((CPU.address_latch >> 16) & 0xFF)
    );
    SERIAL.print(buf);
  #else
    SERIAL.write((uint8_t)(CPU.address_latch & 0xFF));
    SERIAL.write((uint8_t)((CPU.address_latch >> 8) & 0xFF));
    SERIAL.write((uint8_t)((CPU.address_latch >> 16) & 0xFF));
  #endif

  return true;
}

// Server command - ReadAddress
// Read back the contents of the address bus as a sequence of 3 bytes (little-endian)
bool cmd_read_address() {
  read_address();
  static char buf[7];

  #if MODE_ASCII  
    //buf[0] = 0;
    snprintf(
      buf, 7, 
      "%02X%02X%02X",
      (int)(CPU.address_bus & 0xFF),
      (int)((CPU.address_bus >> 8) & 0xFF),
      (int)((CPU.address_bus >> 16) & 0xFF)
    );
    SERIAL.print(buf);
  #else
    SERIAL.write((uint8_t)(CPU.address_bus & 0xFF));
    SERIAL.write((uint8_t)((CPU.address_bus >> 8) & 0xFF));
    SERIAL.write((uint8_t)((CPU.address_bus >> 16) & 0xFF));
  #endif

  return true;
}

bool cmd_invalid() {
  Serial1.println("Called cmd_invalid!");
  return false;
}

// Server command - ReadStatus
// Return the value of the CPU status lines S0-S5 and QS0-QS1
bool cmd_read_status() {
  static char buf[3];
  read_status0();
  #if MODE_ASCII  
    snprintf(buf, 3, "%02X", CPU.status0);
    SERIAL.print(buf);
  #else
    SERIAL.write(CPU.status0);
  #endif
  return true;
}

// Server command - Read8288Command
bool cmd_read_8288_command() {
  static char buf[3];
  read_8288_command_bits();
  #if MODE_ASCII  
    snprintf(buf, 3, "%02X", CPU.command_bits);
    SERIAL.print(buf);
  #else
    SERIAL.write(CPU.command_bits);
  #endif
  return true;
}

// Server command - Read8288Control
bool cmd_read_8288_control() {
  static char buf[3];
  read_8288_control_bits();
  #if MODE_ASCII  
    snprintf(buf, 3, "%02X", CPU.control_bits);
    SERIAL.print(buf);
  #else
    SERIAL.write(CPU.control_bits);
  #endif
  return true;
}

// Server command - ReadDataBus
bool cmd_read_data_bus() {
  static char buf[3];
  #if MODE_ASCII  
    snprintf(buf, 3, "%02X", CPU.data_bus);
    SERIAL.print(buf);
  #else
    SERIAL.write(CPU.data_bus);
  #endif

  return true;
}

// Server command - WriteDataBus
// Takes argument of 1 byte
// Writes the specified byte to the data bus.
// This should not be called for CODE fetches after we have called cmd_prefetch_store(), 
// unless a flow control operation occurs that flushes the queue and returns us to 
// within original program boundaries.
bool cmd_write_data_bus() {
  if (CPU.bus_state_latched == CODE) {
    // We've just been instructed to write a normal fetch byte to the bus.
    // If we were prefetching the store program, reset this status as a queue
    // flush must have executed (or we goofed up...)
    CPU.prefetching_store = false;
    CPU.s_pc = 0;
  }


  CPU.data_bus = COMMAND_BUFFER[0];
  CPU.data_type = DATA_PROGRAM;
  return true;
}

// Server command - PrefetchStore
// Instructs the CPU server to load the next byte of the store program early
// Should be called in place of cmd_write_data_bus() by host on T3/TwLast when 
// program bytes have been exhausted.
// (When we are prefetching past execution boundaries during main program execution)
bool cmd_prefetch_store() {
  if (CPU.s_pc >= sizeof STORE_PROGRAM) {
    set_error("Store program underflow");
    return false;
  }
  CPU.prefetching_store = true;
  CPU.data_bus = STORE_PROGRAM[CPU.s_pc];
  // s_pc is advanced during cycle() to avoid duplicated increments
  #if DEBUG_STORE
    Serial1.print("## PREFETCH_STORE: s_pc: ");
    Serial1.print(CPU.s_pc);
    Serial1.print(" addr: ");
    Serial1.print(CPU.address_latch, 16);
    Serial1.print(" byte: ");
    Serial1.println(STORE_PROGRAM[CPU.s_pc], 16);
  #endif

  CPU.data_type = DATA_PROGRAM_END;
  return true;
}

// Server command - Finalize
// Sets the data bus flag to DATA_PROGRAM_END, so that the Execute state can terminate
// on the next instruction queue fetch
bool cmd_finalize() {
  if(CPU.v_state == Execute) {
    change_state(ExecuteFinalize);

    // Wait for execute done state
    int execute_timeout = 0;
    while(CPU.v_state != ExecuteDone) {
      cycle();
      execute_timeout++;
      if(execute_timeout > FINALIZE_TIMEOUT) {
        set_error("cmd_finalize: state timeout");
        return false;
      }
    }
    return true;
  }
  else {
    error_beep();
    set_error("cmd_finalize: wrong state: ");
    Serial1.println(CPU.v_state);
    return false;
  }
}

// Server command - BeginStore
// Execute state must be in ExecuteDone before intiating BeginStore command
//
bool cmd_begin_store(void) {
  /*
  char err_msg[30];

  // Command only valid in ExecuteDone state
  if(CPU.v_state != ExecuteDone) {
    snprintf(err_msg, 30, "BeginStore: Wrong state: %d ", CPU.v_state);
    set_error(err_msg);
    return false;
  }

  change_state(Store);
  */
  return true;
}

// Server command - Store
// 
// Returns values of registers in the following order, little-endian
// AX, BX, CX, DX, SS, SP, FLAGS, IP, CS, DS, ES, BP, SI, DI
// Execute state must be in StoreDone before executing Store command
bool cmd_store(void) {

  #if DEBUG_STORE
    Serial1.print("IN STORE: s_pc is: ");
    Serial1.println(CPU.s_pc);
  #endif

  char err_msg[30];
  // Command only valid in Store
  if(CPU.v_state != ExecuteDone) {
    snprintf(err_msg, 30, "Store: Wrong state: %d ", CPU.v_state);

    set_error(err_msg);
    return false;
  }

  change_state(Store);

  int store_timeout = 0;

  // Cycle CPU until Store complete
  while(CPU.v_state != StoreDone) {
    cycle();
    store_timeout++;

    if (store_timeout > 500) {
      Serial1.println("## STORE: Timeout! ##");
      snprintf(err_msg, 30, "StoreDone timeout.");
      error_beep();
      return false;
    }
  }

  // Dump final register state to Serial port
  uint8_t *reg_p = (uint8_t *)&CPU.post_regs;
  for(size_t i = 0; i < sizeof CPU.post_regs; i++ ) {
    SERIAL.write(reg_p[i]);
  }

  #if STORE_INDICATOR
    Serial1.print("?");
  #endif
  change_state(Done);
  return true;
}


// Server command - QueueLen
// Return the length of the instruction queue in bytes
bool cmd_queue_len(void) {

  SERIAL.write((uint8_t)CPU.queue.len);
  return true;
}

// Server command - QueueBytes
// Return the contents of the instruction queue, from 0-4 bytes.
bool cmd_queue_bytes(void) {

  for(size_t i = 0; i < CPU.queue.len; i++ ) {
    SERIAL.write(read_queue(i));
  }
  return true;
}

// Server command - Write pin
// Sets the value of the specified CPU input pin
bool cmd_write_pin(void) {

  uint8_t pin_idx = COMMAND_BUFFER[0];
  uint8_t pin_val = COMMAND_BUFFER[1] & 0x01;

  if(pin_idx < sizeof WRITE_PINS) {
    uint8_t pin_no = WRITE_PINS[pin_idx];

    switch(pin_no) {
      case READY_PIN:
        WRITE_READY_PIN(pin_val);
        break;

      case TEST_PIN:
        WRITE_TEST_PIN(pin_val);
        break;

      case INTR_PIN:
        WRITE_INTR_PIN(pin_val);
        break;

      case NMI_PIN:
        WRITE_NMI_PIN(pin_val);
        break;
      
      default:
        error_beep();
        return false;
    }
    return true;
  }
  else {
    // Invalid pin 
    error_beep();
    return false;
  }
}

// Server command - Read pin
bool cmd_read_pin(void) {
  // Not implemented
  SERIAL.write((uint8_t)0);
  return true;
}

// Server command - Get program state
bool cmd_get_program_state(void) {
  SERIAL.write((uint8_t)CPU.v_state);
  return true;
}

// Server command - Get last error
bool cmd_get_last_error(void) {
  SERIAL.write(LAST_ERR);
  return true;
}

// Server command - Get Cycle State
// A combination of all the status info typically needed for a single cycle
// Returns 4 bytes
bool cmd_get_cycle_state(void) {
  read_status0();
  read_8288_command_bits();
  read_8288_control_bits();
  uint8_t byte0 = ((uint8_t)CPU.v_state & 0x0F) << 4;
  byte0 |= (CPU.control_bits & 0x0F);

  SERIAL.write(byte0);
  SERIAL.write(CPU.status0);
  SERIAL.write(CPU.command_bits);
  #if(DATA_BUS_SIZE == 1)
    //debug_proto("Sending one byte of data bus");
    SERIAL.write(CPU.data_bus);
  #else
    SERIAL.write(uint8_t(CPU.data_bus >> 8));
    SERIAL.write(uint8_t(CPU.data_bus & 0xFF));
  #endif

  return true;
}

// Server command - Cycle and Get Cycle State
// Cycle the CPU + return A combination of all the status info typically needed for a single cycle
// Returns 4 bytes
bool cmd_cycle_get_cycle_state(void) {
  cycle();
  read_status0();
  read_8288_command_bits();
  read_8288_control_bits();
  uint8_t byte0 = ((uint8_t)CPU.v_state & 0x0F) << 4;
  byte0 |= (CPU.control_bits & 0x0F);

  SERIAL.write(byte0);
  SERIAL.write(CPU.status0);
  SERIAL.write(CPU.command_bits);
  #if(DATA_BUS_SIZE == 1)
    debug_proto("writing one byte of data bus");
    SERIAL.write(CPU.data_bus);
  #else
    SERIAL.write(uint8_t(CPU.data_bus >> 8));
    SERIAL.write(uint8_t(CPU.data_bus & 0xFF));
  #endif

  return true;
}

void patch_vector(uint8_t *vec, uint16_t seg) {
  *((uint16_t *)&vec[3]) = seg;
}

void patch_load(registers *reg, uint8_t *prog) {
  *((uint16_t *)prog) = reg->flags;
  *((uint16_t *)&prog[LOAD_BX]) = reg->bx;
  *((uint16_t *)&prog[LOAD_CX]) = reg->cx;
  *((uint16_t *)&prog[LOAD_DX]) = reg->dx;
  *((uint16_t *)&prog[LOAD_SS]) = reg->ss;
  *((uint16_t *)&prog[LOAD_DS]) = reg->ds;
  *((uint16_t *)&prog[LOAD_ES]) = reg->es;
  *((uint16_t *)&prog[LOAD_SP]) = reg->sp;
  *((uint16_t *)&prog[LOAD_BP]) = reg->bp;
  *((uint16_t *)&prog[LOAD_SI]) = reg->si;
  *((uint16_t *)&prog[LOAD_DI]) = reg->di;
  *((uint16_t *)&prog[LOAD_AX]) = reg->ax;
  *((uint16_t *)&prog[LOAD_IP]) = reg->ip;
  *((uint16_t *)&prog[LOAD_CS]) = reg->cs;
}

void print_registers(registers *regs) {
  static char buf[130];
  static char flag_buf[17];

  if(!regs) {
    return;
  }

  snprintf(buf, 130, 
    "AX: %04x BX: %04x CX: %04x DX: %04x\n"
    "SP: %04x BP: %04x SI: %04x DI: %04x\n"
    "CS: %04x DS: %04x ES: %04x SS: %04x\n"
    "IP: %04x\n"
    "FLAGS: %04x",
    regs->ax, regs->bx, regs->cx, regs->dx,
    regs->sp, regs->bp, regs->si, regs->di,
    regs->cs, regs->ds, regs->es, regs->ss,
    regs->ip,
    regs->flags );

  Serial1.println(buf);

  // Expand flag info
  uint16_t f = regs->flags;
  char c_chr = CPU_FLAG_CARRY & f ? 'C' : 'c';
  char p_chr = CPU_FLAG_PARITY & f ? 'P' : 'p';
  char a_chr = CPU_FLAG_AUX_CARRY & f ? 'A' : 'a';
  char z_chr = CPU_FLAG_ZERO & f ? 'Z' : 'z';
  char s_chr = CPU_FLAG_SIGN & f ? 'S' : 's';
  char t_chr = CPU_FLAG_TRAP & f ? 'T' : 't';
  char i_chr = CPU_FLAG_INT_ENABLE & f ? 'I' : 'i';
  char d_chr = CPU_FLAG_DIRECTION & f ? 'D' : 'd';
  char o_chr = CPU_FLAG_OVERFLOW & f ? 'O' : 'o';
  
  snprintf(
    flag_buf, 17,
    "1111%c%c%c%c%c%c0%c0%c1%c",
    o_chr, d_chr, i_chr, t_chr, s_chr, z_chr, a_chr, p_chr, c_chr
  );

  Serial1.print("FLAGSINFO: ");
  Serial1.println(flag_buf);
}

void print_cpu_state() {
  
  static char buf[81];
  const size_t op_len = (4 + (DATA_BUS_SIZE * 2) + 1);
  static char op_buf[op_len];
  static char q_buf[15];
  
  const char *ale_str = READ_ALE_PIN ? "A:" : "  ";
  
  char rs_chr = !READ_MRDC_PIN ? 'R' : '.';
  char aws_chr = !READ_AMWC_PIN ? 'A' : '.';
  char ws_chr = !READ_MWTC_PIN ? 'W' : '.';
  
  char ior_chr = !READ_IORC_PIN ? 'R' : '.';
  char aiow_chr = !READ_AIOWC_PIN ? 'A' : '.';
  char iow_chr = !READ_IOWC_PIN ? 'W' : '.';

  char v_chr = MACHINE_STATE_CHARS[(size_t)CPU.v_state];
  uint8_t q = (CPU.status0 >> 6) & 0x03;
  char q_char = QUEUE_STATUS_CHARS[q];
  char s = CPU.status0 & 0x07;

  // Get segment from S3 & S4
  const char *seg_str = "  ";
  if(CPU.bus_cycle != T1) {
    // Status is not avaialble on T1 because address is latched
    uint8_t seg = ((CPU.status0 & 0x18) >> 3) & 0x03;
    seg_str = SEGMENT_STRINGS[(size_t)seg];
  }

  // Draw some sad ascii representation of bus transfers
  char *st_str = "  ";
  switch(CPU.bus_cycle) {
      case T1:
        if((CPU.bus_state != PASV) && (CPU.bus_state != HALT) && (CPU.bus_state != IRQA)) {
          // Begin a bus state
          st_str = "\\ ";
        }
        break;
      case T2: // FALLTHRU
      case T3: // FALLTHRU
      case TW:  
        // Continue a bus state
        st_str = " |";
        break;
      case T4:
        // End a bus state
        st_str = "/ ";
        break;
  }

  // Make string for bus reads and writes
  op_buf[0] = 0;
  if ((!READ_MRDC_PIN || !READ_IORC_PIN) && CPU.bus_state == PASV) {
    snprintf(op_buf, op_len, "<-r %0*X", DATA_BUS_SIZE * 2, data_bus_peek());
  }
  if(!READ_MWTC_PIN || !READ_IOWC_PIN) {
    snprintf(op_buf, op_len, "w-> %0*X", DATA_BUS_SIZE * 2, CPU.data_bus);
  }

  const char *q_str = queue_to_string();

  const char *t_str;
  if ((CPU.bus_cycle == T1) && (CPU.bus_state == PASV)) {
    // Convert T1 to Ti when passive bus
    t_str = "Ti";
  }
  else {
    t_str = CYCLE_STRINGS[(size_t)CPU.bus_cycle];
  }

  snprintf(
    buf, 81, 
    "%08ld %c %s[%05lX] %2s M:%c%c%c I:%c%c%c %-4s %s %2s %-*s | %c%d [%-*s]", 
    CYCLE_NUM, 
    v_chr, 
    ale_str,
    //read_address(), 
    CPU.address_latch,
    seg_str,
    rs_chr, aws_chr, ws_chr,
    ior_chr, aiow_chr, iow_chr,
    BUS_STATE_STRINGS[(size_t)CPU.bus_state],
    t_str, 
    st_str,
    4 + (DATA_BUS_SIZE * 2),
    op_buf,
    q_char,
    CPU.queue.len,
    QUEUE_MAX * 2,
    q_str
  );

  Serial1.print(buf);

  if(q == QUEUE_FIRST) {
    // First byte of opcode read from queue. Decode it to opcode
    snprintf(q_buf, 15, " <-q %02X %s", CPU.qb, get_opcode_str(CPU.opcode, 0, false));
    Serial1.print(q_buf);
  }
  else if(q == QUEUE_SUBSEQUENT) {
    if(IS_GRP_OP(CPU.opcode) && CPU.q_fn == 1) {
      // Modrm was just fetched for a group opcode, so display the mnemonic now
      snprintf(q_buf, 15, " <-q %02X %s", CPU.qb, get_opcode_str(CPU.opcode, CPU.qb, true));
    }
    else {
      snprintf(q_buf, 15, " <-q %02X", CPU.qb);
    }
    Serial1.print(q_buf);
  }

  Serial1.println("");
}

void change_state(machine_state new_state) {
  switch(new_state) {
    case Reset:
      CPU.doing_reset = true;    
      CPU.v_pc = 0;
      CPU.s_pc = 0;
      break;
    case JumpVector:
      CPU.doing_reset = false;
      CPU.v_pc = 0;
      break;
    case Load:
      // Set v_pc to 2 to skip flag bytes
      CPU.v_pc = 2;
      break;
    case LoadDone:
      break;
    case Execute:
      CPU.v_pc = 0;
      CPU.s_pc = 0;
      break;
    case ExecuteFinalize:
      break;
    case ExecuteDone:
      break;
    case Store:
      // Take a raw uint8_t pointer to the register struct. Both x86 and Arduino are little-endian,
      // so we can write raw incoming data over the struct. Faster than logic required to set
      // specific members. 
      CPU.readback_p = (uint8_t *)&CPU.post_regs;      
      break;
    case StoreDone:
      break;
    case Done:
      break;
  }

  uint32_t state_end_time = micros();

  #if TRACE_STATE
    // Report time we spent in the previous state.
    if(CPU.state_begin_time != 0) {
      uint32_t elapsed = state_end_time - CPU.state_begin_time;
      Serial1.print("## Changing to state: ");
      Serial1.print(MACHINE_STATE_STRINGS[(size_t)new_state]);
      Serial1.print(". Spent (");
      Serial1.print(elapsed);
      Serial1.println(") us in previous state. ##");
    }
  #endif

  CPU.state_begin_time = micros();
  CPU.v_state = new_state;
}

void cycle() {

  // First, tick the CPU and increment cycle count
  clock_tick();
  CYCLE_NUM++;
  if(CYCLE_NUM == 0) {
    // overflow
    CYCLE_NUM_H++;
  }

  // Read the CPU status pins
  read_status0();

  // bus_state is the instantaneous state per cycle. May not always be valid. 
  // for state of the current bus transfer use bus_state_latched
  CPU.bus_state = (s_state)(CPU.status0 & 0x07);

  // Extract QS0-QS1 queue status
  uint8_t q = (CPU.status0 >> 6) & 0x03;
  CPU.qb = 0xFF;
  CPU.q_ff = false;

  if(READ_ALE_PIN) {
    // ALE signals start of bus cycle, so set cycle to t1.
    CPU.bus_cycle = T1;
    // Address lines are only valid when ALE is high, so latch address now.
    latch_address();
    CPU.bus_state_latched = CPU.bus_state; 
    //Serial1.print("## LATCHED ADDRESS: ");
    //Serial1.println(CPU.address_latch);
  }

  // Operate current T-state
  switch(CPU.bus_cycle) {
    case T1:
      break;

    case T2:
      break;

    case T3:
      break;

    case T4:
      // Did we complete a code fetch? If so, increment queue len
      if(CPU.bus_state_latched == CODE) {
        //Serial1.print("## T4 of CODE fetch. Q is: ");
        //Serial1.println(q);

        if(q == QUEUE_FLUSHED) {
          Serial1.println("## Queue flush during T4. Supressing queue push.");
          if(CPU.queue.len < QUEUE_MAX) {
            push_queue(CPU.data_bus, CPU.data_type);
          }
          else {
            // Shouldn't be here
            Serial1.println("## Error: Invalid Queue Length++ ##");
          }
        }
        else {
          if(CPU.queue.len < QUEUE_MAX) {
            push_queue(CPU.data_bus, CPU.data_type);
          }
          else {
            // Shouldn't be here
            Serial1.println("## Error: Invalid Queue Length++ ##");
          }
        }
      }
      CPU.bus_state_latched = PASV;
      break;
  }


  // Handle queue activity
  if((q == QUEUE_FIRST) || (q == QUEUE_SUBSEQUENT)) {
    // We fetched a byte from queue last cycle
    if(CPU.queue.len > 0 ) {
      pop_queue(&CPU.qb, &CPU.qt);
      if(q == QUEUE_FIRST) {
        // Set flag for first instruction byte fetched
        CPU.q_ff = true;
        CPU.q_fn = 0; // First byte of instruction
        CPU.opcode = CPU.qb;
        CPU.mnemonic = get_opcode_str(CPU.opcode, 0, false);
        #if DEBUG_INSTR
          if (!IS_GRP_OP(CPU.opcode)) {
            Serial1.print("INST: ");
            Serial1.println(CPU.mnemonic);
          }
          else {
            Serial1.println("INST: Decoding GRP...");
          }
        #endif
      }
      else {
        if(IS_GRP_OP(CPU.opcode) && CPU.q_fn == 1) {
          CPU.mnemonic = get_opcode_str(CPU.opcode, CPU.qb, true);
          #if DEBUG_INSTR 
            Serial1.print("INST: ");
            Serial1.println(CPU.mnemonic);
          #endif
        }
        // Subsequent byte of instruction fetched
        CPU.q_fn++;
      }
    }
    else {
      Serial1.println("## Error: Invalid Queue Length-- ##");
    }
  }
  else if(q == QUEUE_FLUSHED) {
    // Queue was flushed last cycle.

    // Warn if queue is flushed during CODE cycle.
    if (CPU.bus_state_latched == CODE) {
        Serial1.print("## FLUSH during CODE fetch! t-state: ");
        switch (CPU.bus_cycle) {
          case T1:
            Serial1.println("T1");
            break;
          case T2:
            Serial1.println("T2");
            break;
          case T3:
            Serial1.println("T3");
            break;
          case T4:
            Serial1.println("T4");
            break;
        }
    }

    // The queue is flushed once during store program, so we need to adjust s_pc 
    // by the length of the queue when it was flushed or else we'll skip bytes
    // of the store program.
    if (CPU.s_pc > 0) {

      if (CPU.s_pc < 4) {
        #if DEBUG_STORE
          Serial1.println("## FLUSHed STORE bytes (early): Reset s_pc");
        #endif
        CPU.s_pc = 0;
      }
      else if (CPU.s_pc >= CPU.queue.len) {
        CPU.s_pc -= CPU.queue.len;
        #if DEBUG_STORE
          Serial1.print("## FLUSHed STORE bytes: Adjusted s_pc by queue_len: ");
          Serial1.print(CPU.queue.len);
          Serial1.print(" new s_pc: ");
          Serial1.println(CPU.s_pc);
        #endif
      }
      else {
        #if DEBUG_STORE
          Serial1.print("## FLUSHed STORE bytes: Reset s_pc on flush");
        #endif
        CPU.s_pc = 0;
      }
    }

    empty_queue();

    #if TRACE_QUEUE
      SERIAL.println("## Queue Flushed ##");
      SERIAL.print("## PC: ");
      SERIAL.println(CPU.v_pc);
    #endif
  }

  switch(CPU.v_state) {

    case JumpVector:
      // We are executing the initial jump from the reset vector FFFF:0000.
      // This is to avoid wrapping effective address during load procedure.
      // Optional - disable in header

      if(!READ_MRDC_PIN) {
        // CPU is reading (MRDC active-low)      
        if(CPU.bus_state == CODE) {    
          // We are reading a code byte
          if(CPU.v_pc < sizeof JUMP_VECTOR) {
            // Feed jump instruction to CPU
            CPU.data_bus = JUMP_VECTOR[CPU.v_pc];
            CPU.data_type = DATA_PROGRAM;
            CPU.v_pc++;
          }
          else {
            // Ran out of program, so return NOP. Doesn't matter what we feed
            // as queue will be reset.
            CPU.data_bus = OPCODE_NOP;
            CPU.data_type = DATA_PROGRAM_END;
          }
          data_bus_write(CPU.data_bus);
        }
      }        

      if(READ_ALE_PIN) {
        // Jump is finished on first address latch of LOAD_SEG:0
        uint32_t dest = calc_flat_address(LOAD_SEG, 0);
        if(dest == CPU.address_latch) {
          change_state(Load);
          break;
        }
      }      
      break;

    case Load:
      // We are executing the register load routine.

      if(!READ_MRDC_PIN) {
        // CPU is reading (MRDC active-low)
        if(CPU.bus_state == CODE) {      
          // We are reading a code byte
          if(CPU.v_pc < sizeof LOAD_PROGRAM) {
            // Feed load program to CPU

            #if (DATA_BUS_SIZE == 1)
              CPU.data_bus = LOAD_PROGRAM[CPU.v_pc++];
            #else
              CPU.data_bus = LOAD_PROGRAM[CPU.v_pc++] << 8;
              CPU.data_bus = LOAD_PROGRAM[CPU.v_pc++];
            #endif

            CPU.data_type = DATA_PROGRAM;
          }
          else {
            // Ran out of program, so return NOP. JMP cs:ip will actually fetch once before SUSP,
            // so we wil see this NOP prefetched.
            #if (DATA_BUS_SIZE == 1)
              CPU.data_bus = OPCODE_NOP;
            #else
              CPU.data_bus = OPCODE_DOUBLENOP;
            #endif

            CPU.data_type = DATA_PROGRAM_END;
            //change_state(LoadDone);
          }
          data_bus_write(CPU.data_bus);
        }
        
        if(CPU.bus_state == MEMR) {
          // We are reading a memory byte
          // This should only occur during Setup when flags are popped from 0:0
          if(CPU.address_latch < 0x00002 ) {
            // First two bytes of LOAD_PROGRAM were patched with flags
            CPU.data_bus = LOAD_PROGRAM[CPU.address_latch];
            CPU.data_type = DATA_PROGRAM;
            data_bus_write(CPU.data_bus);
            
          }
          else {
            // Unexpected read above address 0x00001
            Serial1.println("## INVALID MEM READ DURING LOAD ##");
          }
        }
      } 

      if (q == QUEUE_FLUSHED) {
        // Queue flush after final jump triggers next state.
        change_state(LoadDone);
      }
      break;

    case LoadDone:
      // LoadDone is triggered by the queue flush following the jump in Load.
      // We wait for the next ALE and begin Execute.
      if(READ_ALE_PIN) {
        // First bus cycle of the instruction to execute. Transition to Execute.
        change_state(Execute);
      }
      break;

    // Unlike in run_program, the Execute state in cpu_server is entirely interactive based on 
    // commands from the client. 
    // This is to support interception of memory reads & writes as instructions execute and to allow
    // the client to query CPU state as it wishes per cpu cycle.
    // When done in the Execute state, a cpu client should execute the ExecuteFinalize command.
    // This is typically done when a CODE fetch occurs past the end of the provided program, although
    // other end conditions are possible.
    case Execute:
    
      if ((!READ_MRDC_PIN || !READ_IORC_PIN) && CPU.bus_state == PASV) {
        // CPU is reading from data bus. We assume that the client has called CmdWriteDataBus to set 
        // the value of CPU.data_bus. Write it.
        data_bus_write(CPU.data_bus);

        if ((CPU.bus_state_latched == CODE) && (CPU.prefetching_store)) {
          CPU.s_pc++;
          #if DEBUG_STORE
            Serial1.print("STORE: Wrote STORE PGM BYTE to bus: ");
            Serial1.print(CPU.data_bus, 16);
            Serial1.print(" new s_pc: ");
            Serial1.println(CPU.s_pc);
          #endif
        }
      }
    
      if(!READ_MWTC_PIN || !READ_IOWC_PIN) {
        // CPU is writing to the data bus, latch value
        CPU.data_bus = data_bus_read();
      }
      
      break;

    // The ExecuteFinalize state is unique to the cpu_server. Since Execute is now an interactive state,
    // we need to be able to transition safely from Execute to Store. ExecuteState feeds the CPU
    // STORE program bytes flagged with DATA_PROGRAM_END and transitions to Store when one of those bytes
    // is fetched as the first byte of an instruction.
    case ExecuteFinalize:
      
      if (!READ_MRDC_PIN && CPU.bus_state == PASV) {
        // CPU is reading (MRDC active-low)
        if ((CPU.bus_state_latched == CODE) && (CPU.prefetching_store)) {
          // Since client does not cycle the CPU in this state, we have to fetch from 
          // STORE program ourselves
          CPU.data_bus = STORE_PROGRAM[CPU.s_pc++];
          CPU.data_type = DATA_PROGRAM_END;
          data_bus_write(CPU.data_bus);
          #if DEBUG_STORE
            Serial1.print("STORE: Wrote STORE PGM BYTE to bus (in FINALIZE): ");
            Serial1.print(CPU.data_bus, 16);
            Serial1.print(" new s_pc: ");
            Serial1.println(CPU.s_pc);
          #endif
        }
        else {
          data_bus_write(CPU.data_bus);
        }
      }

      if(CPU.q_ff && (CPU.qt == DATA_PROGRAM_END)) {
        // We read a flagged NOP, meaning the previous instruction has completed and it is safe to 
        // execute the Store routine.
        change_state(ExecuteDone);
      }
      break;

    case ExecuteDone:
      // We sit in ExecuteDone state until the client requests a Store operation.
      // The client should not cycle the CPU in this state.
      if (!READ_MRDC_PIN && CPU.bus_state == PASV) {
        // CPU is reading (MRDC active-low)
        data_bus_write(CPU.data_bus);

        if ((CPU.bus_state_latched == CODE) && (CPU.prefetching_store)) {
          // Since client does not cycle the CPU in this state, we have to fetch from 
          // STORE program ourselves          
          CPU.data_bus = STORE_PROGRAM[CPU.s_pc++];
          CPU.data_type = DATA_PROGRAM_END;
          data_bus_write(CPU.data_bus);
          #if DEBUG_STORE
            Serial1.print("STORE: Wrote STORE PGM BYTE to bus (in EXECUTE_DONE): ");
            Serial1.print(CPU.data_bus, 16);
            Serial1.print(" new s_pc: ");
            Serial1.println(CPU.s_pc);
          #endif
        }
        else {
          Serial1.println("## Invalid condition: ExecuteDone without loading STORE");
          data_bus_write(CPU.data_bus);
        }
      }      
      break;

    case Store:
      // We are executing the Store program.

      //set_error("We are in Store");
      
      if (!READ_MRDC_PIN && CPU.bus_state == PASV) {
        // CPU is reading
        
        if (CPU.bus_state_latched == CODE) {
          // CPU is doing code fetch
          if(CPU.s_pc < sizeof STORE_PROGRAM) {
            // Read code byte from store program
            CPU.data_bus = STORE_PROGRAM[CPU.s_pc++];
            #if DEBUG_STORE
              Serial1.print("STORE: fetching byte: ");
              Serial1.print(CPU.data_bus, 16);
              Serial1.print(" new s_pc: ");
              Serial1.println(CPU.s_pc);
            #endif
            CPU.data_type = DATA_PROGRAM;

          }
          else {
            CPU.data_bus = 0x90;
            CPU.data_type = DATA_PROGRAM_END;
          }
        }
        data_bus_write(CPU.data_bus);
      }

      // CPU is writing to memory address - this should only happen during readback when
      // the flags register is pushed to the stack (The only way to read the full flags)      
      if(!READ_MWTC_PIN) {
        CPU.data_bus = data_bus_read();

        // Store program sets up SS:SP as 0:4, so write should be to the first four memory
        // addresses, for pushing IP and FLAGS.
        if(CPU.address_latch < 0x00004) {

          #if DEBUG_STORE
            Serial1.println("## Store Stack Push");
          #endif
          *CPU.readback_p = CPU.data_bus;
          CPU.readback_p++;
        }
        else {
          // We shouldn't be writing to any other addresses, something wrong happened
          if (CPU.address_latch == 0x00004) {
            Serial1.println("## TRAP detected in Store operation! Invalid flags?");
          }

          Serial1.println("## INVALID STORE WRITE: ");
          Serial1.println(CPU.address_latch, HEX);
          set_error("Invalid store write");
          // TODO: handle error gracefully
        }
        #if DEBUG_STORE
          Serial1.print("## Store memory write: ");
          Serial1.println(CPU.data_bus, HEX);
        #endif
      }

      // CPU is writing to IO address - this indicates we are saving a register value. 
      // We structured the register struct in the right order, so we can overwrite it
      // with raw uint8_ts.
      if(!READ_IOWC_PIN) {

        if(CPU.address_latch == 0xFD) {
          // Write to 0xFD indicates end of store procedure.
          
          // Adjust IP by offset of CALL instruction.
          #if DEBUG_STORE
            Serial1.print("## Unadjusted IP: ");
            Serial1.println(CPU.post_regs.ip, HEX);
          #endif            
          //CPU.post_regs.ip -= 0x24;
          CPU.post_regs.ip -= (0x24 + 4); // added 4 NOPs to start of STORE program
          
          change_state(StoreDone);
        }
        else {
          CPU.data_bus = data_bus_read();

          *CPU.readback_p = CPU.data_bus;
          CPU.readback_p++;

          #if DEBUG_STORE
            Serial1.print("## Store IO write: ");
            Serial1.println(CPU.data_bus, HEX);
          #endif
        }
      }

    /*
    case Done:
      if(!READ_MRDC_PIN) {
        // CPU is reading
        
        if(CPU.bus_state == CODE) {
          // CPU is doing code fetch
          CPU.data_bus = 0x90;
          CPU.data_type = DATA_PROGRAM_END;
          data_bus_write(CPU.data_bus);
        }
        else {
          Serial1.print("**Unexpected read in DONE**");
        }        
      }    
      break;
      */

    break;
  }

  // Print instruction state if tracing is enabled
  switch(CPU.v_state) {
    case Reset:
      #if TRACE_RESET
        print_cpu_state();
      #endif
      break;
    case JumpVector:
      #if TRACE_VECTOR
        print_cpu_state();
      #endif
      break;
    case Load: // FALLTHROUGH
    case LoadDone:
      #if TRACE_LOAD
        print_cpu_state();
      #endif  
      break;
    case Execute:
      #if TRACE_EXECUTE
        print_cpu_state();
      #endif 
      break;
    case ExecuteDone: // FALLTHROUGH
    case ExecuteFinalize:
      #if TRACE_FINALIZE
        print_cpu_state();
      #endif
      break;
    case Store:
      #if TRACE_STORE
        print_cpu_state();  
      #endif
      break;
  }

  // Transition to next T-state.
  switch(CPU.bus_cycle) {
    case T1:
      // Begin a bus cycle only if signalled, otherwise wait in T1
      if(CPU.bus_state != PASV) {
        CPU.bus_cycle = T2;
      }
      break;

    case T2:
      CPU.bus_cycle = T3;
      break;

    case T3:
      // TODO: Handle wait states between t3 & t4
      CPU.bus_cycle = T4;
      break;

    case T4:
      CPU.bus_cycle = T1;
      CPU.bus_state_latched = PASV;
      break;
  }


}

void print_addr(unsigned long addr) {
  static char addr_buf[6];
  snprintf(addr_buf, 6, "%05lX", addr);
  Serial1.println(addr_buf);
}

// Main sketch loop 
void loop() {
  
  switch(SERVER.c_state) {

    case WaitingForCommand:
      if(SERIAL.available() > 0) {
        uint8_t cmd_byte = SERIAL.read();

        debug_cmd(CMD_STRINGS[cmd_byte], "received!");

        bool got_command = false;
        if(cmd_byte >= (uint8_t)CmdInvalid) {
          // Command is out of range, check against alias list
          for( uint8_t a = 0; a < sizeof CMD_ALIASES; a++) {
            if(cmd_byte == CMD_ALIASES[a]) {
              /*
              SERIAL.print("Got command alias: ");
              SERIAL.println(cmd_byte, HEX);
              */
              cmd_byte = a;
              got_command = true;
              break;
            }
          }

          if(!got_command) {
            send_fail();
            break;
          }
        }
        
        // Valid command, enter ReadingCommand state
        SERVER.cmd = (server_command)cmd_byte;

        if(cmd_byte == 0) {
          // We ignore command byte 0 (null command)
          break;
        }
        else if (cmd_byte > MAX_CMD) {
          // Cmd is out of range
          debug_proto("Command out of range!");
          break;
        }
        else if(CMD_INPUTS[cmd_byte] > 0) {
          // This command requires input bytes before it is executed.
          SERVER.cmd = (server_command)cmd_byte;
          SERVER.cmd_byte_n = 0;
          SERVER.c_state = ReadingCommand;
          SERVER.cmd_bytes_expected = CMD_INPUTS[cmd_byte];
          SERVER.cmd_start_time = millis(); // Get start time for timeout calculation
        }
        else {
          // Command requires no input, execute immediately
          bool result = V_TABLE[cmd_byte - 1]();
          if(result) {  
            debug_proto("Command OK!");
            send_ok();
          }
          else {
            debug_proto("Command FAIL!");
            send_fail();
          }
        }
      }
      break;

    case ReadingCommand:
      // The previously specified command requires paramater bytes, so read them in, or timeout
      if(SERIAL.available() > 0) {
        uint8_t cmd_byte = SERIAL.read();
        
        if(SERVER.cmd_byte_n < MAX_COMMAND_BYTES) {
          // Stil have bytes yet to read
          COMMAND_BUFFER[SERVER.cmd_byte_n] = cmd_byte;
          SERVER.cmd_byte_n++;
 
          if(SERVER.cmd_byte_n == SERVER.cmd_bytes_expected) {
            // We have received enough parameter bytes to execute the in-progress command.
            bool result = V_TABLE[SERVER.cmd - 1]();
            if(result) {  
              send_ok();
            }
            else {
              send_fail();
            }

            // Revert to listening for command
            SERVER.cmd_byte_n = 0;
            SERVER.cmd_bytes_expected = 0;
            SERVER.c_state = WaitingForCommand;
          }
        }
      }
      else {
        // No bytes received yet, so keep track of how long we've been waiting
        uint32_t now = millis();
        uint32_t elapsed = now - SERVER.cmd_start_time;

        if(elapsed >= CMD_TIMEOUT) {
          // Timed out waiting for parameter bytes. Send failure and revert to listening for command
          SERVER.cmd_byte_n = 0;
          SERVER.cmd_bytes_expected = 0;          
          SERVER.c_state = WaitingForCommand;
          debug_proto("Command timeout!");
          send_fail();
        }
      }
      break;
  }
}
