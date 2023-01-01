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

#include "arduino8088.h"

u32 calc_flat_address(u16 seg, u16 offset) {
  return ((u32)seg << 4) + offset;
}

// -------------------------- CPU Interface -----------------------------------

// Execute one clock pulse to the CPU
void clock_tick() {
  SET_CLOCK_HIGH;
  delayMicroseconds(CLOCK_PIN_HIGH_DELAY);
  SET_CLOCK_LOW;
  delayMicroseconds(CLOCK_PIN_LOW_DELAY);
}

// Write a value to the CPU's data bus
void data_bus_write(u8 byte) {
  // Set data bus pins 22-29 to OUTPUT
  DDRA = 0xFF;
  delayMicroseconds(PIN_CHANGE_DELAY);
  // Write byte to data bus pins 22-29
  PORTA = byte;
  delayMicroseconds(PIN_CHANGE_DELAY);
}

// Read a value from the CPU's data bus
u8 data_bus_read() {
  // Set data bus pins 22-29 to INPUT
  DDRA = 0;
  delayMicroseconds(PIN_CHANGE_DELAY);
  // Read byte from data bus pins 22-29
  return PINA;
}

/*
  Read the address pins and store the 20-bit value in global ADDRESS_LATCH.
  Only valid while ALE is HIGH.
*/
void latch_address() {

  // Set data bus pins 22-29 to INPUT
  DDRA = 0;
  delayMicroseconds(PIN_CHANGE_DELAY); // Wait for pin state change before reading
  CPU.address_latch = PINA; // Pins 22-29
  CPU.address_latch |= (unsigned long)BIT_REVERSE_TABLE[PINC] << 8; // Pins 30-37 (Bit order reversed)
  CPU.address_latch |= (unsigned long)(PIND & 0x80) << 9; // Pin 38
  CPU.address_latch |= (unsigned long)(BIT_REVERSE_TABLE[PING] & 0xE0) << 12; // Pins 39-40 (Bit order reversed)
}

// Read the status lines S0-S5 as well as queue status lines QS0-QS1.
void read_status0() {

  CPU.status0 = (PINJ & BIT1) >> 1;         // S0
  CPU.status0 |= (PINJ & BIT0) << 1;        // S1 
  CPU.status0 |= ((PINH & BIT1) >> 1) << 2; // S2 - Pin 16 (H1)
  CPU.status0 |= ((PIND & BIT7) >> 7) << 3; // S3 - Pin 38 (D7)
  CPU.status0 |= ((PING & BIT2) >> 2) << 4; // S4 - Pin 39 (G2)
  CPU.status0 |= ((PING & BIT1) >> 1) << 5; // S5 - Pin 40 (G1)
  CPU.status0 |= ((PINH & BIT6) >> 6) << 6; // QS0 - Pin 9 (H6)
  CPU.status0 |= ((PINH & BIT5) >> 5) << 7; // QS1 - Pin 8 (H5)
}

// Read the i8288 command lines
void read_8288_command_bits() {
  
  CPU.command_bits = ((PINB & BIT2) >> 2);       // MRDC - Pin 51 (B2)
  CPU.command_bits |= ((PINB & BIT1) >> 1) << 1; // AMWC - Pin 52 (B1)
  CPU.command_bits |= ((PINB & BIT0)) << 2;      // MWTC - Pin 53 (B0)
  CPU.command_bits |= ((PINL & BIT3) >> 3) << 3; // IORC - Pin 46 (L3)
  CPU.command_bits |= ((PINL & BIT1) >> 1) << 4; // AIOWC- Pin 48 (L1)
  CPU.command_bits |= ((PINL & BIT2) >> 2) << 5; // IOWC - Pin 47 (L2)
  CPU.command_bits |= ((PINL & PIN4) >> 4) << 6; // INTA - Pin 45 (L4)
  // Although not technically a command status, we have an extra bit, so we can stick ALE in here.
  CPU.command_bits |= ((PINB & BIT3) >> 3) << 7; // ALE  - Pin 50 (B3)
}

// Read the i8288 control lines
void read_8288_control_bits() {
  
  CPU.control_bits = (PINB & BIT3) >> 3;         // ALE      - Pin 50 (B3)
  CPU.control_bits |= (PINL & BIT0) << 1;        // DTR      - Pin 49 (L0)
  CPU.control_bits |= ((PINL & BIT6) >> 6) << 2; // MCE/PDEN - Pin 43 (L6)
  CPU.control_bits |= ((PINL & BIT5) >> 5) << 3; // DEN      - Pin 44 (L5)
}

// Resets the CPU by asserting RESET line for at least 4 cycles and waits for ALE signal.
bool cpu_reset() {

  CYCLE_NUM_H = 0;
  CYCLE_NUM = 0;
  bool ale_went_off = false;
  CPU.state_begin_time = 0;
  change_state(Reset);
  CPU.data_bus = 0x00; 
  init_queue();

  // Hold RESET high for 4 cycles
  SET_RESET_HIGH;

  for (int i = 0; i < RESET_HOLD_CYCLE_COUNT; i++) {

    if(READ_ALE_PIN == false) {
      ale_went_off = true;
    }
    clock_tick();
  }

  // CPU didn't reset for some reason.
  if(ale_went_off == false) {
    return false;
  }

  SET_RESET_LOW;

  // Clock CPU while waiting for ALE
  int ale_cycles = 0;

  // Reset takes 7 cycles, bit we can try for longer
  for( int i = 0; i < RESET_CYCLE_TIMEOUT; i++ ) {
    cycle();
    ale_cycles++;      

    if(READ_ALE_PIN) {
      // ALE is active! CPU has successfully reset
      CPU.doing_reset = false;
      return true;
    }
  }

  // ALE did not turn on within the specified cycle timeout, so we failed to reset the  cpu.
  return false;
}

// ---------------------- Processor Instruction Queue -------------------------
void init_queue() {
  CPU.queue.len = 0;
  CPU.queue.back = 0;
  CPU.queue.front = 0;
}

void push_queue(u8 byte, u8 dtype) {
  if(CPU.queue.len < QUEUE_MAX) {
    CPU.queue.queue[CPU.queue.front] = byte;
    CPU.queue.types[CPU.queue.front] = dtype;
    CPU.queue.front = (CPU.queue.front + 1) & 0x03;
    CPU.queue.len++;
  }
}

void pop_queue(u8 *byte, u8 *dtype) {
  if(CPU.queue.len > 0) {
    *byte = CPU.queue.queue[CPU.queue.back];
    *dtype = CPU.queue.types[CPU.queue.back];
    CPU.queue.back = (CPU.queue.back + 1) & 0x03;
    CPU.queue.len--;
    return byte;
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
  u8 i;
  u8 byte;
  for(i = 0; i < CPU.queue.len; i++ ) {
    byte = CPU.queue.queue[(CPU.queue.back + i) % QUEUE_MAX];
    sprintf(hex, "%02X", byte);
    strcat(buf, hex);
  }
  Serial.println(buf);
}

u8 read_queue(size_t idx) {
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
  u8 byte;
  for(u8 i = 0; i < CPU.queue.len; i++ ) {
    byte = CPU.queue.queue[(CPU.queue.back + i) % QUEUE_MAX];
    snprintf(buf_p, buf_len - (i * 2), "%02X", byte);
    buf_p += 2;
  }  

  return buf;
}

void test_queue() {

  u8 i;
  for(i = 0; i < 4; i++ ) {
    push_queue(i, 0);
  }

  print_queue();

  u8 qt, qb;

  pop_queue(&qt, &qb);
  pop_queue(&qt, &qb);
  push_queue(5, 0);

  print_queue();
}

// ----------------------------------Buzzer------------------------------------
void beep(u32 time) {
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

// ----------------------------------Opcodes-----------------------------------

// Return the mnemonic name for the specified opcode. If the opcode is a group
// opcode, op2 should be specified and modrm set to true.
const char *get_opcode_str(u8 op1, u8 op2, bool modrm) {

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