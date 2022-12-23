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
    byte = CPU.queue.queue[(CPU.queue.back + i) & 0x03];
    sprintf(hex, "%02X", byte);
    strcat(buf, hex);
  }
  Serial.println(buf);
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

// ----------------------------------Opcodes-----------------------------------

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
