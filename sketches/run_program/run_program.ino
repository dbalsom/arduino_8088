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
#include "opcodes.h"

// ------------------------- User-provided state ------------------------------

// Define the initial register state to load here.
// Reserved flags will be set for you.
registers LOAD_REGISTERS = {
  0x4000, // AX
  0x5000, // BX
  0x6000, // CX
  0x7000, // DX
  0x1000, // SS
  0xFFFF, // SP
  0x0000, // FLAGS
  0xCCCC, // IP
  0xF000, // CS
  0x2000, // DS
  0x3000, // ES
  0x0001, // BP
  0x1000, // SI
  0x0003, // DI
};

// Write the program to execute here, to start at the provided CS:IP.
const unsigned char CODE_SEGMENT[] = {  
  
  0x90, 0x90, 0x90, 0x90

};

// Specify how many bytes of CODE_SEGMENT to have prefetched before execution begins
// This value must be <= the length of CODE_SEGMENT
size_t PREFETCH_LEN = 4;

// -----------------------End User-provided state -----------------------------


static Cpu CPU;
registers INITIAL_REGISTERS;

// Register load routine.
static u8 LOAD_PROGRAM[] = {
  0x00, 0x00, 0xB8, 0x00, 0x00, 0x8E, 0xD0, 0x89, 0xC4, 0x9D, 0xBB, 0x00, 0x00, 0xB9, 0x00, 0x00,
  0xBA, 0x00, 0x00, 0xB8, 0x00, 0x00, 0x8E, 0xD0, 0xB8, 0x00, 0x00, 0x8E, 0xD8, 0xB8, 0x00, 0x00,
  0x8E, 0xC0, 0xB8, 0x00, 0x00, 0x89, 0xC4, 0xB8, 0x00, 0x00, 0x89, 0xC5, 0xB8, 0x00, 0x00, 0x89,
  0xC6, 0xB8, 0x00, 0x00, 0x89, 0xC7, 0xB8, 0x00, 0x00, 0xEA, 0x00, 0x00, 0x00, 0x00
};

static u8 JUMP_VECTOR[] = {
  0xEA, 0x00, 0x00, 0x00, 0x00
};

// Prefetch programs.
// We need an instruction that fetches one more byte then the length of the instruction.
// We use an 'or al, *' instruction here, manipulating the modrm to adjust the length of
// the EA calculation so that the first prefetched byte is executed in a t1 state.

// 0 is fed to OR so that al is not changed. However, this does overwrite flags, so POPF
// is used to reset the flag state. Luckily, POPF fetches one more byte so this works out
// for us nicely. The only issue is there appears to be a delay when a byte is fetched 
// from a full queue - POPF from a full queue will not fetch a code byte. I haven't 
// thought of a way around this yet, so for now, flags can only be preserved if prefetching
// 1-3 bytes.

size_t PREFETCH_PROGRAM_LEN = 0;
size_t PREFETCH_STACK_ADJUST = 0;
u8 *PREFETCH_PROGRAM = NULL;

// Program to set up 1 byte in prefetch and pop flags
static u8 PREFETCH_1[] = {
  0x0A, 0xC0, 0x9D
};

// Program to set up 2 bytes in prefetch and pop flags
static u8 PREFETCH_2[] = {
  0x0A, 0x0A, 0x9D, 0x90, 0x90, 0x90, 0x90
};

// Program to set up 3 bytes in prefetch and pop flags
static u8 PREFETCH_3[] = {
  0x0A, 0x02, 0x0A, 0x04, 0x9D
};

// Program to set up 4 bytes in prefetch
// NOTE: does not preserve flags due to fetch delays after full queue
static u8 PREFETCH_4[] = {
  0x0A, 0x02, 0x0A, 0x02, 0x0A, 0x02
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
static const u8 STORE_PROGRAM[] = {
  0xE7, 0xFE, 0x89, 0xD8, 0xE7, 0xFE, 0x89, 0xC8, 0xE7, 0xFE, 0x89, 0xD0, 0xE7, 0xFE, 0x8C, 0xD0,
  0xE7, 0xFE, 0x89, 0xE0, 0xE7, 0xFE, 0xB8, 0x00, 0x00, 0x8E, 0xD0, 0xB8, 0x04, 0x00, 0x89, 0xC4,
  0x9C, 0xE8, 0x00, 0x00, 0x8C, 0xC8, 0xE7, 0xFE, 0x8C, 0xD8, 0xE7, 0xFE, 0x8C, 0xC0, 0xE7, 0xFE,
  0x89, 0xE8, 0xE7, 0xFE, 0x89, 0xF0, 0xE7, 0xFE, 0x89, 0xF8, 0xE7, 0xFE, 0xB0, 0xFF, 0xE6, 0xFD
};

void setup() {
  Serial.begin(BAUD_RATE);

  // Set all output pins to OUTPUT
  for( int p = 0; p < (sizeof OUTPUT_PINS / sizeof OUTPUT_PINS[0]); p++ ) {
    pinMode(OUTPUT_PINS[p], OUTPUT);
  }
  // Set all input pins to INPUT
  for( int p = 0; p < (sizeof INPUT_PINS / sizeof INPUT_PINS[0]); p++ ) {
    pinMode(INPUT_PINS[p], INPUT);
  }

  // Default output pin states
  digitalWrite(READY_PIN, HIGH);
  digitalWrite(TEST_PIN, LOW);
  // Must set these to a known value or risk spurious interrupts!
  digitalWrite(INTR_PIN, LOW);
  digitalWrite(NMI_PIN, LOW);  

  // Wait for CPU to initialize
  delayMicroseconds(100);

  // Patch the reset vector jump
  patch_vector(JUMP_VECTOR, LOAD_SEG);

  // Prefetch request setup
  // Don't allow PREFETCH_LEN > CODE_SEGMENT
  if(PREFETCH_LEN > sizeof CODE_SEGMENT) {
    PREFETCH_LEN = 0;
  }
  switch(PREFETCH_LEN) {
    case 0:
      // No adjustment necessary
      PREFETCH_PROGRAM_LEN = 0;   
      PREFETCH_STACK_ADJUST = 0;
      break;
    case 1:
      PREFETCH_PROGRAM = PREFETCH_1;
      PREFETCH_PROGRAM_LEN = sizeof PREFETCH_1;
      PREFETCH_STACK_ADJUST = 2; // Account for POPF
      break;
    case 2:
      PREFETCH_PROGRAM = PREFETCH_2;
      PREFETCH_PROGRAM_LEN = sizeof PREFETCH_2;
      PREFETCH_STACK_ADJUST = 2; // Account for POPF
      break;
    case 3:
      PREFETCH_PROGRAM = PREFETCH_3;
      PREFETCH_PROGRAM_LEN = sizeof PREFETCH_3;
      PREFETCH_STACK_ADJUST = 2; // Account for POPF
      break;
    case 4:
      PREFETCH_PROGRAM = PREFETCH_4;
      PREFETCH_PROGRAM_LEN = sizeof PREFETCH_4;
      PREFETCH_STACK_ADJUST = 0; // Prefetch 4 does not pop flags
      break;
    default:  
      // Invalid value for prefetch specfied. Don't prefetch.
      PREFETCH_PROGRAM_LEN = 0;
      PREFETCH_STACK_ADJUST = 0;
      PREFETCH_LEN = 0;
  }

  adjust_flags(LOAD_REGISTERS.ax & 0xFF);

  // Save initial register state
  memcpy(&INITIAL_REGISTERS, &LOAD_REGISTERS, sizeof LOAD_REGISTERS);

  // Adjust IP by size of prefetch program so that we 'catch up' at end of prefetch
  LOAD_REGISTERS.ip -= PREFETCH_PROGRAM_LEN;
  // Adjust SP if prefetch program pops flags
  LOAD_REGISTERS.sp -= PREFETCH_STACK_ADJUST;

  // Patch load routine with specified register values
  patch_load(&LOAD_REGISTERS, LOAD_PROGRAM);
}

// Adjust user supplied flags. For various reasons we may not want to allow the user
// to specify certain flag states, or are unable to set them and must model the
// intial flag state
void adjust_flags(u8 result) {

  // Enforce reserved bits in user-provided flags
  LOAD_REGISTERS.flags &= CPU_FLAG_DEFAULT_CLEAR;
  LOAD_REGISTERS.flags |= CPU_FLAG_DEFAULT_SET;

  // Don't allow Trap or Interrupt flags
  LOAD_REGISTERS.flags &= ~CPU_FLAG_TRAP;
  LOAD_REGISTERS.flags &= ~CPU_FLAG_INTERRUPT;

  // A prefetch setup of 4 bytes cannot restore flags due to bus delays.
  // This means the starting result of flags should be adjusted for the 'or al, 0'
  // operation.
  if(PREFETCH_LEN == 4) {

    // Clear overflow & carry flags
    LOAD_REGISTERS.flags &= ~CPU_FLAG_OVERFLOW;
    LOAD_REGISTERS.flags &= ~CPU_FLAG_CARRY;

    // Zero flag
    if(result == 0) {
      LOAD_REGISTERS.flags |= CPU_FLAG_ZERO;
    }
    else {
      LOAD_REGISTERS.flags &= ~CPU_FLAG_ZERO;
    }
    // Sign flag
    if(result & 0x80) {
      LOAD_REGISTERS.flags |= CPU_FLAG_SIGN;
    }
    else {
      LOAD_REGISTERS.flags &= ~CPU_FLAG_SIGN;
    }
    // Parity flag. Could replace with table lookup
    int n_bits = 0;
    for(int i = 0; i < 8; i++) {
      if((result >> i) & 0x01) {
        // Count the 1 bit
        n_bits++;
      }
    }
    if(n_bits & 0x01) {
      // Parity is odd - clear parity flag
      LOAD_REGISTERS.flags &= ~CPU_FLAG_PARITY;
    }
    else {
      // Parity is even - set parity flag
      LOAD_REGISTERS.flags |= CPU_FLAG_PARITY;
    }
  }

}

void patch_vector(u8 *vec, u16 seg) {
  *((u16 *)&vec[3]) = seg;
}

void patch_load(registers *reg, u8 *prog) {
  *((u16 *)prog) = reg->flags;
  *((u16 *)&prog[LOAD_BX]) = reg->bx;
  *((u16 *)&prog[LOAD_CX]) = reg->cx;
  *((u16 *)&prog[LOAD_DX]) = reg->dx;
  *((u16 *)&prog[LOAD_SS]) = reg->ss;
  *((u16 *)&prog[LOAD_DS]) = reg->ds;
  *((u16 *)&prog[LOAD_ES]) = reg->es;
  *((u16 *)&prog[LOAD_SP]) = reg->sp;
  *((u16 *)&prog[LOAD_BP]) = reg->bp;
  *((u16 *)&prog[LOAD_SI]) = reg->si;
  *((u16 *)&prog[LOAD_DI]) = reg->di;
  *((u16 *)&prog[LOAD_AX]) = reg->ax;
  *((u16 *)&prog[LOAD_IP]) = reg->ip;
  *((u16 *)&prog[LOAD_CS]) = reg->cs;
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

  Serial.println(buf);

  // Expand flag info
  u16 f = regs->flags;
  char c_chr = CPU_FLAG_CARRY & f ? 'C' : 'c';
  char p_chr = CPU_FLAG_PARITY & f ? 'P' : 'p';
  char a_chr = CPU_FLAG_AUX_CARRY & f ? 'A' : 'a';
  char z_chr = CPU_FLAG_ZERO & f ? 'Z' : 'z';
  char s_chr = CPU_FLAG_SIGN & f ? 'S' : 's';
  char t_chr = CPU_FLAG_TRAP & f ? 'T' : 't';
  char i_chr = CPU_FLAG_INTERRUPT & f ? 'I' : 'i';
  char d_chr = CPU_FLAG_DIRECTION & f ? 'D' : 'd';
  char o_chr = CPU_FLAG_OVERFLOW & f ? 'O' : 'o';
  
  snprintf(
    flag_buf, 17,
    "1111%c%c%c%c%c%c0%c0%c1%c",
    o_chr, d_chr, i_chr, t_chr, s_chr, z_chr, a_chr, p_chr, c_chr
  );

  Serial.print("FLAGSINFO: ");
  Serial.println(flag_buf);
}

void print_cpu_state() {
  
  static char buf[81];
  static char op_buf[7];
  static char q_buf[15];
  
  char *ale_str = READ_ALE_PIN ? "A:" : "  ";
  
  char rs_chr = !READ_MRDC_PIN ? 'R' : '.';
  char aws_chr = !READ_AMWC_PIN ? 'A' : '.';
  char ws_chr = !READ_MWTC_PIN ? 'W' : '.';
  
  char ior_chr = !READ_IORC_PIN ? 'R' : '.';
  char aiow_chr = !READ_AIOWC_PIN ? 'A' : '.';
  char iow_chr = !READ_IOWC_PIN ? 'W' : '.';

  char v_chr = MACHINE_STATE_CHARS[(size_t)CPU.v_state];
  u8 q = (CPU.status0 >> 6) & 0x03;
  char q_char = QUEUE_STATUS_CHARS[q];
  char s = CPU.status0 & 0x07;

  // Get segment from S3 & S4
  char *seg_str = "  ";
  if(CPU.bus_cycle != t1) {
    // Status is not avaialble on T1 because address is latched
    u8 seg = ((CPU.status0 & 0x18) >> 3) & 0x03;
    seg_str = SEGMENT_STRINGS[(size_t)seg];
  }

  // Draw some sad ascii representation of bus transfers
  char *st_str = "  ";
  switch(CPU.bus_cycle) {
      case t1:
        
        if((CPU.bus_state != PASV) && (CPU.bus_state != HALT) && (CPU.bus_state != IRQA)) {
          // Begin a bus state
          st_str = "\\ ";
        }
        break;
      case t2: // FALLTHRU
      case t3: // FALLTHRU
      case tw:  
        // Continue a bus state
        st_str = " |";
        break;
      case t4:
        // End a bus state
        st_str = "/ ";
        break;
  }

  // Make string for bus reads and writes
  op_buf[0] = 0;
  if(!READ_MRDC_PIN || !READ_IORC_PIN) {
    snprintf(op_buf, 7, "<-r %02X", CPU.data_bus);
  }
  if(!READ_MWTC_PIN || !READ_IOWC_PIN) {
    snprintf(op_buf, 7, "w-> %02X", CPU.data_bus);
  }

  const char *q_str = queue_to_string();

  snprintf(
    buf, 81, 
    "%08ld %c %s[%05lX] %2s M:%c%c%c I:%c%c%c %-4s %s %2s %6s | %c%d [%-8s]", 
    CYCLE_NUM, 
    v_chr, 
    ale_str,
    CPU.address_latch, 
    seg_str,
    rs_chr, aws_chr, ws_chr,
    ior_chr, aiow_chr, iow_chr,
    BUS_STATE_STRINGS[(size_t)CPU.bus_state],
    CYCLE_STRINGS[(size_t)CPU.bus_cycle], 
    st_str,
    op_buf,
    q_char,
    CPU.queue.len,
    q_str
  );

  Serial.print(buf);

  if(q == QUEUE_FIRST) {
    // First byte of opcode read from queue. Decode it to opcode
    snprintf(q_buf, 15, " <-q %02X %s", CPU.qb, get_opcode_str(CPU.opcode, 0, false));
    Serial.print(q_buf);
  }
  else if(q == QUEUE_SUBSEQUENT) {
    if(IS_GRP_OP(CPU.opcode) && CPU.q_fn == 1) {
      // Modrm was just fetched for a group opcode, so display the mnemonic now
      snprintf(q_buf, 15, " <-q %02X %s", CPU.qb, get_opcode_str(CPU.opcode, CPU.qb, true));
    }
    else {
      snprintf(q_buf, 15, " <-q %02X", CPU.qb);
    }
    Serial.print(q_buf);
  }

  Serial.println("");
}

void change_state(machine_state new_state) {

  switch(new_state) {
    case Reset:
      CPU.doing_reset = true;    
      CPU.v_pc = 0;
      break;
    case JumpVector:
      CPU.v_pc = 0;
      break;
    case Load:
      // Set v_pc to 2 to skip flag bytes
      CPU.v_pc = 2;
      break;
    case LoadDone:
      break;
    case PrefetchSetup:
      CPU.v_pc = 0;
      break;
    case Execute:

      #if RESET_CYCLE_NUMBER
        CYCLE_NUM = 1;
      #endif

      if(CPU.v_state == PrefetchSetup) {
        // Carry over p_pc to v_pc as first bytes of program have been prefetched
        CPU.v_pc = CPU.p_pc;
      }
      else {
        CPU.v_pc = 0;
      }
      break;
    case Store:
      CPU.v_pc = 0;
      
      // Take a raw u8 pointer to the register struct. Both x86 and Arduino are little-endian,
      // so we can write raw incoming data over the struct. Faster than logic required to set
      // specific members. 
      CPU.readback_p = (u8 *)&CPU.post_regs;      
      break;
  }

  u32 state_end_time = micros();

  #if TRACE_STATE
  // Report time we spent in the previous state.
  if(CPU.state_begin_time != 0) {
    u32 elapsed = state_end_time - CPU.state_begin_time;
    Serial.print("## Changing to state: ");
    Serial.print(MACHINE_STATE_STRINGS[(size_t)new_state]);
    Serial.print(". Spent (");
    Serial.print(elapsed);
    Serial.println(") us in previous state.");
  }
  #endif

  CPU.state_begin_time = micros();
  CPU.v_state = new_state;
}

void cycle() {

  clock_tick();

  CYCLE_NUM++;

  if(!CPU.doing_reset) {
    switch(CPU.bus_cycle) {
      case t1:
        // Begin a bus cycle only if signalled
        if(CPU.bus_state != PASV) {
          CPU.bus_cycle = t2;
        }
        // Capture the state of the cycle in T1, as the state will go passive T3-T4
        CPU.cycle_state = (s_state)(CPU.status0 & 0x07);    
        break;

      case t2:
        CPU.bus_cycle = t3;
        break;

      case t3:
        // TODO: Handle wait states between t3 & t4
        CPU.bus_cycle = t4;
        break;

      case t4:
        // Did we complete a code fetch? If so, increment queue len
        if(CPU.cycle_state == CODE) {
          if(CPU.queue.len < QUEUE_MAX) {
            push_queue(CPU.data_bus, CPU.data_type);
          }
          else {
            // Shouldn't be here
            Serial.println("## Error: Invalid Queue Length++ ##");
          }
        }
        CPU.bus_cycle = t1;
        break;
    }
  }

  read_status0();
  // bus_state is the instantaneous state per cycle. May not always be valid. 
  // for state of the current bus cycle use cycle_state
  CPU.bus_state = (s_state)(CPU.status0 & 0x07);

  // Check QS0-QS1 queue status bits
  u8 q = (CPU.status0 >> 6) & 0x03;
  CPU.qb = 0xFF;
  CPU.q_ff = false;

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
      }
      else {
        // Subsequent byte of instruction fetched
        CPU.q_fn++;
      }
    }
    else {
      Serial.println("## Error: Invalid Queue Length-- ##");
    }
  }
  else if(q == QUEUE_FLUSHED) {
    // Queue was flushed last cycle
    empty_queue();

    #if TRACE_QUEUE
      Serial.println("## Queue Flushed ##");
      Serial.print("## PC: ");
      Serial.println(CPU.v_pc);
    #endif
  }

  if(READ_ALE_PIN) {
    // ALE signals start of bus cycle, so set cycle to t1.
    CPU.bus_cycle = t1;
    // Address lines are only valid when ALE is high, so latch address now.
    latch_address();
  }

  switch(CPU.v_state) {

    case JumpVector:
      // We are executing the initial jump from the reset vector FFFF:0000.
      // This is to avoid wrapping effective address during load procedure.
      // Optional - disable in header

      if(!READ_MRDC_PIN) {
        // CPU is reading (MRDC active-low)      
        if(CPU.bus_state == CODE) {    

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
        u32 dest = calc_flat_address(LOAD_SEG, 0);
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
            CPU.data_bus = LOAD_PROGRAM[CPU.v_pc];
            CPU.data_type = DATA_PROGRAM;
            CPU.v_pc++;
          }
          else {
            // Ran out of program, so return NOP. JMP cs:ip will actually fetch once before SUSP,
            // so we wil see this NOP prefetched.
            CPU.data_bus = OPCODE_NOP;
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
            Serial.println("## INVALID MEM READ DURING LOAD ##");
          }
        }
      } 

      if (q == 0x02) {
        // Queue flush after final jump triggers next state.
        change_state(LoadDone);
      }      
      break;

    case LoadDone:
      // LoadDone is triggered by the queue flush following the jump in Load.
      // We wait for the next ALE and begin Execute.
      if(READ_ALE_PIN) {
        // First bus cycle after jump. If requested prefetch is 0, transition directly
        // to execute state, otherwise transition to PrefetchSetup.
        if(PREFETCH_LEN > 0) {
          change_state(PrefetchSetup);
        } 
        else {
          change_state(Execute);
        }
      }
      break;

    // PrefetchSetup fills the prefetch queue with the requested number of bytes from 
    // the user program (or NOP on overflow).
    case PrefetchSetup:
      if(!READ_MRDC_PIN) {
        // CPU is reading (MRDC active-low)

        if(CPU.bus_state == CODE) {
          // CPU is reading code byte
          if(CPU.v_pc < PREFETCH_PROGRAM_LEN) {
            // Feed prefetch program instruction to CPU
            CPU.data_bus = PREFETCH_PROGRAM[CPU.v_pc];
            CPU.data_type = DATA_PREFETCH_PROGRAM;
            CPU.v_pc++;
          }
          else if(CPU.p_pc < PREFETCH_LEN) {
            // We are out of the prefetch program, feed bytes from code segment

            CPU.data_bus = CODE_SEGMENT[CPU.p_pc];
            CPU.data_type = DATA_PROGRAM;
            CPU.p_pc++;
          }
          else {
            // Ran out of program, so return a flagged NOP. (Shouldn't occur...)
            CPU.data_bus = OPCODE_NOP;
            CPU.data_type = DATA_PROGRAM_END;
          }
          data_bus_write(CPU.data_bus);
        }

        if(CPU.bus_state == MEMR && CPU.opcode == 0x0A) {
          // Memory read during OR prefetch-padding operation. To preserve AL, provide
          // 0 as an operand.
          //Serial.println("## OR MEMORY OPERAND ##");
          CPU.data_bus = 0;
          CPU.data_type = DATA_PREFETCH_PROGRAM;
          data_bus_write(CPU.data_bus);
        }

        if(CPU.bus_state == MEMR && CPU.opcode == OPCODE_POPF) {
          if(((segment)((CPU.status0 & 0x18) >> 3) & 0x03) == SegSS) {
            // Read from stack segment. This should be from prefetch POPF.

            if(CPU.p_popread_n < 2) {
              // First stack read, send LSB of flags
              //Serial.println("## POPF STACK READ ##");
              CPU.data_bus = LOAD_PROGRAM[CPU.p_popread_n];
              CPU.data_type = DATA_PREFETCH_PROGRAM;
            }
            else {
              Serial.println("## INVALID STACK READ DURING PREFETCH SETUP ##");
            }
          }
          data_bus_write(CPU.data_bus);
        }        
      }

      // We leave the PrefetchSetup state when the first byte of the user program is fetched
      // from the queue
      if(CPU.q_ff && (CPU.qt == DATA_PROGRAM)) {
        // From PrefetchSetup -> Execute transition, p_pc should become new v_pc
        change_state(Execute);
      }      
      break;

    case Execute:
      // CPU is reading (MRDC active-low)
      if(!READ_MRDC_PIN) {
        if(CPU.bus_state == CODE) {
          // Reading code byte
          if(CPU.v_pc < sizeof CODE_SEGMENT) {
            CPU.data_bus = CODE_SEGMENT[CPU.v_pc];
            CPU.data_type = DATA_PROGRAM;
            CPU.v_pc++;
          }
          else {
            // Ran out of program, so return NOP.
            CPU.data_bus = OPCODE_NOP;
            CPU.data_type = DATA_PROGRAM_END;
          }
        }

        if(CPU.bus_state == MEMR && CPU.opcode == 0x9D) {
          // Intercept POPF to prevent TRAP flag from being set
          CPU.data_bus = 0;
        }
        data_bus_write(CPU.data_bus);
      }

      // CPU is writing (MWTC active-low)
      if(!READ_MWTC_PIN) {
        CPU.data_bus = data_bus_read();
      }    

      // We end the Execute state when the first non-program NOP is fetched from the queue as the first
      // byte of an instruction. This accomodates user programs with incomplete instructions, they will 
      // fetch 'subsequent byte' NOPs for remaining required arguments (hopefully valid ones), until a 
      // 'first byte' NOP is read.
      if(CPU.q_ff && (CPU.qt == DATA_PROGRAM_END)) {
        change_state(Store);
      }

      break;

    case Store:
      // We are executing the Store program.

      if(!READ_MRDC_PIN) {
        // CPU is reading
        
        if(CPU.bus_state == CODE) {    
          // CPU is doing code fetch
          if(CPU.v_pc < sizeof STORE_PROGRAM) {
            // Read code byte from store program
            CPU.data_bus = STORE_PROGRAM[CPU.v_pc];
            CPU.data_type = DATA_PROGRAM;
            CPU.v_pc++;
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
            Serial.println("## Store Stack Push");
          #endif
          *CPU.readback_p = CPU.data_bus;
          CPU.readback_p++;
        }
        else {
          // We shouldn't be writing to any other addresses, something wrong happened
          Serial.print("## INVALID STORE WRITE @");
          Serial.print(CPU.v_pc, HEX);
          Serial.println(" ##");
          //print_cpu_state();
        }
        #if DEBUG_STORE
          Serial.print("## Store memory write: ");
          Serial.println(CPU.data_bus, HEX);
        #endif
      }    

      // CPU is writing to IO address - this indicates we are saving a register value. 
      // We structured the register struct in the right order, so we can overwrite it
      // with raw u8s.
      if(!READ_IOWC_PIN) {
        CPU.data_bus = data_bus_read();

        *CPU.readback_p = CPU.data_bus;
        CPU.readback_p++;

        #if DEBUG_STORE
          Serial.print("## Store IO write: ");
          Serial.println(CPU.data_bus, HEX);
        #endif

        if(CPU.address_latch == 0xFD) {
          // Write to 0xFD indicates end of store procedure.
          
          // Adjust IP by offset of CALL instruction.
          #if DEBUG_STORE
            Serial.print("## Unadjusted IP: ");
            Serial.println(CPU.post_regs.ip, HEX);
          #endif            
          CPU.post_regs.ip -= 0x24;
          change_state(Done);
        }
      }

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
    case PrefetchSetup:
      #if TRACE_PREFETCH
        print_cpu_state();
      #endif
      break;
    case Execute:
      #if TRACE_EXECUTE
        print_cpu_state();
      #endif 
      break;
    case Store:
      #if TRACE_STORE
        print_cpu_state();  
      #endif
      break;
  }
}

void print_addr(unsigned long addr) {
  static char addr_buf[6];
  snprintf(addr_buf, 6, "%05lX", addr);
  Serial.println(addr_buf);
}

// Resets the CPU by asserting RESET line for at least 4 cycles and waits for ALE signal.
bool cpu_reset() {

  memset(&CPU, 0, sizeof CPU);
  
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

  /*
  unsigned long start_time = micros();
  while((READ_ALE_PIN == false)) {
  }
  unsigned long end_time = micros();
  unsigned long elapsed = end_time - start_time;

  Serial.print("ALE took: ");
  Serial.print(elapsed);
  Serial.print("us (");
  Serial.print(ale_cycles);
  Serial.println("cycles)");
  */

  // ALE did not turn on within the specified cycle timeout, so we failed to reset the  cpu.
  return false;
}

void loop() {
  
  bool is_reset = false;
  bool tried_reset = false;

  static unsigned long cycle_ct = 0;

  while (CPU.v_state != Done) {
   
    if (!tried_reset) {
      // Only try resetting once

      Serial.println("Resetting CPU...");
      bool result = cpu_reset();
      //bool result = false;

      if (result) {
        Serial.println("Reset CPU!");

        #if USE_LOAD_SEG
          change_state(JumpVector);
        #else
          change_state(Load);
        #endif
      }
      else {
        Serial.println("Failed to reset CPU.");
      }
      tried_reset = true;
    }

    cycle();
    cycle_ct++;

    if(CPU.v_state == Done) {

      Serial.println(">> Initial Loaded state:");
      print_registers(&INITIAL_REGISTERS);
      Serial.println(">> End state:");
      print_registers(&CPU.post_regs);
    }
  }
}
