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
#ifndef _ARDUINO8088_OPCODES_H
#define _ARDUINO8088_OPCODES_H

#define OPCODE_NOP 0x90
#define OPCODE_DOUBLENOP 0x9090
#define MODRM_OP(M) (((M & 0b00111000) >> 3) & 0x07)

#define GRP1 105
#define GRP2A 106
#define GRP2B 110
#define GRP3 107
#define GRP4 108
#define GRP5 109
#define IS_GRP_OP(O) ((OPCODE_REFS[O] >= GRP1) && (OPCODE_REFS[O] <= GRP2B))

// LUT of primary opcode to Mnemonic (Or Group name)
static const uint8_t OPCODE_REFS[] = {
 0, 0, 0, 0, 0, 0, 1, 2, 3, 3, 3, 3, 3, 3, 1, 2,
 4, 4, 4, 4, 4, 4, 1, 2, 5, 5, 5, 5, 5, 5, 1, 2,
 6, 6, 6, 6, 6, 6, 7, 8, 9, 9, 9, 9, 9, 9, 10, 11, 
 12, 12, 12, 12, 12, 12, 13, 14, 15, 15, 15, 15, 15, 15, 16, 17,
 18, 18, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 19, 19, 19,
 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2,
 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35,
 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35,
 105, 105, 105, 105, 36, 36, 37, 37, 38, 38, 38, 38, 38, 39, 38, 2,
 111, 37, 37, 37, 37, 37, 37, 37, 40, 41, 42, 103, 43, 44, 45, 46,
 38, 38, 38, 38, 47, 48, 49, 50, 36, 36, 51, 52, 53, 54, 55, 56,
 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38,
 57, 57, 57, 57, 58, 59, 38, 38, 60, 60, 60, 60, 61, 61, 62, 63,
 106, 106, 110, 110, 71, 73, 104, 75, 104, 104, 104, 104, 104, 104, 104, 104,
 76, 77, 78, 79, 80, 80, 81, 81, 82, 83, 84, 83, 80, 80, 81, 81,
 85, 104, 86, 87, 89, 90, 107, 107, 97, 98, 99, 100, 101, 102, 108, 109 
};

static char *OPCODE_STRS[] = {
  "ADD",
  "PUSH",
  "POP",
  "OR",
  "ADC",
  "SBB",
  "AND",
  "ES",
  "DAA",
  "SUB",
  "CS",
  "DAS",
  "XOR",
  "SS",
  "AAA",
  "CMP",
  "DS",
  "AAS",
  "INC",
  "DEC",
  "JO",
  "JNO",
  "JB",
  "JNB",
  "JZ",
  "JNZ",
  "JBE",
  "JNBE",
  "JS",
  "JNS",
  "JP",
  "JNP",
  "JL",
  "JNL",
  "JLE",
  "JNLE",
  "TEST",
  "XCHG",
  "MOV",
  "LEA",
  "CBW",
  "CWD",
  "CALLF",
  "PUSHF",
  "POPF",
  "SAHF",
  "LAHF",
  "MOVSB",
  "MOVSW",
  "CMPSB",
  "CMPSW",
  "STOSB",
  "STOSW",
  "LODSB",
  "LODSW",
  "SCASB",
  "SCASW",
  "RETN",
  "LES",
  "LDS",
  "RETF",
  "INT",
  "INTO",
  "IRET",
  "ROL",
  "ROR",
  "RCL",
  "RCR",
  "SHL",
  "SHR",
  "SAR",
  "AAM",
  "AMX",
  "AAD",
  "ADX",
  "XLAT",
  "LOOPNE",
  "LOOPE",
  "LOOP",
  "JCXZ",
  "IN",
  "OUT",
  "CALL",
  "JMP",
  "JMPF",
  "LOCK",
  "REPNZ",
  "REP",
  "REPZ",
  "HLT",
  "CMC",
  "NOT",
  "NEG",
  "MUL",
  "IMUL",
  "DIV",
  "IDIV",
  "CLC",
  "STC",
  "CLI",
  "STI",
  "CLD",
  "STD",
  "WAIT",
  "INVAL",
  "GRP1",
  "GRP2A",
  "GRP3",
  "GRP4",
  "GRP5",
  "GRP2B",
  "NOP",
  
};

// 0x80 - 0x81
static char *OPCODE_STRS_GRP1[] = {
  "ADD",
  "OR",
  "ADC",
  "SBB",
  "AND",
  "SUB",
  "XOR",
  "CMP"
};

// 0xD0 - 0xD1
static char *OPCODE_STRS_GRP2A[] = {
  "ROL",
  "ROR",
  "RCL",
  "RCR",
  "SHL",
  "SHR",
  "SETMO",
  "SAR"
};

// 0xD2 - 0xD3
static char *OPCODE_STRS_GRP2B[] = {
  "ROL",
  "ROR",
  "RCL",
  "RCR",
  "SHL",
  "SHR",
  "SETMOC",
  "SAR"
};

// 0xF6 - 0xF7
static char *OPCODE_STRS_GRP3[] = {
  "TEST",
  "TEST",
  "NOT",
  "NEG",
  "MUL",
  "IMUL",
  "DIV",
  "IDIV",
};

// 0xFE
static char *OPCODE_STRS_GRP4[] = {
  "INC",
  "DEC",
  "INVAL",
  "INVAL",
  "INVAL",
  "INVAL",
  "INVAL",
  "INVAL"
};

// 0xFF
static char *OPCODE_STRS_GRP5[] = {
  "INC",
  "DEC",
  "CALL",
  "CALLF",
  "JMP",
  "JMPF",
  "PUSH",
  "INVAL"
};

#endif 