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
#ifndef _CPU_SERVER_H
#define _CPU_SERVER_H

#define BAUD_RATE 460800

#define CMD_TIMEOUT 100 // Command timeout in milliseconds
#define MAX_COMMAND_BYTES 28 // Maximum length of command parameter input

#define MODE_ASCII 0 // Use ASCII response codes (for interactive debugging only, client won't support)
#define DEBUG_PROTO 0 // Insert debugging messages into serial output (Escaped by ##...##)

#define MAX_ERR_LEN 50 // Maximum length of an error string

const char RESPONSE_CHRS[] = {
  '!', '.'
};

const char VERSION_DAT[] = {
  'a', 'r', 'd', '8', '0', '8', '8'
};

const u8 VERSION_NUM = 1;

typedef enum {
  CmdNone            = 0x00,
  CmdVersion         = 0x01,
  CmdReset           = 0x02,
  CmdLoad            = 0x03,
  CmdCycle           = 0x04,
  CmdReadAddress     = 0x05,
  CmdReadStatus      = 0x06,
  CmdRead8288Command = 0x07,
  CmdRead8288Control = 0x08, 
  CmdReadDataBus     = 0x09,
  CmdWriteDataBus    = 0x0A,
  CmdFinalize        = 0x0B,
  CmdStore           = 0x0C,
  CmdQueueLen        = 0x0D,
  CmdQueueBytes      = 0x0E,
  CmdWritePin        = 0x0F,
  CmdReadPin         = 0x10,
  CmdGetProgramState = 0x11,
  CmdLastError       = 0x12,
  CmdInvalid         = 0x13,
  
} server_command;

typedef bool (*command_func)();

#define RESPONSE_FAIL 0x00
#define RESPONSE_OK 0x01

// ASCII aliases for commands, mostly for interactive debugging
const u8 CMD_ALIASES[] = {
  0, // CmdNone
  'v', // CmdVersion
  'r', // CmdReset
  'l', // CmdLoad
  'c', // CmdCycle
  'a', // CmdReadAddress
  's', // CmdReadStatus
  't', // CmdRead8288Command
  'u', // CmdRead8288Control
  'r', // CmdReadDataBus
  'w', // CmdWriteDataBus,
  'z', // CmdFinalize
  'v', // CmdStore,
  'q', // CmdQueueLen,
  'b', // CmdQueueBytes,
  'x', // CmdWritePin,
  'y', // CmdReadPin,
  'g', // CmdGetProgramState
  'e', // CmdGetLastError
  0 // CmdInvalid
};

// List of valid arguments to CmdWritePin. Only these specific pins
// can have state written to.
const u8 WRITE_PINS[] = {
  6,  // READY
  7,  // TEST
  10,  // !LOCK
  12, // INTR
  13, // NMI
};

// Number of argument bytes expected for each command
const u8 CMD_INPUTS[] = {
  0,  // CmdNone
  0,  // CmdVersion
  0,  // CmdReset
  28, // CmdLoad
  0,  // CmdCycle
  0,  // CmdReadAddress
  0,  // CmdReadStatus
  0,  // CmdRead8288Command 
  0,  // CmdRead8288Control 
  0,  // CmdReadDataBus 
  1,  // CmdWriteDataBus
  0,  // CmdFinalize
  0,  // CmdStore,
  0,  // CmdQueueLen,
  0,  // CmdQueueBytes,
  2,  // CmdWritePin,
  1,  // CmdReadPin,
  0,  // CmdGetProgramState,
  0,  // CmdGetLastError,
  0   // CmdInvalid
};

typedef enum {
  WaitingForCommand = 0x01,
  ReadingCommand,
  ExecutingCommand
} command_state;

typedef struct server_state {
  command_state c_state;
  server_command cmd;
  u8 cmd_byte_n;
  u8 cmd_bytes_expected;
  u32 cmd_start_time;
} Server;

bool cmd_version(void);
bool cmd_reset(void);
bool cmd_load(void);
bool cmd_cycle(void);
bool cmd_read_address(void);
bool cmd_read_status(void);
bool cmd_read_8288_command(void);
bool cmd_read_8288_control(void);
bool cmd_read_data_bus(void);
bool cmd_write_data_bus(void);
bool cmd_finalize(void);
bool cmd_store(void);
bool cmd_queue_len(void);
bool cmd_queue_bytes(void);
bool cmd_write_pin(void);
bool cmd_read_pin(void);
bool cmd_get_program_state(void);
bool cmd_get_last_error(void);

#endif
