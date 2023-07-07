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
#ifndef _LIB_H
#define _LIB_H

bool cpu_reset();
const char *get_opcode_str(uint8_t op1, uint8_t op2, bool modrm);
const char *queue_to_string();
uint32_t calc_flat_address(uint16_t seg, uint16_t offset);
uint8_t data_bus_read();
uint8_t read_queue(size_t idx);
void beep(uint32_t time);
void clock_tick();
void data_bus_write(uint8_t byte);
void empty_queue();
void error_beep();
void init_queue();
void latch_address();
void pop_queue(uint8_t *byte, uint8_t *dtype);
void print_queue();
void push_queue(uint8_t byte, uint8_t dtype);
void read_8288_command_bits();
void read_8288_control_bits();
void read_status0();
void test_queue();

#endif
