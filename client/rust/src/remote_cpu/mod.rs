#![allow(dead_code, unused_variables)]

use crate::arduino_8088_client::*;

mod queue;
#[macro_use]
mod opcodes;

use crate::remote_cpu::queue::*;
use crate::remote_cpu::opcodes::*;

pub const CPU_FLAG_CARRY: u16      = 0b0000_0000_0000_0001;
pub const CPU_FLAG_RESERVED1: u16  = 0b0000_0000_0000_0010;
pub const CPU_FLAG_PARITY: u16     = 0b0000_0000_0000_0100;
pub const CPU_FLAG_RESERVED3: u16  = 0b0000_0000_0000_1000;
pub const CPU_FLAG_AUX_CARRY: u16  = 0b0000_0000_0001_0000;
pub const CPU_FLAG_RESERVED5: u16  = 0b0000_0000_0010_0000;
pub const CPU_FLAG_ZERO: u16       = 0b0000_0000_0100_0000;
pub const CPU_FLAG_SIGN: u16       = 0b0000_0000_1000_0000;
pub const CPU_FLAG_TRAP: u16       = 0b0000_0001_0000_0000;
pub const CPU_FLAG_INT_ENABLE: u16 = 0b0000_0010_0000_0000;
pub const CPU_FLAG_DIRECTION: u16  = 0b0000_0100_0000_0000;
pub const CPU_FLAG_OVERFLOW: u16   = 0b0000_1000_0000_0000;

const ADDRESS_SPACE: usize = 1_048_576;
const IO_FINALIZE_ADDR: u32 = 0x00FF;

const ISR_SEGMENT: u16 = 0xF800;

#[derive (Default, Debug)]
pub struct RemoteCpuRegisters {
    
    pub ax: u16,
    pub bx: u16,
    pub cx: u16,
    pub dx: u16,
    pub ss: u16,
    pub ds: u16,
    pub es: u16,
    pub sp: u16,
    pub bp: u16,
    pub si: u16,
    pub di: u16,
    pub cs: u16,
    pub ip: u16,
    pub flags: u16
}

#[derive (PartialEq)]
pub enum BusCycle {
    T1,
    T2,
    T3,
    T4,
    Tw
}

pub struct RemoteCpu {
    client: CpuClient,
    regs: RemoteCpuRegisters,
    memory: Vec<u8>,
    pc: usize,
    start_addr: usize,
    end_addr: usize,
    program_state: ProgramState,

    address_latch: u32,
    status: u8,
    command_status: u8,
    control_status: u8,
    data_bus: u8,
    data_type: QueueDataType,

    cycle_num: u32,
    mcycle_state: BusState,
    bus_cycle: BusCycle,

    queue: InstructionQueue,
    queue_byte: u8,
    queue_type: QueueDataType,
    queue_first_fetch: bool,
    queue_fetch_n: u8,
    queue_fetch_addr: u32,
    opcode: u8,
    finalize: bool,

    do_nmi: bool,
    intr: bool,
}

impl RemoteCpu {
    pub fn new(client: CpuClient) -> RemoteCpu {
        RemoteCpu {
            client,
            regs: Default::default(),
            memory: vec![0; ADDRESS_SPACE],
            pc: 0,
            start_addr: 0,
            end_addr: 0,
            program_state: ProgramState::Reset,
            address_latch: 0,
            status: 0,
            command_status: 0,
            control_status: 0,
            data_bus: 0,
            data_type: QueueDataType::Program,
            cycle_num: 0,
            mcycle_state: BusState::PASV,
            bus_cycle: BusCycle::T1,
            queue: InstructionQueue::new(),
            queue_byte: 0,
            queue_type: QueueDataType::Program,
            queue_first_fetch: true,
            queue_fetch_n: 0,
            queue_fetch_addr: 0,
            opcode: 0,
            finalize: false,
            do_nmi: false,
            intr: false
        }
    }

    pub fn reset(&mut self) {

        self.program_state = ProgramState::Reset;
        self.address_latch = 0;
        self.status = 0;
        self.command_status = 0;
        self.control_status = 0;
        self.data_bus = 0;
        self.data_type = QueueDataType::Program;
        self.cycle_num = 0;
        self.mcycle_state = BusState::PASV;
        self.bus_cycle = BusCycle::T1;
        self.queue = InstructionQueue::new();
        self.queue_byte = 0;
        self.queue_type = QueueDataType::Program;
        self.queue_first_fetch = true;
        self.queue_fetch_n = 0;
        self.queue_fetch_addr = 0;
        self.opcode = 0;
        self.finalize = false;
        self.do_nmi = false;
    }

    pub fn set_pc(&mut self, cs: u16, ip: u16) {

        self.regs.cs = cs;
        self.regs.ip = ip;
        self.pc = ((cs as usize) << 4) + ip as usize;
    }

    pub fn get_pc(&self) -> usize {
        self.pc
    }

    pub fn mount_bin(&mut self, data: &[u8], location: usize) -> bool {

        let src_size = data.len();

        if location + src_size > self.memory.len() {
            // copy request goes out of bounds
            return false
        }
        
        let mem_slice: &mut [u8] = &mut self.memory[location..location + src_size];
        for (dst, src) in mem_slice.iter_mut().zip(data) {
            *dst = *src;
        }

        // Update end address past sizeof program
        self.start_addr = location;
        self.end_addr = location + src_size;

        log::debug!("start addr: {:05X} end addr: {:05X}", self.start_addr, self.end_addr);

        true
    }

    pub fn set_program_bounds(&mut self, start: usize, end: usize) {
        self.start_addr = start;
        self.end_addr = end;
    }

    pub fn setup_ivr(&mut self) {

        for i in 0..256 {
            // Calculate address of ISR for each IVT entry

            let isr_offset: usize = i * 4;
            
            // Write offset first
            self.write_u16(isr_offset, (isr_offset / 2) as u16);
            // Write segment next
            self.write_u16(isr_offset + 2, ISR_SEGMENT);

            // Write ISR routine 
            let isr_address = RemoteCpu::calc_linear_address(ISR_SEGMENT, (isr_offset / 2) as u16);

            self.memory[isr_address as usize] = OPCODE_IRET;
            self.memory[(isr_address + 1) as usize] = OPCODE_NOP;

        }
    }

    /// Return true if this address is an ISR 
    pub fn is_isr_address(&mut self, address: u32) -> bool {
        
        let isr_start =  RemoteCpu::calc_linear_address(ISR_SEGMENT, 0);
        let isr_end = RemoteCpu::calc_linear_address(ISR_SEGMENT, 256 * 4);
        
        if address >= isr_start && address < isr_end {
            true
        }
        else {
            false
        }
    }

    pub fn write_u8(&mut self, address: usize, byte: u8) {
        if address < self.memory.len() {
            self.memory[address] = byte;
        }
    }

    pub fn write_u16(&mut self, address: usize, word: u16) {
        if address < self.memory.len() - 1 {
            self.memory[address] = (word & 0xFF) as u8;
            self.memory[address+1] = (word >> 8) as u8;     
        }
    }

    pub fn calc_linear_address(segment: u16, offset: u16) -> u32 {
        ((segment as u32) << 4) + offset as u32 & 0xFFFFFu32
    }

    pub fn load_registers_from_buf(&mut self, reg_data: &[u8]) -> bool {

        self.reset(); // CPU is reset on register load

        match self.client.load_registers_from_buf(reg_data) {
            Ok(_) => true,
            Err(_) => false
        }
    }

    pub fn load_registers_from_struct(&mut self, regs: &RemoteCpuRegisters) -> bool {

        self.reset(); // CPU is reset on register load

        let mut reg_data: [u8; 28] = [0; 28];
        // ax:    (r[0x00] as u16) | (r[0x01] as u16) << 8,
        // bx:    (r[0x02] as u16) | (r[0x03] as u16) << 8,
        // cx:    (r[0x04] as u16) | (r[0x05] as u16) << 8,
        // dx:    (r[0x06] as u16) | (r[0x07] as u16) << 8,
        // ss:    (r[0x08] as u16) | (r[0x09] as u16) << 8,
        // sp:    (r[0x0A] as u16) | (r[0x0B] as u16) << 8,
        // flags: (r[0x0C] as u16) | (r[0x0D] as u16) << 8,
        // ip:    (r[0x0E] as u16) | (r[0x0F] as u16) << 8,
        // cs:    (r[0x10] as u16) | (r[0x11] as u16) << 8,
        // ds:    (r[0x12] as u16) | (r[0x13] as u16) << 8,
        // es:    (r[0x14] as u16) | (r[0x15] as u16) << 8,
        // bp:    (r[0x16] as u16) | (r[0x17] as u16) << 8,
        // si:    (r[0x18] as u16) | (r[0x19] as u16) << 8,
        // di:    (r[0x1A] as u16) | (r[0x1B] as u16) << 8,
        reg_data[0] = (regs.ax & 0xFF) as u8;
        reg_data[1] = (regs.ax >> 8) as u8;
        reg_data[2] = (regs.bx & 0xFF) as u8;
        reg_data[3] = (regs.bx >> 8) as u8;
        reg_data[4] = (regs.cx & 0xFF) as u8;
        reg_data[5] = (regs.cx >> 8) as u8;
        reg_data[6] = (regs.dx & 0xFF) as u8;
        reg_data[7] = (regs.dx >> 8) as u8;
        reg_data[8] = (regs.ss & 0xFF) as u8;
        reg_data[9] = (regs.ss >> 8) as u8;    
        reg_data[10] = (regs.sp & 0xFF) as u8;
        reg_data[11] = (regs.sp >> 8) as u8;    
        reg_data[12] = (regs.flags & 0xFF) as u8;
        reg_data[13] = (regs.flags >> 8) as u8;            
        reg_data[14] = (regs.ip & 0xFF) as u8;
        reg_data[15] = (regs.ip >> 8) as u8;    
        reg_data[16] = (regs.cs & 0xFF) as u8;
        reg_data[17] = (regs.cs >> 8) as u8;            
        reg_data[18] = (regs.ds & 0xFF) as u8;
        reg_data[19] = (regs.ds >> 8) as u8;   
        reg_data[20] = (regs.es & 0xFF) as u8;
        reg_data[21] = (regs.es >> 8) as u8;           
        reg_data[22] = (regs.bp & 0xFF) as u8;
        reg_data[23] = (regs.bp >> 8) as u8;  
        reg_data[24] = (regs.si & 0xFF) as u8;
        reg_data[25] = (regs.si >> 8) as u8;  
        reg_data[26] = (regs.di & 0xFF) as u8;
        reg_data[27] = (regs.di >> 8) as u8;          

        match self.client.load_registers_from_buf(&reg_data) {
            Ok(_) => true,
            Err(_) => false
        }
    }    

    pub fn update_state(&mut self) -> bool {
        /*
        self.program_state = self.client.get_program_state().expect("Failed to get program state!");
        self.status = self.client.read_status().expect("Failed to get status!");
        self.command_status = self.client.read_8288_command().expect("Failed to get 8288 command status!");
        self.control_status = self.client.read_8288_control().expect("Failed to get 8288 control status!");
        self.data_bus = self.client.read_data_bus().expect("Failed to get data bus!");
        */

        (
            self.program_state, 
            self.control_status, 
            self.status, 
            self.command_status, 
            self.data_bus
        ) = self.client.get_cycle_state().expect("Failed to get cycle state!");

        
        true
    }

    pub fn get_last_error(&mut self) -> String {

        let error_msg = match self.client.get_last_error() {
            Ok(err_str) => err_str,
            Err(err) => format!("Couldn't get error string: {err}")
        };

        error_msg
    }

    pub fn cycle(&mut self) -> bool {

        self.client.cycle().expect("Failed to cycle cpu!");
        
        self.bus_cycle = match self.bus_cycle {
            BusCycle::T1 => {
                // Capture the state of the bus transfer in T1, as the state will go PASV in t3-t4
                self.mcycle_state = get_bus_state!(self.status);
                
                // Only exit T1 state if bus transfer state indicates a bus transfer
                match get_bus_state!(self.status) {
                    BusState::PASV => BusCycle::T1,
                    BusState::HALT => BusCycle::T1,
                    _ => BusCycle::T2
                }
            }
            BusCycle::T2 => {
                BusCycle::T3
            }
            BusCycle::T3 => {
                // TODO: Handle wait states
                BusCycle::T4
            }
            BusCycle::Tw => {
                // TODO: Handle wait states
                BusCycle::T4
            }            
            BusCycle::T4 => {
                if self.mcycle_state == BusState::CODE {
                    // We completed a code fetch, so add to prefetch queue

                    if !self.is_isr_address(self.address_latch) 
                        && ((self.address_latch as usize >= self.end_addr)
                        || ((self.address_latch as usize) < self.start_addr)) {
                        // We fetched past the end of the current program, push a flagged NOP into the queue.
                        // When a byte flagged with Finalize is read we will enter the Finalize state.
                        self.queue.push(self.data_bus, QueueDataType::Finalize, self.address_latch);
                    }
                    else {
                        // Normal fetch
                        self.queue.push(self.data_bus, self.data_type, self.address_latch);
                    }
                }
                BusCycle::T1
            }            
        };

        self.update_state();
        if self.program_state == ProgramState::ExecuteDone {
            self.cycle_num += 1;
            return true
        }

        if(self.command_status & COMMAND_ALE_BIT) != 0 {
            if self.bus_cycle != BusCycle::T1 {
                log::warn!("ALE on non-T1 cycle state! CPU desynchronized.");
                self.bus_cycle = BusCycle::T1;
            }

            self.address_latch = self.client.read_address_latch().expect("Failed to get address latch!");
        }
        //log::trace!("state: {:?}", self.program_state);

        // Do reads & writes if we are in execute state.
        if self.program_state == ProgramState::Execute {
            // MRDC status is active-low.
            if ((self.command_status & COMMAND_MRDC_BIT) == 0) && (self.bus_cycle == BusCycle::T2) {

                match self.mcycle_state {
                    BusState::MEMR => {
                        // CPU is reading data from bus. Provide value from memory.
                        self.data_bus = self.memory[self.address_latch as usize];
                    }
                    BusState::CODE => {
                        // CPU is reading code from bus. Provide value from memory if we are not past the 
                        // end of the program area.

                        if self.address_latch as usize >= self.end_addr {
                            self.data_bus = OPCODE_NOP;
                        }
                        else {
                            self.data_bus = self.memory[self.address_latch as usize];
                        }
                    }                    
                    _ => {
                        // Handle other states?
                    }
                }


                //log::trace!("Reading data bus: {:02X}", self.data_bus);
                self.client.write_data_bus(self.data_bus).expect("Failed to write data bus.");
            }

            // MWTC status is active-low.
            if (self.command_status & COMMAND_MWTC_BIT) == 0 {
                // CPU is writing to memory. Get data bus from CPU and write to memory..
                self.data_bus = self.client.read_data_bus().expect("Failed to read data bus.");
                self.memory[self.address_latch as usize] = self.data_bus;
            }

            // IOWC status is active-low.
            if (self.command_status & COMMAND_IOWC_BIT) == 0 {
                // CPU is writing to IO address.

                self.data_bus = self.client.read_data_bus().expect("Failed to read data bus.");

                // Check if this is our special port address
                if self.address_latch == 0x000FF {
                    //log::trace!("IO write to INTR trigger!");
                    
                    // Set INTR line high
                    self.client.write_pin(CpuPin::INTR, true).expect("Failed to set INTR line high.");
                    self.intr = true;
                }
            }
        }

        // Handle queue activity
        let q_op = get_queue_op!(self.status);
            
        match q_op {
            QueueOp::First | QueueOp::Subsequent => {
                // We fetched a byte from the queue last cycle
                (self.queue_byte, self.queue_type, self.queue_fetch_addr) = self.queue.pop();
                if q_op == QueueOp::First {
                    // First byte of instruction fetched.
                    self.queue_first_fetch = true;
                    self.queue_fetch_n = 0;
                    self.opcode = self.queue_byte;

                    // Was NMI triggered?
                    if self.do_nmi {
                        log::trace!("Setting NMI pin high...");
                        self.client.write_pin(CpuPin::NMI, true).expect("Failed to write NMI pin!");
                        self.do_nmi = false;
                    }

                    // Is this opcode an NMI trigger?
                    if self.opcode == OPCODE_NMI_TRIGGER {
                        // set flag to enable NMI on next instruction
                        
                        //self.do_nmi = true;
                    }

                    // Is this opcode flagged as the end of execution?
                    if self.queue_type == QueueDataType::Finalize {
                        
                        log::trace!("Finalizing execution!");
                        self.client.finalize().expect("Failed to finalize!");
                    }
                }
                else {
                    // Subsequent byte of instruction fetched
                    self.queue_fetch_n += 1;
                }
            }
            QueueOp::Flush => {
                // Queue was flushed last cycle
                self.queue.flush();
            }
            _ => {}
        }


        self.cycle_num += 1;

        if self.cycle_num > 100 {
            match self.client.finalize() {
                Ok(_) => {
                    log::trace!("Finalized execution!");
                }
                Err(_) => {
                    log::trace!("Failed to finalize: {}", self.client.get_last_error().unwrap());
                }
            }
        }
        true
    }   


    pub fn print_cpu_state(&mut self) {
        println!("{}", self.get_cpu_state_str())
    }

    pub fn get_cpu_state_str(&mut self) -> String {

        let ale_str = match self.command_status & COMMAND_ALE_BIT != 0 {
            true => "A:",
            false => "  "
        };

        let mut seg_str = "  ";
        if self.bus_cycle != BusCycle::T1 {
            // Segment status only valid in T2+
            seg_str = match get_segment!(self.status) {
                Segment::ES => "ES",
                Segment::SS => "SS",
                Segment::CS => "CS",
                Segment::DS => "DS"
            };    
        }

        let q_op = get_queue_op!(self.status);
        let q_op_chr = match q_op {
            QueueOp::Idle => ' ',
            QueueOp::First => 'F',
            QueueOp::Flush => 'E',
            QueueOp::Subsequent => 'S'
        };

        // All read/write signals are active/low
        let rs_chr   = match self.command_status & 0b0000_0001 == 0 {
            true => 'R',
            false => '.',
        };
        let aws_chr  = match self.command_status & 0b0000_0010 == 0 {
            true => 'A',
            false => '.',
        };
        let ws_chr   = match self.command_status & 0b0000_0100 == 0 {
            true => 'W',
            false => '.',
        };
        let ior_chr  = match self.command_status & 0b0000_1000 == 0 {
            true => 'R',
            false => '.',
        };
        let aiow_chr = match self.command_status & 0b0001_0000 == 0 {
            true => 'A',
            false => '.',
        };
        let iow_chr  = match self.command_status & 0b0010_0000 == 0 {
            true => 'W',
            false => '.',
        };

        let intr_chr = if self.intr { 'R' } else { '.' };
        let inta_chr = if self.command_status & COMMAND_INTA_BIT == 0 { 'A' } else { '.' };

        let bus_str = match get_bus_state!(self.status) {
            BusState::IRQA => "IRQA",
            BusState::IOR  => "IOR ",
            BusState::IOW  => "IOW ",
            BusState::HALT => "HALT",
            BusState::CODE => "CODE",
            BusState::MEMR => "MEMR",
            BusState::MEMW => "MEMW",
            BusState::PASV => "PASV"           
        };

        let t_str = match self.bus_cycle {
            BusCycle::T1 => "T1",
            BusCycle::T2 => "T2",
            BusCycle::T3 => "T3",
            BusCycle::T4 => "T4",
            BusCycle::Tw => "Tw",
        };

        let is_reading = is_reading!(self.command_status);
        let is_writing = is_writing!(self.command_status);

        let mut xfer_str = "      ".to_string();
        if is_reading {
            xfer_str = format!("<-r {:02X}", self.data_bus);
        }
        else if is_writing {
            xfer_str = format!("w-> {:02X}", self.data_bus);
        }

        // Handle queue activity

        let mut q_read_str = String::new();

        if q_op == QueueOp::First {
            // First byte of opcode read from queue. Decode it to opcode or group specifier
            if self.queue_byte == OPCODE_IRET {

                let iret_addr = self.queue_fetch_addr;
                let isr_base_addr = RemoteCpu::calc_linear_address(ISR_SEGMENT, 0);
                let isr_number = (iret_addr - isr_base_addr) / 2;
                q_read_str = format!("<-q {:02X} {} @ [{:05X}] ISR:{:02X}", self.queue_byte, opcodes::get_opcode_str(self.opcode, 0, false), self.queue_fetch_addr, isr_number);
            }
            else {
                q_read_str = format!("<-q {:02X} {} @ [{:05X}]", self.queue_byte, opcodes::get_opcode_str(self.opcode, 0, false), self.queue_fetch_addr);
            }
        }
        else if q_op == QueueOp::Subsequent {
            if is_group_op(self.opcode) && self.queue_fetch_n == 1 {
                // Modrm was just fetched for a group opcode, so display the mnemonic now
                q_read_str = format!("<-q {:02X} {}", self.queue_byte, opcodes::get_opcode_str(self.opcode, self.queue_byte, true));
            }
            else {
                // Not modrm byte
                q_read_str = format!("<-q {:02X}", self.queue_byte);
            }
        }        
      
        format!(
            "{:08} {:02}[{:05X}] {:02} M:{}{}{} I:{}{}{} Q:{}{} {:04} {:02} {:06} | {:1}{:1} [{:08}] {}",
            self.cycle_num,
            ale_str,
            self.address_latch,
            seg_str,
            rs_chr, aws_chr, ws_chr, ior_chr, aiow_chr, iow_chr,
            intr_chr, inta_chr,
            bus_str,
            t_str,
            xfer_str,
            q_op_chr,
            self.queue.len(),
            self.queue.to_string(),
            q_read_str
        )
    }

    pub fn run(&mut self, cycle_limit: u32, print: bool ) -> Result<RemoteCpuRegisters, bool> {

        self.address_latch = self.client.read_address_latch().expect("Failed to get address latch!");

        self.update_state();

        // ALE should be active at start of execution
        if self.command_status & COMMAND_ALE_BIT == 0 {
            log::warn!("Execution is not starting on T1.");
        }

        if print {
            self.print_cpu_state();
        }

        while self.cycle_num < cycle_limit {
            //self.cycle();
            //if print {
            //    self.print_cpu_state();
            //}

            while self.program_state != ProgramState::ExecuteDone {
                match self.program_state {

                    ProgramState::Execute => {
                        self.cycle();
                        if print {
                            self.print_cpu_state();
                        }
                        
                    }
                    ProgramState::ExecuteFinalize => {
                        self.cycle();
                    }
                    _=> {
                        log::error!("Invalid program state: {:?}!", self.program_state);
                        panic!("Invalid program state!");
                    }
                }

                //log::trace!("Program state: {:?}", self.program_state);
            }


            // Program finalized!
            log::trace!("Program finalized! Run store now.");

            let regs = self.store();

            let err = self.get_last_error();
            log::debug!("flagbuf is {}", err);
            return Ok(regs);

            /*
            if self.mcycle_state == BusState::CODE && (self.address_latch as usize >= self.end_addr) {
                // We are fetching past the end of the program. Send a flagged NOP to finalize execution on fetch.

                if self.program_state == ProgramState::Execute {
                    self.client.finalize().expect("Failed to finalize!");
    
                    // Wait for execution to finalize
                    while self.program_state != ProgramState::ExecuteDone {
                        self.cycle();
                        
                        if self.program_state != ProgramState::ExecuteDone {
                            self.print_cpu_state();
                        }
                    }
    
                    // Program finalized!
                    log::trace!("Program finalized! Run store now.");

                    self.store();
                    
                    break;
                }
            }
            */
        }

        Err(false)
    }

    pub fn store(&mut self) -> RemoteCpuRegisters {

        let mut buf: [u8; 28] = [0; 28];
        self.client.store_registers_to_buf(&mut buf).expect("Failed to store registers!");

        let regs = RemoteCpu::buf_to_regs(&buf);
        //RemoteCpu::print_regs(&regs);
        regs
    }

    pub fn test(&mut self) -> bool {

        let al = match self.client.read_address_latch() {
            Ok(address) => address,
            Err(_) => {
                return false
            }
        };

        log::trace!("Address latch: {:05X}", al);
        true
    }

    pub fn buf_to_regs(r: &[u8]) -> RemoteCpuRegisters {

        //println!("{:0x?}", r);

        RemoteCpuRegisters {
            ax:    (r[0x00] as u16) | ((r[0x01] as u16) << 8),
            bx:    (r[0x02] as u16) | ((r[0x03] as u16) << 8),
            cx:    (r[0x04] as u16) | ((r[0x05] as u16) << 8),
            dx:    (r[0x06] as u16) | ((r[0x07] as u16) << 8),
            ss:    (r[0x08] as u16) | ((r[0x09] as u16) << 8),
            sp:    (r[0x0A] as u16) | ((r[0x0B] as u16) << 8),
            flags: (r[0x0C] as u16) | ((r[0x0D] as u16) << 8),
            ip:    (r[0x0E] as u16) | ((r[0x0F] as u16) << 8),
            cs:    (r[0x10] as u16) | ((r[0x11] as u16) << 8),
            ds:    (r[0x12] as u16) | ((r[0x13] as u16) << 8),
            es:    (r[0x14] as u16) | ((r[0x15] as u16) << 8),
            bp:    (r[0x16] as u16) | ((r[0x17] as u16) << 8),
            si:    (r[0x18] as u16) | ((r[0x19] as u16) << 8),
            di:    (r[0x1A] as u16) | ((r[0x1B] as u16) << 8),
        }
    }

    pub fn print_regs(regs: &RemoteCpuRegisters) {
        
        let reg_str = format!(
          "AX: {:04x} BX: {:04x} CX: {:04x} DX: {:04x}\n\
          SP: {:04x} BP: {:04x} SI: {:04x} DI: {:04x}\n\
          CS: {:04x} DS: {:04x} ES: {:04x} SS: {:04x}\n\
          IP: {:04x}\n\
          FLAGS: {:04x}",
          regs.ax, regs.bx, regs.cx, regs.dx,
          regs.sp, regs.bp, regs.si, regs.di,
          regs.cs, regs.ds, regs.es, regs.ss,
          regs.ip,
          regs.flags );
      
        println!("{}", reg_str);
        /*
        // Expand flag info
        u16 f = regs->flags;
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
      
        Serial.print("FLAGSINFO: ");
        Serial.println(flag_buf);
        */
    }
}