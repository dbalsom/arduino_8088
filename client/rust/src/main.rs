
#[macro_use]
mod arduino_8088_client;
mod remote_cpu;

use std::path::PathBuf;

use clap::Parser;
use crate::arduino_8088_client::*;
use crate::remote_cpu::*;



#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
   
    // The type of CPU connected to the Arduino8088
    #[arg(long, required(true))]
    cpu_type: CpuType,

    // The binary file containing the register data. Produced from an assembly 
    // file 'program_regs.asm'
    #[arg(long, required(true))]
    reg_file: PathBuf,
   
    // The binary file containing the code to execute.
    #[arg(long, required(true))]
    bin_file: PathBuf,
   
    // Specify the address in memory to mount the bin file. This should typically
    // match the address specified by CS:IP, but doesn't have to...
    #[arg(long, required(true))]
    mount_addr: String,

    // Specify the number of wait states for every bus transfer.
    // TODO: Currently no division between memory and IO, should change...
    #[arg(long, required(true))]
    wait_states: u32,

    // Fill the prefetch queue before executing code.
    #[arg(long, required(false))]
    prefetch: bool,

    // Raise the INTR line on N cycles after HLT.
    #[arg(long, required(false))]
    intr_after: u32,

    // Raise the INTR line on the specified cycle #.
    #[arg(long, required(false))]
    intr_on: u32,

    // Raise the NMI line on the specified cycle #.
    #[arg(long, required(false))]
    nmi_on: u32,
}

fn main() {

    env_logger::init();

    let args = Args::parse();

    // Parse commandline arguments
    let mut reg_bytes = std::fs::read(args.reg_file.clone()).unwrap_or_else(|e| {
        eprintln!("Couldn't read register file {:?}: {}", args.reg_file, e);
        std::process::exit(1);
    });

    let bin_bytes = std::fs::read(args.bin_file.clone()).unwrap_or_else(|e| {
        eprintln!("Couldn't read binary file {:?}: {}", args.bin_file, e);
        std::process::exit(1);
    });    

    let mount_addr = u32::from_str_radix(&args.mount_addr, 16).unwrap_or_else(|e| {
        eprintln!("Couldn't parse code mount address '{}': {}", args.mount_addr, e);
        std::process::exit(1);
    });    

    if (mount_addr as usize) > (0xFFFFF as usize - bin_bytes.len()) {
        eprintln!("Specified mount point out of range.");
        std::process::exit(1);
    }

    // Create a cpu_client connection to cpu_server.
    let cpu_client = match CpuClient::init() {
        Ok(ard_client) => {
            println!("Opened connection to Arduino_8088 server!");
            ard_client
        }
        Err(e) => {
            eprintln!("Error connecting to Arduino_8088 server: {e}");
            std::process::exit(1);
        }
    };

    // Create a remote cpu instance using the cpu_client which should now be connected.
    let mut cpu = RemoteCpu::new(
        args.cpu_type,
        cpu_client, 
        args.prefetch,
        args.wait_states, 
        args.intr_on,
        args.intr_after,
        args.nmi_on,
    );

    // Capture initial regs before adjustment.
    let initial_regs = RemoteCpuRegisters::from(reg_bytes.as_slice());

    // Copy the binary to memory
    log::debug!("Mounting program code at: {:05X}", mount_addr);
    cpu.mount_bin(&bin_bytes, mount_addr as usize);

    // Set up IVR table
    cpu.setup_ivr();

    // Load the registers from binary file
    let result = cpu.load_registers_from_buf(&reg_bytes);
    if result {
        log::trace!("Successfully set up registers!");

        println!("Initial register state:");

        RemoteCpu::print_regs(&initial_regs);

        let print_opts = PrintOptions {
            print_pgm: true,
            print_preload: false,
            print_finalize: true,
        };

        //cpu.test();
        match cpu.run(Some(10_000), &print_opts) {
            Ok(regs) => {
                RemoteCpu::print_regs_delta(&initial_regs, &regs);
            }
            Err(_) => {
                log::error!("Program execution failed!");
            }
        }       
    }
    else {
        log::error!("Register setup failed: {}", cpu.get_last_error());
    }

}

pub fn rewind_ip(regs: &mut [u8], amount: u16) {

    // IP is at offset 14 in regs array.
    let mut ip: u16 = regs[14] as u16 | ((regs[15] as u16) << 8);

    ip = ip.wrapping_sub(amount);

    regs[14] = (ip & 0xFF) as u8;
    regs[15] = (ip >> 8) as u8;
}

#[cfg(test)]
mod tests {

    use crate::arduino_8088_client::*;
    use crate::remote_cpu::*;

    #[derive (Copy, Clone)]
    pub struct daa_result {
        ax: u16,
        flags: u16,
        af: bool,
        cf: bool,
        of: bool
    }

    pub fn daa(mut al: u8, mut af: bool, mut cf: bool) -> (u8, bool, bool) {
        let old_al = al;
        //let mut temp16 = al as u16;
        let old_cf = cf;

        if (al & 0x0F) > 9 || af {
            //temp16 = (self.al as u16).wrapping_add(6);
            // self.set_register8(Register8::AL, (temp16 & 0xFF) as u8);
            al = al.wrapping_add(6);

            // Set carry flag on overflow from AL + 6
            //self.set_flag_state(Flag::Carry, old_cf || temp16 & 0xFF00 != 0);
            af = true;
        }
        else {
            af = false;
        }

        // Different sources show this value 0x99 or 0x9F, does it matter?
        // Current intel documents show 0x99
        if (old_al > 0x99) || old_cf {
            //self.set_register8(Register8::AL, temp16.wrapping_add(0x60) as u8);
            al = al.wrapping_add(0x60);
            cf = true;
        }
        else {
            cf = false;
        }

        //self.set_szp_flags_from_result_u8(self.al);

        (al, af, cf)
    }

    #[test]
    fn test_flag_init() {
    // Create a cpu_client connection to cpu_server.

        let mut results = [daa_result{ ax: 0, flags: 0, af: false, cf: false, of: false} ; 512];

        let cpu_client = match CpuClient::init() {
            Ok(ard_client) => {
                println!("Opened connection to Arduino_8088 server!");
                ard_client
            }
            Err(e) => {
                eprintln!("Error connecting to Arduino_8088 server: {e}");
                std::process::exit(1);
            }
        };

        // Create a remote cpu instance using the cpu_client which should now be connected.
        let mut cpu = RemoteCpu::new(CpuType::Intel8088, cpu_client, 0, 0, 0, 0);

        let mut regs = RemoteCpuRegisters {
            ax: 0,
            bx: 0,
            cx: 0,
            dx: 0,
            ss: 0,
            ds: 0,
            es: 0,
            sp: 0xFFFF,
            bp: 0,
            si: 0,
            di: 0,
            cs: 0xF000,
            ip: 0x0000,
            flags: 0xF002
        };

        // Load the registers from struct
        let result = cpu.load_registers_from_struct(&regs);
        if result {

            log::trace!("Successfully set up registers!");

            // Load opcode into memory at cs:ip
            let pc = (regs.cs as usize) << 4 + regs.ip as usize;
            cpu.write_u8( pc, 0x90); // NOP
            cpu.set_program_bounds(pc, pc+1);

            cpu.test();
            match cpu.run(Some(100), &[], &PrintOptions::default()) {
                Ok(regs) => {

                    println!("Flags: {:04X}", regs.flags);
                }
                Err(_) => {
                    log::error!("Program execution failed!");
                }
            }
        }
        else {
            log::error!("Register setup failed: {}", cpu.get_last_error());
        }        
        
    }

    #[test]
    fn test_daa() {
    // Create a cpu_client connection to cpu_server.

        let mut results = [daa_result{ ax: 0, flags: 0, af: false, cf: false, of: false} ; 512];

        let cpu_client = match CpuClient::init() {
            Ok(ard_client) => {
                println!("Opened connection to Arduino_8088 server!");
                ard_client
            }
            Err(e) => {
                eprintln!("Error connecting to Arduino_8088 server: {e}");
                std::process::exit(1);
            }
        };

        // Create a remote cpu instance using the cpu_client which should now be connected.
        let mut cpu = RemoteCpu::new(CpuType::Intel8088, cpu_client, 0, 0, 0, 0);

        let cf = true;

        for af in 0..2 {
            for i in 0..256 {
                //println!("i:{}", i);
                let mut regs = RemoteCpuRegisters {
                    ax: i as u16,
                    bx: 0,
                    cx: 0,
                    dx: 0,
                    ss: 0,
                    ds: 0,
                    es: 0,
                    sp: 0xFFFF,
                    bp: 0,
                    si: 0,
                    di: 0,
                    cs: 0xF000,
                    ip: 0x0000,
                    flags: 0
                };
                
                regs.flags &= !CPU_FLAG_AUX_CARRY;
                
                if cf {
                    regs.flags |= CPU_FLAG_CARRY;
                }

                if af == 1 {
                    regs.flags |= CPU_FLAG_AUX_CARRY;
                }
            
                // Load the registers from struct
                let result = cpu.load_registers_from_struct(&regs);
                if result {

                    log::trace!("Successfully set up registers!");

                    // Load opcode into memory at cs:ip
                    let pc = (regs.cs as usize) << 4 + regs.ip as usize;
                    cpu.write_u8( pc, 0x27); // DAA
                    cpu.set_program_bounds(pc, pc+1);

                    cpu.test();
                    match cpu.run(Some(100), &[], &PrintOptions::default()) {
                        Ok(regs) => {

                            let idx = i +  (256 * af);
                            println!("idx: {}", idx);
                            results[idx].ax = regs.ax;
                            results[idx].flags = regs.flags;
                            results[idx].af = regs.flags & CPU_FLAG_AUX_CARRY != 0;
                            results[idx].cf = regs.flags & CPU_FLAG_CARRY != 0;
                            results[idx].of = regs.flags & CPU_FLAG_OVERFLOW != 0;
                            //RemoteCpu::print_regs(&regs);
                        }
                        Err(_) => {
                            log::error!("Program execution failed!");
                        }
                    }
                }
                else {
                    log::error!("Register setup failed: {}", cpu.get_last_error());
                }
            }
        }

        for i in 0..256 {

            let (d_al, d_cf, d_af) = daa(((i & 0xFF) as u8), false, cf);
            println!("{:04X} (af==0): ax: {:04X} flags: {:04X} af: {} cf: {} of: {}  | daa(): ax: {:04x} af: {} cf:{} of: {}", 
                i & 0xFF, 
                results[i].ax, 
                results[i].flags,
                results[i].af, 
                results[i].cf, 
                results[i].of,
                d_al as u16,
                d_af,
                d_cf,
                false
            );
        }
        for i in 256..512 {
            let (d_al, d_cf, d_af) = daa(((i & 0xFF) as u8), true, cf);
            println!("{:04X} (af==1): ax: {:04X} flags: {:04X} af: {} cf: {} of: {}  | daa(): ax: {:04x} af: {} cf:{} of: {}", 
                i & 0xFF, 
                results[i].ax, 
                results[i].flags,
                results[i].af, 
                results[i].cf, 
                results[i].of,
                d_al as u16,
                d_af,
                d_cf,
                false
            );
        }        
    }   


    #[test]
    fn test_das() {
    // Create a cpu_client connection to cpu_server.

        let mut results = [daa_result{ ax: 0, flags: 0, af: false, cf: false, of: false} ; 512];

        let cpu_client = match CpuClient::init() {
            Ok(ard_client) => {
                println!("Opened connection to Arduino_8088 server!");
                ard_client
            }
            Err(e) => {
                eprintln!("Error connecting to Arduino_8088 server: {e}");
                std::process::exit(1);
            }
        };

        // Create a remote cpu instance using the cpu_client which should now be connected.
        let mut cpu = RemoteCpu::new(CpuType::Intel8088, cpu_client, 0, 0, 0, 0);

        let cf = true;

        for af in 0..2 {
            for i in 0..256 {
                //println!("i:{}", i);
                let mut regs = RemoteCpuRegisters {
                    ax: i as u16,
                    bx: 0,
                    cx: 0,
                    dx: 0,
                    ss: 0,
                    ds: 0,
                    es: 0,
                    sp: 0xFFFF,
                    bp: 0,
                    si: 0,
                    di: 0,
                    cs: 0xF000,
                    ip: 0x0000,
                    flags: 0
                };
                
                regs.flags &= !CPU_FLAG_AUX_CARRY;
                
                if cf {
                    regs.flags |= CPU_FLAG_CARRY;
                }

                if af == 1 {
                    regs.flags |= CPU_FLAG_AUX_CARRY;
                }
            
                // Load the registers from struct
                let result = cpu.load_registers_from_struct(&regs);
                if result {

                    log::trace!("Successfully set up registers!");

                    // Load opcode into memory at cs:ip
                    let pc = (regs.cs as usize) << 4 + regs.ip as usize;
                    cpu.write_u8( pc, 0x2F); // DAS
                    cpu.set_program_bounds(pc, pc+1);

                    cpu.test();
                    match cpu.run(Some(100), &[], &PrintOptions::default()) {
                        Ok(regs) => {

                            let idx = i +  (256 * af);
                            println!("idx: {}", idx);
                            results[idx].ax = regs.ax;
                            results[idx].flags = regs.flags;
                            results[idx].af = regs.flags & CPU_FLAG_AUX_CARRY != 0;
                            results[idx].cf = regs.flags & CPU_FLAG_CARRY != 0;
                            results[idx].of = regs.flags & CPU_FLAG_OVERFLOW != 0;
                            //RemoteCpu::print_regs(&regs);
                        }
                        Err(_) => {
                            log::error!("Program execution failed!");
                        }
                    }
                }
                else {
                    log::error!("Register setup failed: {}", cpu.get_last_error());
                }
            }
        }

        for i in 0..256 {

            println!("{:04X} (af==0): ax: {:04X} flags: {:04X} af: {} cf: {} of: {}", 
                i & 0xFF, 
                results[i].ax, 
                results[i].flags,
                results[i].af, 
                results[i].cf, 
                results[i].of
            );
        }
        for i in 256..512 {

            println!("{:04X} (af==1): ax: {:04X} flags: {:04X} af: {} cf: {} of: {}", 
                i & 0xFF, 
                results[i].ax, 
                results[i].flags,
                results[i].af, 
                results[i].cf, 
                results[i].of
            );
        }        
    }       

    #[derive (Copy, Clone)]
    pub struct aaa_result {
        ax: u16,
        flags: u16,
        of: bool,
        sf: bool,
        zf: bool,
        pf: bool
    }

    #[test]
    fn test_aaa() {
    // Create a cpu_client connection to cpu_server.

        let mut results = [aaa_result{ ax: 0, flags: 0, of: false, sf: false, zf: false, pf: false} ; 512];

        let cpu_client = match CpuClient::init() {
            Ok(ard_client) => {
                println!("Opened connection to Arduino_8088 server!");
                ard_client
            }
            Err(e) => {
                eprintln!("Error connecting to Arduino_8088 server: {e}");
                std::process::exit(1);
            }
        };

        // Create a remote cpu instance using the cpu_client which should now be connected.
        let mut cpu = RemoteCpu::new(CpuType::Intel8088, cpu_client, 0, 0, 0, 0);

        let cf = true;

        for af in 0..2 {
            for i in 0..256 {
                //println!("i:{}", i);
                let mut regs = RemoteCpuRegisters {
                    ax: i as u16,
                    bx: 0,
                    cx: 0,
                    dx: 0,
                    ss: 0,
                    ds: 0,
                    es: 0,
                    sp: 0xFFFF,
                    bp: 0,
                    si: 0,
                    di: 0,
                    cs: 0xF000,
                    ip: 0x0000,
                    flags: 0
                };
                
                regs.flags &= !CPU_FLAG_AUX_CARRY;
                
                if cf {
                    regs.flags |= CPU_FLAG_CARRY;
                }

                if af == 1 {
                    regs.flags |= CPU_FLAG_AUX_CARRY;
                }
            
                // Load the registers from struct
                let result = cpu.load_registers_from_struct(&regs);
                if result {

                    log::trace!("Successfully set up registers!");

                    // Load opcode into memory at cs:ip
                    let pc = (regs.cs as usize) << 4 + regs.ip as usize;
                    cpu.write_u8( pc, 0x37); // AAA
                    cpu.set_program_bounds(pc, pc+1);

                    cpu.test();
                    match cpu.run(Some(100), &[], &PrintOptions::default()) {
                        Ok(regs) => {

                            let idx = i +  (256 * af);
                            println!("idx: {}", idx);
                            results[idx].ax = regs.ax;
                            results[idx].flags = regs.flags;
                            results[idx].of = regs.flags & CPU_FLAG_OVERFLOW != 0;
                            results[idx].sf = regs.flags & CPU_FLAG_SIGN != 0;
                            results[idx].zf = regs.flags & CPU_FLAG_ZERO != 0;
                            results[idx].pf = regs.flags & CPU_FLAG_PARITY != 0;
                            //RemoteCpu::print_regs(&regs);
                        }
                        Err(_) => {
                            log::error!("Program execution failed!");
                        }
                    }
                }
                else {
                    log::error!("Register setup failed: {}", cpu.get_last_error());
                }
            }
        }

        for i in 0..256 {

            println!("{:04X} (af==0): ax: {:04X} flags: {:04X} of: {} sf: {} zf: {} pf: {}", 
                i & 0xFF, 
                results[i].ax, 
                results[i].flags,
                results[i].of, 
                results[i].sf, 
                results[i].zf,
                results[i].pf
            );
        }
        for i in 256..512 {

            println!("{:04X} (af==1): ax: {:04X} flags: {:04X} of: {} sf: {} zf: {} pf: {}", 
                i & 0xFF, 
                results[i].ax, 
                results[i].flags,
                results[i].of, 
                results[i].sf, 
                results[i].zf,
                results[i].pf
            );
        }        
    }      


    #[test]
    fn test_aas() {
    // Create a cpu_client connection to cpu_server.

        let mut results = [aaa_result{ ax: 0, flags: 0, of: false, sf: false, zf: false, pf: false} ; 512];

        let cpu_client = match CpuClient::init() {
            Ok(ard_client) => {
                println!("Opened connection to Arduino_8088 server!");
                ard_client
            }
            Err(e) => {
                eprintln!("Error connecting to Arduino_8088 server: {e}");
                std::process::exit(1);
            }
        };

        // Create a remote cpu instance using the cpu_client which should now be connected.
        let mut cpu = RemoteCpu::new(CpuType::Intel8088, cpu_client, 0, 0, 0, 0);

        let cf = true;

        for af in 0..2 {
            for i in 0..256 {
                //println!("i:{}", i);
                let mut regs = RemoteCpuRegisters {
                    ax: i as u16,
                    bx: 0,
                    cx: 0,
                    dx: 0,
                    ss: 0,
                    ds: 0,
                    es: 0,
                    sp: 0xFFFF,
                    bp: 0,
                    si: 0,
                    di: 0,
                    cs: 0xF000,
                    ip: 0x0000,
                    flags: 0
                };
                
                regs.flags &= !CPU_FLAG_AUX_CARRY;
                
                if cf {
                    regs.flags |= CPU_FLAG_CARRY;
                }

                if af == 1 {
                    regs.flags |= CPU_FLAG_AUX_CARRY;
                }
            
                // Load the registers from struct
                let result = cpu.load_registers_from_struct(&regs);
                if result {

                    log::trace!("Successfully set up registers!");

                    // Load opcode into memory at cs:ip
                    let pc = (regs.cs as usize) << 4 + regs.ip as usize;
                    cpu.write_u8( pc, 0x3F); // AAS
                    cpu.set_program_bounds(pc, pc+1);

                    cpu.test();
                    match cpu.run(Some(100), &[], &PrintOptions::default()) {
                        Ok(regs) => {

                            let idx = i +  (256 * af);
                            println!("idx: {}", idx);
                            results[idx].ax = regs.ax;
                            results[idx].flags = regs.flags;
                            results[idx].of = regs.flags & CPU_FLAG_OVERFLOW != 0;
                            results[idx].sf = regs.flags & CPU_FLAG_SIGN != 0;
                            results[idx].zf = regs.flags & CPU_FLAG_ZERO != 0;
                            results[idx].pf = regs.flags & CPU_FLAG_PARITY != 0;
                            //RemoteCpu::print_regs(&regs);
                        }
                        Err(_) => {
                            log::error!("Program execution failed!");
                        }
                    }
                }
                else {
                    log::error!("Register setup failed: {}", cpu.get_last_error());
                }
            }
        }

        for i in 0..256 {

            println!("{:04X} (af==0): ax: {:04X} flags: {:04X} of: {} sf: {} zf: {} pf: {}", 
                i & 0xFF, 
                results[i].ax, 
                results[i].flags,
                results[i].of, 
                results[i].sf, 
                results[i].zf,
                results[i].pf
            );
        }
        for i in 256..512 {

            println!("{:04X} (af==1): ax: {:04X} flags: {:04X} of: {} sf: {} zf: {} pf: {}", 
                i & 0xFF, 
                results[i].ax, 
                results[i].flags,
                results[i].of, 
                results[i].sf, 
                results[i].zf,
                results[i].pf
            );
        }        
    }         
}