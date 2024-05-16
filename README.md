# Arduino8088
![arduino8088_pcb](/images/render_v1_1.png)

### About Arduino8088
I've writtten a blog article that gives an overview of this project and how it is used.

https://martypc.blogspot.com/2023/06/hardware-validating-emulator.html

### Description

This project expands on the basic idea of controlling an Intel 8088 or NEC V20 CPU via GPIO pins to clock the CPU and read and write control and data signals.
This can be used to validate an emulator's accuracy, but also as a general method of exploring the operation of 8088 and V20 instructions and timings.

Where it differs from Raspberry Pi based projects is that it uses an Arduino MEGA2560, DUE or GIGA to supply enough GPIO pins to operate the 8088 in Maximum mode without requiring any shifters. This enables several useful signals to be read such as the QS0 & QS1 processor instruction queue status lines, which give us more insight into the internal state of the CPU. We can also enable inputs such as READY, NMI, INTR, and TEST, so we can in theory execute interrupts, emulate wait states, and perhaps simulate FPU and DMA operations.

The board supports an Intel 8288 bus controller which can produce bus signals, but this chip is optional as the sketch can perform i8288 emulation.

The original Arduino8088 board utilized an Arduino MEGA, but this board is now considered deprecated for this project. As of the current release, an [Arduino DUE](https://store.arduino.cc/products/arduino-due) should be used instead. Although the DUE has 3v GPIO, the current board design is modified for 3V operation.  The 80C88 itself tolerates 3V well, and i8288 emulation can be used if you lack a CMOS 8288.

I have been using this project to validate the cycle-accuracy of my PC emulator, MartyPC: https://github.com/dbalsom/martypc 

## Can the CPU be clocked fast enough?

In short, no. We are well past the published minimum cycle times when executing programs via a serial protocol, cycle by cycle. Some chips tolerate this better than others. When working with an Intel branded 8088, I noticed that effective address calculations were failing to add the displacement or index register, but otherwise functioned. I have had more luck with the AMD second-source 8088 CPUs, which seem to function perfectly under slow clocks although they will hang and need to be reset if not cycled for a several milliseconds. The issue is "dynamic logic" - logic gates that lose their state if not refrehsed electrically within a frequent enough interval. To be absolutely safe, it is best to use a fully CMOS process CPU such as the 80C88. 

## Credits

Inspired by and borrows from the Pi8088 validator created by Andreas Jonsson as part of the VirtualXT project:

https://github.com/andreas-jonsson/virtualxt/tree/develop/tools/validator/pi8088

A very similar project is homebrew8088's Raspberry Pi Hat:

https://github.com/homebrew8088/pi86

## To use

If you don't want to order and build the PCB, connect the GPIO pins to the CPU on a breadboard as specified in the KiCad project schematic.

The main Arduino8088 sketch, cpu_server, operates a simple binary serial protocol to execute operations on the CPU and read and write data, status and control signals. This is designed for integration with an emulator or instruction test generator.

Additionally there is a sketch, 'run_program', that will take any user-supplied register state and array of instruction bytes defined in the source code, execute it, and print cycle traces and final register state. This is useful for investigating the timing and operation of certain instructions without needing any external software integrations, however it is restricted in the number of memory reads or writes it can support, due to the limited RAM on the Arduino MEGA (8k!) 'run_program' does not currently support the DUE.

An example client for cpu_server is provded, written in Rust, in the client directory. It demonstrates how to upload arbitrary code to the Arduino8088 and display cycle traces. The client will emulate the entire address space and set up a basic IVT.

## PCB
![pcb_shield50](/images/pcb_v1_1.png)

KiCad project files for the PCB are supplied. 

In theory the board could also support an 8086 CPU. Version 1.1 adds a connection for the 8086's BHE pin, 
which indicates the size of the current bus transfer.  The cpu_server sketch and protocol still needs modification to support 16 bit data transfers and the longer queue length on the 8086.

# BOM
- A compatible 80C88 or NEC V20 CPU. Beware of counterfeits on eBay and other online vendors.
A legitimate chip will not look shiny and new with perfect printing on it.
- (Optional) An Intel 80C8288 Bus Controller
- A set of Arduino stacking headers (also usable with DUE) 
https://www.amazon.com/Treedix-Stacking-Headers-Stackable-Compatible/dp/B08G4FGBPQ

- A DIP-40 and (optionally) DIP-20 socket
  - Optional: You can spring for a ZIF socket such as [https://www.amazon.com/-/en/gp/product/B00B886OZI](https://www.amazon.com/-/en/gp/product/B00B886OZI)

- (Optional) A 3V, 12mm buzzer <= 30Ma
  https://www.digikey.com/en/products/detail/mallory-sonalert-products-inc/PB-1226PEAQ/1957866
  WARNING: Only connect a buzzer if using an Arduino MEGA.  The DUE has much lower GPIO max current supply.
  
- (2x) 0.047 0805 bypass capacitors
  https://www.digikey.com/en/products/detail/kemet/C0805C473K5RAC7800/411165

I have included optional LEDs on the current revision board but have not chosen models or resistor values 
yet. I will update this document when I do.
