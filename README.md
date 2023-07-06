# Arduino8088
![arduino8088_pcb](https://user-images.githubusercontent.com/7229541/217981230-db9d45e3-1733-4521-9f44-7010e94fcdd5.jpg)

This project expands on the basic idea of controlling an Intel 8088 CPU via GPIO pins to clock the CPU and read and write control and data signals.
This can be used to validate an emulator's accuracy, but also as a general method of exploring the operation of 8088 instructions and timings.

I've writtten a blog article that gives an overview of this project and how it is used.
https://martypc.blogspot.com/2023/06/hardware-validating-emulator.html

Where it differs from Raspberry Pi based projects is that it uses an Arduino MEGA2560 to supply enough GPIO pins to operate the 8088 in Maximum mode without requiring any shifters. This enables several useful signals to be read such as the QS0 & QS1 processor instruction queue status lines, which give us more insight into the internal state of the CPU. We can also enable inputs such as READY, NMI, INTR, and TEST, so we can in theory execute interrupts, emulate wait states, and perhaps simulate FPU and DMA operations.

I am utilizing an Intel 8288 bus controller, although in theory one could calculate its outputs by decoding the S0-S2 status lines. 

One disadvantage to this approach may be speed - in theory the Raspberry Pi's GPIO can be toggled very quickly, and integration with an emulator running on the same hardware should take advantage of the native IO. In contrast, controlling the CPU state over a serial connection introduces the serial link as a bottleneck. One mitigation is that programs that set up or read out register state can be hosted completely on the Arduino itself, keeping the serial protocol as brief as possible. Multiple GPIO lines on the Arduino can be read at the same time by direct access to pin registers. In some cases like the AD0-AD7 address/data lines, there is a direct mapping from the value of the data bus to a single pin register.

The main advantage is that this allows CPU hardware validation to occur on a regular PC, which helps me out since the Raspberry Pi and the UI library I use for my emulator do not cooperate. Another advantage is that the Arduino GPIO pins operate at the 8088's desired voltage of +5V. This may improve compatibility.

I have been using this project to validate the cycle-accuracy of my PC emulator, MartyPC: https://github.com/dbalsom/martypc 

## Can the CPU be clocked fast enough?

In short, no. We are well past the published minimum cycle times when executing programs via a serial protocol, cycle by cycle. Some chips tolerate this better than others. When working with an Intel branded 8088, I noticed that effective address calculations were failing to add the displacement or index register, but otherwise functioned. I have had more luck with the AMD second-source 8088 CPUs, which seem to function perfectly under slow clocks although they will hang and need to be reset if not cycled for a several milliseconds. The issue is "dynamic logic" - logic gates that lose their state if not refrehsed electrically within a frequent enough interval. To be absolutely safe, it is best to use a fully CMOS process CPU such as the 80C88. 

## Credits

Inspired by and borrows from the Pi8088 validator created by Andreas Jonsson as part of the VirtualXT project:

https://github.com/andreas-jonsson/virtualxt/tree/develop/tools/validator/pi8088

A very similar project is homebrew8088's Raspberry Pi Hat:

https://github.com/homebrew8088/pi86

## To use

Connect the GPIO pins as specified in the KiCad project schematic.

The main Arduino8088 sketch, cpu_server, operates a simple binary serial protocol to execute operations on the CPU and read and write data, status and control signals. This is designed for integration with an emulator or instruction test generator.

Additionally there is a sketch, run_program, that will take any user-supplied register state and array of instruction bytes defined in the source code, execute it, and print cycle traces and final register state. This is useful for investigating the timing and operation of certain instructions without needing any external software integrations, however it is restricted in the number of memory reads or writes it can support, due to the limited RAM on the Arduino (8k!)

An example client for cpu_server is provded, written in Rust, in the client directory. It demonstrates how to upload arbitrary code to the Arduino8088 and display cycle traces. The client will emulate the entire address space and set up a basic IVT.

## PCB
![pcb_shield50](https://user-images.githubusercontent.com/7229541/209396773-b776a14a-baaf-46df-903d-24367d3a043c.PNG)

KiCad project files for the PCB are supplied. 

In theory the board could also support an 8086 CPU, although the sketches would need to be modified to read all 16 data bus lines. As my main interest is in emulation of the original IBM PC, I don't have any immediate plans to do so, but if there's any interest in this support, let me know.

