; program.asm
; Compile with nasm to build program.bin for cpu_client
; nasm program.asm -o program.bin
cpu	8086
org	0h

; We can either specify instructions or raw bytes

sti
mov al, 0x08
out 0xFF, al
nop
nop
nop
nop
nop
nop
nop
nop