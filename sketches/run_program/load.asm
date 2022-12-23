; load.asm
; Original routine by Andreas Jonsson
; https://github.com/andreas-jonsson/virtualxt/tree/develop/tools/validator/pi8088
;
; Assemble with nasm: 
; nasm load.asm -o load.bin

; This routine sets up a known register state. It must be patched by the validator
; program with the desired values replacing the word operands set to 0 here.

; Patching offsets are as follows:

; FLAGS = 0x00;
; BX = 0x0B;
; CX = 0x0E;
; DX = 0x11;
; SS = 0x14;
; DS = 0x19;
; ES = 0x1E;
; SP = 0x23;
; BP = 0x28;
; SI = 0x2D;
; DI = 0x32;
; AX = 0x37;
; IP = 0x3A;
; CS = 0x3C;

cpu	8086
org	0h

dw 0

mov ax, 0
mov ss, ax
mov sp, ax
popf            ; Flags

mov bx, word 0  ; BX
mov cx, word 0  ; CX
mov dx, word 0  ; DX

mov ax, word 0  ; SS
mov ss, ax
mov ax, word 0  ; DS
mov ds, ax
mov ax, word 0  ; ES
mov es, ax
mov ax, word 0  ; SP
mov sp, ax
mov ax, word 0  ; BP
mov bp, ax
mov ax, word 0  ; SI
mov si, ax
mov ax, word 0  ; DI
mov di, ax

mov ax, word 0  ; AX
jmp 0:0         ; CS:IP
