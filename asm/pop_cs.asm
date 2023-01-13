cpu	8086
org	0h

mov ax, 0xd000
push ax
pop cs
nop
