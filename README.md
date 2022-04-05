# 6502_emulator
A simple 6502 written in C programming language.

# Project Status:
1. This emulator was originally created for a nes emulator.
2. It is still in development phase.
3. The emulator could log disassembly of the binary code.
See the "log.txt" file after running the program.

# How to buid:
1. The default binary file is set to "/ROM/nestest.nes" in the "file.c".
Search and edit line No:13 in "file.c", to add your binary file.
2. To build the project:
$ make cc
3. Run the project:
$ make rr

# Bugs:
1. The disassembler would fall in non-terminating or long loops if the binary code contains such looping jump instructions.
In these case you need to manually terminate the program.
2. The "log.txt" file could grow very large on longer execution.

**Please report any bugs by raising an issue on the repository.**
