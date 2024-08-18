#include <memory.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// I/0 SPACE
uint8_t IO[IO_SIZE];

// MEMORY
uint8_t* MEMORY;

void initMemory(){
    memset(IO, 0, IO_SIZE);
    MEMORY = malloc(MEMORY_SIZE);
}

void freeMemory(){
    free(MEMORY);
}

uint8_t readMemory(z80_t* z80, uint16_t addr){
    return MEMORY[addr];
}

void writeMemory(z80_t* z80, uint16_t addr, uint8_t val){
    MEMORY[addr] = val;
}

uint8_t readIO(z80_t* z80, uint16_t addr){
    return IO[0];
}

void writeIO(z80_t* z80, uint16_t addr, uint8_t val){
    IO[0] = val;
}

void loadROM(const char* filename){
    FILE* fptr = fopen(filename, "rb");
    fseek(fptr, 0, SEEK_END);
    size_t size = ftell(fptr);
    rewind(fptr);
    fread(MEMORY+0x100, 1, size, fptr);
    fclose(fptr);
}