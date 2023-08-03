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

uint8_t* getReadAddress(uint16_t address){
    return MEMORY + address;
}

uint8_t* getWriteAddress(uint16_t address){
    return MEMORY + address;
}

uint8_t* getReadIO(uint16_t address){
    return IO;
}

uint8_t* getWriteIO(uint16_t address){
    return IO;
}

void loadROM(const char* filename){
    FILE* fptr = fopen(filename, "rb");
    fseek(fptr, 0, SEEK_END);
    size_t size = ftell(fptr);
    rewind(fptr);
    fread(MEMORY+0x100, 1, size, fptr);
    fclose(fptr);
}