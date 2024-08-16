#ifndef _MEMORY_H_
#define _MEMORY_H_

#include <stdint.h>

#define IO_SIZE 256

#define MEMORY_SIZE 0x10000

// I/0 SPACE
extern uint8_t IO[IO_SIZE];

//RAM
extern uint8_t* MEMORY;

void initMemory();
void loadROM(const char*);
void freeMemory();

uint8_t readMemory(uint16_t);
void writeMemory(uint16_t, uint8_t);
uint8_t readIO(uint16_t);
void writeIO(uint16_t, uint8_t);


#endif