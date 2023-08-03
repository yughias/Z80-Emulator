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

uint8_t* getReadAddress(uint16_t);
uint8_t* getWriteAddress(uint16_t);
uint8_t* getReadIO(uint16_t);
uint8_t* getWriteIO(uint16_t);

#endif