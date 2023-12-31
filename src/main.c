#include <stdio.h>
#include <memory.h>
#include <z80.h>

#define PRELIM_CYCLES 8721
#define ZEXDOC_CYCLES 46734978649
#define ZEXALL_CYCLES 46734978649

z80_t z80 = {
    .readMemory  = getReadAddress,
    .writeMemory = getWriteAddress,
    .readIO      = getReadIO,
    .writeIO     = getWriteIO
};

void startTest(const char*, unsigned long long);
void emulateTest();

int main(){
    initMemory();

    MEMORY[0x0000] = 0xD3;
    MEMORY[0x0001] = 0x00;
    MEMORY[0x0005] = 0xDB;
    MEMORY[0x0006] = 0x00;
    MEMORY[0x0007] = 0xC9;
    
    //freopen("log.txt", "wb", stderr);
    
    startTest("ROM/prelim.com", PRELIM_CYCLES);
    startTest("ROM/zexdoc.cim", ZEXDOC_CYCLES);
    startTest("ROM/zexall.com", ZEXALL_CYCLES);

    freeMemory();
}

void startTest(const char* filename, unsigned long long expected_cycles){
    loadROM(filename);
    initCPU(&z80);
    *z80.PC = 0x100;
    emulateTest();
    printf("\n\nexpected: %llu \t emulated: %llu\n\n", expected_cycles, z80.cycles);
    
}

void emulateTest(){
    for(;;){
        //infoCPU(&z80);
        stepCPU(&z80);

        if(*z80.PC == 0x5)
            if(*z80.C == 0x2)
                printf("0x%2X ", *z80.E);
            else if(*z80.C == 0x9){
                for(int i = *z80.DE; MEMORY[i] != '$'; i++)
                    printf("%c", MEMORY[i]);
                stepCPU(&z80);
            }
        
        if(*z80.PC == 0){
            stepCPU(&z80);
            break;
        }
    }
}