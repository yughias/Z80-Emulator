# Z80-Emulator

This is a simple Z80 emulator written purely in C that passes the following tests:
- prelim.com
- zexdoc.cim
- zexall.com

To create an instance of your z80, just do the following:

```c
z80_t cpu {
    .readMemory = read_memory_func,
    .writeMemory = write_memory_func,
    .readIO = read_io_func,
    .writeIO = write_io_func
};
```
Every read function in the memory/IO bus must have the following signature:

```c
uint8_t read_byte(z80_t* z80, uint16_t address);
```

Every write function in the memory/IO bus must have the following signature:

```c
void access_bus(z80_t* z80, uint16_t address, uint8_t byte);
```
