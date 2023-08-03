# Z80-Emulator

This is a simple Z80 emulator written purely in C that pass the following text:
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
Every read/write function in the memory/IO bus must have the following signature:

```c
uint8_t* access_bus(uint16_t address);
```

This function must return a pointer to the adress accessed during a read/write operation.
