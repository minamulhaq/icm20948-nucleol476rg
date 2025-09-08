#ifndef __ICM20948_SERIAL_IF_H
#define __ICM20948_SERIAL_IF_H

#include <stdint.h>

typedef struct Context {
    void* handle;
    uint16_t addr;
} Context;

typedef struct ICM20948_Serial_If {
    Context* context;
    int (*read_reg)(void* context, uint8_t reg, uint8_t* buf, uint32_t len);
    int (*write_reg)(void* context, uint8_t reg, uint8_t* buf,
                     uint32_t len);
} ICM20948_Serial_If;

#endif