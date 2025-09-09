#ifndef __MESSAGE_PRINTER_H
#define __MESSAGE_PRINTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "usart.h"

void print_message(const char* msg);
void print_accel_data(float* accel, uint32_t* timestampe);

#ifdef __cplusplus
}
#endif

#endif