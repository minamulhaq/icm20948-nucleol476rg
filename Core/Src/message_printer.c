

#include "message_printer.h"

#include <string.h>
#include <stdio.h>

// #include "stdio.h"

#define PRINT_BUFFER_SIZE 256
char print_buffer[PRINT_BUFFER_SIZE] = {0};

void print_message(const char* msg) {
    strncpy(print_buffer, msg, sizeof(print_buffer) - 1);
    print_buffer[sizeof(print_buffer) - 1] = '\0';  // Ensure null termination
    HAL_UART_Transmit(&huart2, (uint8_t*)print_buffer, strlen(print_buffer), HAL_MAX_DELAY);
    memset(print_buffer, 0, PRINT_BUFFER_SIZE);  // Ensure the buffer is cleared
}

void print_accel_data(float* accel, uint32_t* timestamp) {
    sprintf(print_buffer, "%10lu, %10.3f, %10.3f, %10.3f, %10.3f, %10.3f, %10.3f\r\n",
            *timestamp, accel[0], accel[1], accel[2], accel[3], accel[4], accel[5]);
    HAL_UART_Transmit(&huart2, (uint8_t*)print_buffer, strlen(print_buffer), HAL_MAX_DELAY);
    memset(print_buffer, 0, sizeof(print_buffer));
}