#include "i2c_bus_scanner.h"
#include "stdio.h"
#include "string.h"
#include "message_printer.h"
#include "stm32l4xx_hal.h"
extern I2C_HandleTypeDef hi2c1;

void I2C_Scan_Bus(void){
    char msg[64];  // Buffer for UART messages
    uint8_t i2c_address;

    print_message("\nScanning I2C bus...\r\n");

    for (i2c_address = 1; i2c_address < 128; i2c_address++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (i2c_address << 1), 1, 10) ==
            HAL_OK) {
            sprintf(msg, "..... [0x%02X] .....\r\n", i2c_address);
            print_message(msg);
        }
    }

    print_message("\nI2C scan complete.\r\n");
}