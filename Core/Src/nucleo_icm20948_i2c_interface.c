#include "nucleo_icm20948_i2c_interface.h"
#include "message_printer.h"

ICM20948_Serial_If* Get_Nucleo_I2C_Serial_IF(uint8_t addr) {
    // Dynamically allocate memory for the Nucleo_I2C_Context
    Context* nucleo_context = (Context*)malloc(sizeof(Context));
    nucleo_context->handle = &hi2c1;
    nucleo_context->addr = addr << 1;  // Shift the 7-bit address to 8-bit

    // Dynamically allocate memory for the ICM20948_Serial_If
    ICM20948_Serial_If* i2c_if =
        (ICM20948_Serial_If*)malloc(sizeof(ICM20948_Serial_If));
    i2c_if->context = nucleo_context;  // Assign dynamically allocated context
    i2c_if->read_reg = nucleo_i2c_read;
    i2c_if->write_reg = nucleo_i2c_write;
    i2c_if->delay_ms = HAL_Delay;

    return i2c_if;
}

void Is_IMU_Ready(ICM20948* device) {
#ifdef USE_HAL_DRIVER
    while (HAL_I2C_IsDeviceReady(device->serial_if->context->handle,
                                 device->serial_if->context->addr, 5,
                                 HAL_MAX_DELAY != HAL_OK)) {
        print_message("Waiting for IMU to boot up\r\n");
    }
    print_message("Device ready\r\n");
#endif
}