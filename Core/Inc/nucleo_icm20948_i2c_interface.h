#ifndef __NUCLEO_ICM20948_I2C_INTERFACE_H
#define __NUCLEO_ICM20948_I2C_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32l4xx_hal.h"
#include "i2c.h"
#include "stdio.h"
#include "stdlib.h"
#include "usart.h"
#include "string.h"
#include "icm20948_serial_if.h"
#include "icm20948.h"


void Is_IMU_Ready(ICM20948 *device);

static inline int nucleo_i2c_read(void* ctx, uint8_t reg, uint8_t* buf, uint32_t len) {
    Context* context = (Context*)ctx;

    HAL_StatusTypeDef rc = HAL_I2C_Mem_Read(context->handle, context->addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, HAL_MAX_DELAY);

    // if (rc != HAL_OK) {
    //     uint8_t message[50];
    //     snprintf((char*)message, sizeof(message), "I2C Read Error: %d\n", rc);
    //     HAL_UART_Transmit(&huart2, message, strlen((char*)message), HAL_MAX_DELAY);
    // }
    return rc;
}

static inline int nucleo_i2c_write(void* ctx, uint8_t reg, uint8_t* buf, uint32_t len) {
    Context* context = (Context*)ctx;

    // HAL_StatusTypeDef rc = HAL_I2C_Mem_Write(context->handle, context->addr, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)buf, len, HAL_MAX_DELAY);
    HAL_StatusTypeDef rc = HAL_I2C_Mem_Write(
        context->handle,
        context->addr,
        (uint16_t) reg,
        sizeof(uint8_t),
        buf,
        len,
        HAL_MAX_DELAY
    );

    // if (rc != HAL_OK) {
    //     uint8_t message[50];
    //     snprintf((char*)message, sizeof(message), "I2C Write Error: %d\n", rc);
    //     HAL_UART_Transmit(&huart2, message, strlen((char*)message), HAL_MAX_DELAY);
    // }
    return rc;
}




ICM20948_Serial_If* Get_Nucleo_I2C_Serial_IF(uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif