#ifndef __ICM20948_MEM_H
#define __ICM20948_MEM_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define IMU_BUFFER_SIZE 10

typedef struct
{
    double accel_gyro_data[6];
    double angle_accel[3];
    double angle_gyro[3];
    double fused_angles[3];
    uint32_t timestamp;
} IMUValues;

typedef struct
{
    uint16_t current;
    uint16_t previous;
    IMUValues *buffer;
    double mean_accel;
    double mean_gyro;
    double std_accel;
    double std_gyro;
    uint16_t size;
} IMUBuffer;

IMUBuffer *IMUBuffer_Init(uint16_t size);
void IMUBuffer_Free(IMUBuffer *buf);
bool IMUBuffer_Push(IMUBuffer *buf, IMUValues *values);
uint16_t IMUBuffer_GetSize(IMUBuffer *buf);
void IMUBuffer_increaseIndex(IMUBuffer *buf);
IMUValues *IMUBuffer_GetPreviousValue(IMUBuffer *buf);
IMUValues *IMUBuffer_GetCurrentValue(IMUBuffer *buf);
IMUValues *IMUBuffer_GetNextValue(IMUBuffer *buf);

#endif