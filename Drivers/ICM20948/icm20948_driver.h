#ifndef _ICM20948_DRIVER_H
#define _ICM20948_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "icm20948.h"
#include "icm20948_errors.h"
#include "icm20948_mem.h"
#include "stdbool.h"

#define GET_BANK(reg) ((reg) >> 7)

ICM20948 GET_ICM20948_Device(ICM20948_Serial_If* sif);
int ICM20948_INIT(ICM20948 *device);
ICM20948_STATUS ICM20948_WHO_AM_I(ICM20948 *device);
uint8_t ICM20948_Update_Bank(ICM20948 *device, uint16_t reg);

void ICM20948_Reset_Device(ICM20948 *device);

void ICM20948_Set_Accel_Config(ICM20948 *device, enum dlpfcfg dlpf_cfg,
                               enum mpu_accel_fs accel_fs,
                               enum FCHOICE_STATE fchoice);
void ICM20948_Set_Gyro_Config(ICM20948 *device, enum dlpfcfg dlpf_cfg,
                              enum mpu_gyro_fs gyro_fs,
                              enum FCHOICE_STATE fchoice);
void ICM20948_Set_Gyro_Config2(ICM20948 *device, enum averaging avg);

void ICM20948_Set_Accel_Output_Data_Rate(ICM20948 *device, uint16_t rate);

void ICM20948_READ_ACCEL_GYRO_DATA(ICM20948 *device);
void ICM20948_Calculate_YawPitchRoll(ICM20948 *device);
void ICM20948_Calculate_Mean(ICM20948 *device);

#ifdef __cplusplus
}
#endif

#endif