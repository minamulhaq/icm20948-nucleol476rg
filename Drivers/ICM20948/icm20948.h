#ifndef __ICM20948_H
#define __ICM20948_H

#ifdef __cplusplus
extern "C" {
#endif

#include "icm20948_errors.h"
#include "icm20948_mem.h"
#include "icm20948_serial_if.h"
#include "icm2098_defs.h"

#define CONSTRUCT_ACCEL_GYRO_CONFIG(fchoice, fs_sel, dlpfcfg)   \
    (((((fchoice) & 0x01) << 0) | /* Bit 0: FCHOICE */    \
      (((fs_sel) & 0x03) << 1) |  /* Bits 1-2: FS_SEL */  \
      (((dlpfcfg) & 0x07) << 3) | /* Bits 3-5: DLPFCFG */ \
      (0x00 << 6)))               /* Bits 6-7: Reserved (set to 0) */

typedef struct Configuration {
    float accel_sensitivity;
    float gyro_sensitivity;
} Configuration;

typedef struct ICM20948 {
    ICM20948_Serial_If* serial_if;
    uint8_t current_bank;
    Configuration configuration;
    IMUBuffer* imuBuffer;
} ICM20948;

#ifdef __cplusplus
}
#endif

#endif