#ifndef __ICM20948_ERRORS_H
#define __ICM20948_ERRORS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ICM20948_STATUS_ERROR   = -1,  /* Generic failure */
    ICM20948_STATUS_SUCCESS =  0   /* Operation successful */
} ICM20948_STATUS;

#ifdef __cplusplus
}
#endif

#endif /* __ICM20948_ERRORS_H */