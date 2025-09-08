#include "icm20948_driver.h"

#include "i2c.h"
#include "icm20948_mem.h"
#include "icm2098_defs.h"
#include "math.h"
#include "message_printer.h"
#include "nucleo_icm20948_i2c_interface.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"

// IMUValues previous
// IMUValues current
// IMUValues values[2] = {{0}, {0}};

void Debug_Handle_Check(ICM20948_Serial_If* sif) {
    uint8_t message[50];
    if ((sif->context)->handle == &hi2c1) {
        snprintf((char*)message, sizeof(message), "Handle is correctly set to &hi2c1\n");
    } else {
        snprintf((char*)message, sizeof(message), "Handle is NOT set correctly!\n");
    }
    HAL_UART_Transmit(&huart2, message, strlen((char*)message), HAL_MAX_DELAY);
}

ICM20948 GET_ICM20948_Device(uint8_t addr) {
    ICM20948_Serial_If* sif = Get_Nucleo_I2C_Serial_IF(addr);
    Debug_Handle_Check(sif);

    ICM20948 device = {
        .serial_if = sif,
        .current_bank = 0x00};
    return device;
}

uint8_t ICM20948_Update_Bank(ICM20948* device, uint16_t reg) {
    uint8_t bank = GET_BANK(reg);
    uint8_t address = 0xFF & reg;
    if (bank != device->current_bank) {
        device->current_bank = bank;
        nucleo_i2c_write(device->serial_if->context, REG_BANK_SEL, &bank, 1);
    }
    return address;
}

int ICM20948_WHO_AM_I(ICM20948* device) {
    uint8_t address = ICM20948_Update_Bank(device, REG_WHO_AM_I);
    uint8_t id = 0xFF;
    HAL_StatusTypeDef rc = device->serial_if->read_reg(device->serial_if->context, address, &id, 1);

    if (rc != HAL_OK) {
        char error_msg[] = "Error: Unable to read IMU ID!\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)error_msg, sizeof(error_msg) - 1, HAL_MAX_DELAY);
        return -1;
    }

    char msg[50];
    if (id == WHOAMI) {
        snprintf(msg, sizeof(msg), "IMU ID successfully read: 0x%02X\r\n", id);
        print_message_uart2(msg);
        return 0;
    } else {
        snprintf(msg, sizeof(msg), "IMU ID mismatch! Read: 0x%02X\r\n", id);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return -1;
    }
}
void ICM20848_InitAngles(ICM20948* device) {
    print_message_uart2("Initializing angles...\r\n");
    uint8_t samples = 20;
    double angle_accel_x = 0.0;
    double angle_accel_z = 0.0;

    for (uint8_t i = 0; i < samples; i++) {
        ICM20948_READ_ACCEL_GYRO_DATA(device);
        ICM20948_Calculate_YawPitchRoll(device);
        IMUValues* current = IMUBuffer_GetCurrentValue(device->imuBuffer);
        angle_accel_x += current->angle_accel[0];
        angle_accel_z += current->angle_accel[1];
    }

    angle_accel_x /= samples;
    angle_accel_z /= samples;
    char msg[50];
    sprintf(msg, "accelx: %f, accel_z: %f\r\n", angle_accel_x, angle_accel_z);
    print_message_uart2(msg);
    IMUValues* values = IMUBuffer_GetCurrentValue(device->imuBuffer);
    values->angle_gyro[0] = angle_accel_x;
    values->angle_gyro[1] = angle_accel_z;
    values->timestamp = HAL_GetTick();
}

/**
 * @brief Initialize IMU
 * Steps:
 * 1. Reset in PowerManagement 1 register
 * 2. Select the clock source
 * @param device
 * @return int
 */
int ICM20948_INIT(ICM20948* device) {
    int rc = 0;
    device->imuBuffer = IMUBuffer_Init(IMU_BUFFER_SIZE);
    ICM20948_WHO_AM_I(device);
    ICM20948_Reset_Device(device);

    ICM20948_Set_Accel_Config(device, DLPFCFG2, MPU_FS_2G, FCHOICE_Enable);
    ICM20948_Set_Gyro_Config(device, DLPFCFG2, MPU_FS_500dps, FCHOICE_Enable);
    ICM20948_Set_Gyro_Config2(device, AVERAGING_128X);

    ICM20948_Set_Accel_Output_Data_Rate(device, 100);
    device->configuration.gyro_sensitivity = SENSITIVITY_GYRO_FS_0;
    ICM20848_InitAngles(device);
    return rc;
}
void Is_IMU_Ready(ICM20948* device) {
#ifdef USE_HAL_DRIVER
    while (HAL_I2C_IsDeviceReady(device->serial_if->context->handle, device->serial_if->context->addr, 5, HAL_MAX_DELAY != HAL_OK)) {
        print_message_uart2("Waiting for IMU to boot up\r\n");
    }
    print_message_uart2("Device ready\r\n");
#endif
}

void ICM20948_Reset_Device(ICM20948* device) {
    print_message_uart2("Resetting IMU\r\n");
    uint8_t address = ICM20948_Update_Bank(device, REG_PWR_MGMT_1);
    uint8_t val = BIT_H_RESET | BIT_SLEEP | BIT_CLK_PLL;
    device->serial_if->write_reg(device->serial_if->context, address, &val, 1);
    HAL_Delay(200);
    val = BIT_CLK_PLL;
    device->serial_if->write_reg(device->serial_if->context, address, &val, 1);
    Is_IMU_Ready(device);
}

void ICM20948_Set_Accel_Output_Data_Rate(ICM20948* device, uint16_t rate) {
    uint16_t val = (1125 / rate) - 1;
    uint8_t msb = (val >> 8) & 0xFF;
    uint8_t lsb = val & 0xFF;
    uint8_t addr = ICM20948_Update_Bank(device, REG_ACCEL_SMPLRT_DIV_1);
    device->serial_if->write_reg(device->serial_if->context, addr, &msb, 1);
    addr = ICM20948_Update_Bank(device, REG_ACCEL_SMPLRT_DIV_2);
    device->serial_if->write_reg(device->serial_if->context, addr, &lsb, 1);
}

void ICM20948_Set_Accel_Config(ICM20948* device, enum dlpfcfg dlpf_cfg, enum mpu_accel_fs accel_fs, enum FCHOICE_STATE fchoice) {
    uint8_t config_val = CONSTRUCT_ACCEL_GYRO_CONFIG(fchoice, accel_fs, dlpf_cfg);
    uint8_t addr = ICM20948_Update_Bank(device, (uint16_t)REG_ACCEL_CONFIG);
    device->serial_if->write_reg(device->serial_if->context, addr, &config_val, sizeof(config_val));
    print_message_uart2("FCHOICE value successfully set\r\n");

    switch (accel_fs) {
        case MPU_FS_2G:
            device->configuration.accel_sensitivity = SENSITIVITY_ACCEL_FS_0;
            break;
        case MPU_FS_4G:
            device->configuration.accel_sensitivity = SENSITIVITY_ACCEL_FS_1;
            break;
        case MPU_FS_8G:
            device->configuration.accel_sensitivity = SENSITIVITY_ACCEL_FS_2;
            break;
        case MPU_FS_16G:
            device->configuration.accel_sensitivity = SENSITIVITY_ACCEL_FS_3;
            break;
        default:
            device->configuration.accel_sensitivity = SENSITIVITY_ACCEL_FS_0;
            break;
    }
}

void ICM20948_Set_Gyro_Config(ICM20948* device, enum dlpfcfg dlpf_cfg, enum mpu_gyro_fs gyro_fs, enum FCHOICE_STATE fchoice) {
    uint8_t config_val = CONSTRUCT_ACCEL_GYRO_CONFIG(fchoice, gyro_fs, dlpf_cfg);
    uint8_t addr = ICM20948_Update_Bank(device, (uint16_t)REG_GYRO_CONFIG_1);
    device->serial_if->write_reg(device->serial_if->context, addr, &config_val, sizeof(config_val));
    switch (gyro_fs) {
        case MPU_FS_250dps:
            device->configuration.gyro_sensitivity = SENSITIVITY_GYRO_FS_0;
            break;
        case MPU_FS_500dps:
            device->configuration.accel_sensitivity = SENSITIVITY_GYRO_FS_1;
            break;
        case MPU_FS_1000dps:
            device->configuration.accel_sensitivity = SENSITIVITY_GYRO_FS_2;
            break;
        case MPU_FS_2000dps:
            device->configuration.accel_sensitivity = SENSITIVITY_GYRO_FS_3;
            break;
        default:
            device->configuration.accel_sensitivity = SENSITIVITY_GYRO_FS_0;
            break;
    }
}

void ICM20948_Set_Gyro_Config2(ICM20948* device, enum averaging avg) {
    uint8_t config_val = avg;
    uint8_t addr = ICM20948_Update_Bank(device, (uint16_t)REG_GYRO_CONFIG_2);
    device->serial_if->write_reg(device->serial_if->context, addr, &config_val, sizeof(config_val));
}

void ICM20948_READ_ACCEL_GYRO_DATA(ICM20948* device) {
    uint8_t reg_addr = ICM20948_Update_Bank(device, REG_ACCEL_XOUT_H_SH);
    uint8_t raw_buffer[12] = {0};
    device->serial_if->read_reg(device->serial_if->context, reg_addr, raw_buffer, 12);
    IMUValues* values = IMUBuffer_GetNextValue(device->imuBuffer);
    values->timestamp = HAL_GetTick();
    int16_t accel_raw_x = (int16_t)((raw_buffer[0] << 8) | raw_buffer[1]);
    int16_t accel_raw_y = (int16_t)((raw_buffer[2] << 8) | raw_buffer[3]);
    int16_t accel_raw_z = (int16_t)((raw_buffer[4] << 8) | raw_buffer[5]);
    int16_t gyro_raw_x = (int16_t)((raw_buffer[6] << 8) | raw_buffer[7]);
    int16_t gyro_raw_y = (int16_t)((raw_buffer[8] << 8) | raw_buffer[9]);
    int16_t gyro_raw_z = (int16_t)((raw_buffer[10] << 8) | raw_buffer[11]);
    values->accel_gyro_data[0] = accel_raw_x / device->configuration.accel_sensitivity;
    values->accel_gyro_data[1] = accel_raw_y / device->configuration.accel_sensitivity;
    values->accel_gyro_data[2] = accel_raw_z / device->configuration.accel_sensitivity;

    float gyro_bias[3] = {0.0, 0, 0, 0.0};
    // float gyro_bias[3] = {-0.11428002f, -0.64344996f, 0.88612f};
    values->accel_gyro_data[3] = gyro_raw_x / device->configuration.gyro_sensitivity - gyro_bias[0];
    values->accel_gyro_data[4] = gyro_raw_y / device->configuration.gyro_sensitivity - gyro_bias[1];
    values->accel_gyro_data[5] = gyro_raw_z / device->configuration.gyro_sensitivity - gyro_bias[2];

    memset(values->angle_accel, 0, sizeof(double) * 3);
    memset(values->angle_gyro, 0, sizeof(double) * 3);

    IMUBuffer_increaseIndex(device->imuBuffer);
}

void ICM20948_Calculate_YawPitchRoll(ICM20948* device) {
    IMUValues* current = IMUBuffer_GetCurrentValue(device->imuBuffer);
    IMUValues* previous = IMUBuffer_GetPreviousValue(device->imuBuffer);

    // Calculate accelerometer angles (Y-axis vertical)
    current->angle_accel[0] = atan2(current->accel_gyro_data[2], current->accel_gyro_data[1]) * 180 / M_PI;   // Pitch
    current->angle_accel[1] = atan2(-current->accel_gyro_data[0], current->accel_gyro_data[1]) * 180 / M_PI;  // Roll

    // Time difference in seconds
    double dt = (current->timestamp - previous->timestamp) / 1000.0;


    current->angle_gyro[0] = previous->angle_gyro[0] + current->accel_gyro_data[3] * dt;
    current->angle_gyro[1] = previous->angle_gyro[1] + current->accel_gyro_data[4] * dt;

    // Fuse angles
    double alpha = 0.98;
    current->fused_angles[0] = alpha * current->angle_gyro[0] + (1 - alpha) * current->angle_accel[0];  // Pitch
    current->fused_angles[1] = alpha * current->angle_gyro[1] + (1 - alpha) * current->angle_accel[1];  // Roll
}

void ICM20948_Calculate_Mean(ICM20948* device) {
    double overall_mean_a = 0.0;
    double overall_mean_g = 0.0;
    double sum_of_squares_a = 0.0;

    // Step 1: Calculate the mean and sum of squares in a single loop
    for (uint8_t i = 0; i < IMU_BUFFER_SIZE; i++) {
        // Calculate the magnitude of the acceleration vector
        double meana = 0.0;
        meana = (device->imuBuffer->buffer[i].accel_gyro_data[0] * device->imuBuffer->buffer[i].accel_gyro_data[0]) +
                (device->imuBuffer->buffer[i].accel_gyro_data[1] * device->imuBuffer->buffer[i].accel_gyro_data[1]) +
                (device->imuBuffer->buffer[i].accel_gyro_data[2] * device->imuBuffer->buffer[i].accel_gyro_data[2]);
        double magnitude_a = sqrt(meana);
        // Accumulate the mean and sum of squares
        overall_mean_a += magnitude_a;
        sum_of_squares_a += magnitude_a * magnitude_a;

        double meang = 0.0;
        meang = (device->imuBuffer->buffer[i].accel_gyro_data[3] * device->imuBuffer->buffer[i].accel_gyro_data[3]) +
                (device->imuBuffer->buffer[i].accel_gyro_data[4] * device->imuBuffer->buffer[i].accel_gyro_data[4]) +
                (device->imuBuffer->buffer[i].accel_gyro_data[5] * device->imuBuffer->buffer[i].accel_gyro_data[5]);
        double magnitude_g = sqrt(meang);
        // Accumulate the mean and sum of squares
        overall_mean_g += magnitude_g;
    }

    // Step 2: Calculate the mean
    overall_mean_a /= (double)IMU_BUFFER_SIZE;
    overall_mean_g /= (double)IMU_BUFFER_SIZE;

    // Step 3: Calculate the variance
    double var = (sum_of_squares_a / (double)IMU_BUFFER_SIZE) - (overall_mean_a * overall_mean_a);
    var = sqrt(var);

    // Step 4: Store the mean and standard deviation
    device->imuBuffer->mean_accel = overall_mean_a;
    device->imuBuffer->mean_gyro = overall_mean_g;

    device->imuBuffer->std_accel = 2 * var;
}
