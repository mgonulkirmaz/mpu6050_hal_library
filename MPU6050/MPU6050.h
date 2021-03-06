/*
 * MPU6050.h
 *
 *  Created on: Aug 16, 2021
 *      Author: mgk
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#ifdef __cplusplus
extern "C" {
#endif

// INCLUDES
#include <stdint.h>

#include "stm32f4xx_hal.h"

// DEFINES

#ifndef MPU6050_ADDR
#define MPU6050_ADDR 0x68
#endif

#define MPU6050_CALIBRATION_CNT 1000
#define RAD_TO_DEG 57.2957795

// REGISTERS AND MASKS

#define SELF_TEST_X 0x0D
#define SELF_TEST_Y 0x0E
#define SELF_TEST_Z 0x0F
#define SELF_TEST_A 0x10
#define SMPRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define FIFO_EN 0x23
#define I2C_MST_CTRL 0x24
#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV1_ADDR 0x28
#define I2C_SLV1_REG 0x29
#define I2C_SLV1_CTRL 0x2A
#define I2C_SLV2_ADDR 0x2B
#define I2C_SLV2_REG 0x2C
#define I2C_SLV2_CTRL 0x2D
#define I2C_SLV3_ADDR 0x2E
#define I2C_SLV3_REG 0x2F
#define I2C_SLV3_CTRL 0x30
#define I2C_SLV4_ADDR 0x31
#define I2C_SLV4_REG 0x32
#define I2C_SLV4_DO 0x33
#define I2C_SLV4_CTRL 0x34
#define I2C_SLV4_DI 0x35
#define I2C_MST_STATUS 0x36
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define INT_STATUS 0x3A
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define I2C_SLV0_DO 0x63
#define I2C_SLV1_DO 0x64
#define I2C_SLV2_DO 0x65
#define I2C_SLV3_DO 0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET 0x68
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define FIFO_COUNTH 0x72
#define FIFO_COUNTL 0x73
#define FIFO_R_W 0x74
#define WHO_AM_I 0x75

#define FS_SEL_0 (0x00 << 3)
#define FS_SEL_1 (0x01 << 3)
#define FS_SEL_2 (0x02 << 3)
#define FS_SEL_3 (0x03 << 3)
#define AFS_SEL_0 (0x00 << 3)
#define AFS_SEL_1 (0x01 << 3)
#define AFS_SEL_2 (0x02 << 3)
#define AFS_SEL_3 (0x03 << 3)
#define INT_LEVEL_0 (0x00 << 7)
#define INT_LEVEL_1 (0x01 << 7)
#define INT_OPEN_0 (0x00 << 6)
#define INT_OPEN_1 (0x01 << 6)
#define LATCH_INT_EN_0 (0x00 << 5)
#define LATCH_INT_EN_1 (0x01 << 5)
#define INT_RD_CLEAR_0 (0x00 << 4)
#define INT_RD_CLEAR_1 (0x01 << 4)
#define FSYNC_INT_LEVEL_0 (0x00 << 3)
#define FSYNC_INT_LEVEL_1 (0x01 << 3)
#define FSYNC_INT_EN_0 (0x00 << 2)
#define FSYNC_INT_EN_1 (0x01 << 2)
#define I2C_BYPASS_EN_0 (0x00 << 1)
#define I2C_BYPASS_EN_1 (0x01 << 1)
#define FIFO_OFLOW_EN_0 (0x00 << 4)
#define FIFO_OFLOW_EN_1 (0x01 << 4)
#define I2C_MST_INT_EN_0 (0x00 << 3)
#define I2C_MST_INT_EN_1 (0x01 << 3)
#define DATA_RDY_EN_0 (0x00 << 0)
#define DATA_RDY_EN_1 (0x01 << 0)
#define FIFO_EN_0 (0x00 << 6)
#define FIFO_EN_1 (0x01 << 6)
#define I2C_MST_EN_0 (0x00 << 5)
#define I2C_MST_EN_1 (0x01 << 5)
#define I2C_IF_DIS_0 (0x00 << 4)
#define I2C_IF_DIS_1 (0x01 << 4)
#define FIFO_RESET_0 (0x00 << 2)
#define FIFO_RESET_1 (0x01 << 2)
#define I2C_MST_RESET_0 (0x00 << 1)
#define I2C_MST_RESET_1 (0x01 << 1)
#define SIG_COND_RESET_0 (0x00 << 0)
#define SIG_COND_RESET_1 (0x01 << 0)
#define DEVICE_RESET_0 (0x00 << 7)
#define DEVICE_RESET_1 (0x01 << 7)
#define SLEEP_0 (0x00 << 6)
#define SLEEP_1 (0x01 << 6)
#define CYCLE_0 (0x00 << 5)
#define CYCLE_1 (0x01 << 5)
#define TEMP_DIS_0 (0x00 << 3)
#define TEMP_DIS_1 (0x01 << 3)
#define CLKSEL_0 (0x00 << 0)
#define CLKSEL_1 (0x01 << 0)
#define CLKSEL_2 (0x02 << 0)
#define CLKSEL_3 (0x03 << 0)
#define CLKSEL_4 (0x04 << 0)
#define CLKSEL_5 (0x05 << 0)
#define CLKSEL_6_RES (0x06 << 0)
#define CLKSEL_7 (0x07 << 0)
#define EXT_SYNC_SET_0 (0x00 << 3)
#define EXT_SYNC_SET_1 (0x01 << 3)
#define EXT_SYNC_SET_2 (0x02 << 3)
#define EXT_SYNC_SET_3 (0x03 << 3)
#define EXT_SYNC_SET_4 (0x04 << 3)
#define EXT_SYNC_SET_5 (0x05 << 3)
#define EXT_SYNC_SET_6 (0x06 << 3)
#define EXT_SYNC_SET_7 (0x07 << 3)
#define DLPF_CFG_0 (0x00 << 0)
#define DLPF_CFG_1 (0x01 << 0)
#define DLPF_CFG_2 (0x02 << 0)
#define DLPF_CFG_3 (0x03 << 0)
#define DLPF_CFG_4 (0x04 << 0)
#define DLPF_CFG_5 (0x05 << 0)
#define DLPF_CFG_6 (0x06 << 0)
#define DLPF_CFG_7 (0x07 << 0)
#define SMPLRT_DIV 0x07

// VARIABLES

typedef struct {
	int16_t Accel_X_RAW;
	int16_t Accel_Y_RAW;
	int16_t Accel_Z_RAW;
	int16_t Temperature_RAW;
	int16_t Gyro_X_RAW;
	int16_t Gyro_Y_RAW;
	int16_t Gyro_Z_RAW;

	float Accel_X;
	float Accel_Y;
	float Accel_Z;
	float Temperature;
	float Gyro_X;
	float Gyro_Y;
	float Gyro_Z;

	float Pitch;
	float Roll;
	float Yaw;

	float PitchAcc;
	float RollAcc;
	float YawAcc;

	float P_Comp_Coeff;
	float R_Comp_Coeff;
	float Y_Comp_Coeff;

	float Accel_X_Offset;
	float Accel_Y_Offset;
	float Accel_Z_Offset;
	float Gyro_X_Offset;
	float Gyro_Y_Offset;
	float Gyro_Z_Offset;

} MPU6050_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx, uint8_t fs_sel, uint8_t afs_sel);
void MPU6050_Calibrate(I2C_HandleTypeDef *I2Cx, MPU6050_t *mpu);
void MPU6050_ReadAccel(I2C_HandleTypeDef *I2Cx, MPU6050_t *mpu);
void MPU6050_ReadGyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *mpu);
void MPU6050_ReadTemperature(I2C_HandleTypeDef *I2Cx, MPU6050_t *mpu);
void MPU6050_Read(I2C_HandleTypeDef *I2Cx, MPU6050_t *mpu);
void MPU6050_ConvertRawValues(MPU6050_t *mpu);
void MPU6050_ZeroAll(MPU6050_t *mpu);
void MPU6050_ComplementaryFilter(MPU6050_t *mpu, uint32_t delta_t_us);

#ifdef __cplusplus
}
#endif

#endif /* INC_MPU6050_H_ */
